/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Valeri Atamaniouk <valeri@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "track_profiles.h"
#include "track_profile_utils.h"

#include <libswiftnav/constants.h>
#include <libswiftnav/track.h>

#include <board.h>
#include <signal.h>
#include <chconf_board.h>
#include <platform_cn0.h>
#include <platform_track.h>
#include <nap/nap_common.h>
#include <nap/nap_hw.h>

#include <string.h>
#include <math.h>
#include <assert.h>

/** Maximum number of supported satellite vehicles */
#define TP_MAX_SUPPORTED_SVS NUM_TRACKER_CHANNELS

/** Default C/N0 threshold in dB/Hz for keeping track */
#define TP_DEFAULT_CN0_USE_THRESHOLD  (30.f)
/** Default C/N0 threshold in dB/Hz for dropping track (for 1 ms integration) */
#define TP_DEFAULT_CN0_DROP_THRESHOLD (31.f)

/** C/N0 threshold when we can't say if we are still tracking */
#define TP_HARD_CN0_DROP_THRESHOLD (18.f)

/** Revert acceleration flag, if last acceleration
   has been seen earlier than this time [ms] */
#define TP_ACCELERATION_MAX_AGE_MS (2000)

/** Unknown delay indicator */
#define TP_DELAY_UNKNOWN -1

/** Helper macro for array size computation */
#define ARR_SIZE(x) (sizeof(x)/sizeof((x)[0]))

/** Indices of specific entries in gps_profiles[] table below */
typedef enum {
  /** Placeholder for an index. Indicated an unused index field. */
  IDX_NONE         = -1,
  /** Recovery profiles initial index */
  IDX_RECOVERY     = 5,

  /** High CN0 range profiles initial index */
  IDX_HIGH_CN0     = 9,
  /** Middle CN0 range profiles initial index */
  IDX_MID_CN0      = 10,
  /** Low CN0 range profiles initial index */
  IDX_LOW_CN0_INI  = 11,
  /** Low CN0 range profiles final index */
  IDX_LOW_CN0_FIN  = 17,

  /** Sensitivity profile index */
  IDX_SENS         = 18,

  /** Sensitivity to low range CN0 transitional profiles index */
  IDX_TRAN_CN0     = 19,
  /** Sensitivity to dynamics transitional profiles index */
  IDX_TRAN_DYN     = 22,

  /** Dynamics profile for low CN0 index */
  IDX_LOW_CN0_DYN  = 24,
  /** Dynamics profile for low CN0 index */
  IDX_HIGH_CN0_DYN = 25
} profile_indices_t;

typedef enum {
  TP_LOW_CN0    = (1 << 0), /**< Watch low CN0 value */
  TP_HIGH_CN0   = (1 << 1), /**< Watch high CN0 value */
  TP_LOW_DYN    = (1 << 2), /**< Watch low dynamics */
  TP_HIGH_DYN   = (1 << 3), /**< Watch high dynamics */
  TP_NO_PLOCK   = (1 << 4), /**< Watch no pessimistic lock condition */
  TP_WAIT_BSYNC = (1 << 5), /**< Wait for bit sync */
  TP_WAIT_PLOCK = (1 << 6), /**< Wait for pessimistic lock */
  TP_WAIT_CN0   = (1 << 7), /**< Wait for CN0 to grow above a threshold */
  TP_USE_NEXT   = (1 << 8), /**< Use next index to choose next profile */

  /** Watch high CN0 value, once pessimistic lock is acquired and no dynamics */
  TP_HIGH_CN0_WAIT_PLOCK_N0_DYN = (1 << 9),

  /** Watch high dynamics, once pessimistic lock is acquired
      and CN0 value is above a threshold */
  TP_HIGH_DYN_WAIT_PLOCK_CN0 = (1 << 10)
} tp_profile_flags_t;

/** Tracking loop parameter placeholder */
#define TP_LOOP_PARAM_PLACE_HOLDER 0.f

/** Time interval in ms for printing channel statistics (when DEBUG is enabled)*/
#define DEBUG_PRINT_TIME_INTERVAL_MS (20000)

/**
 * Compact SID
 *
 * Compact version of SID. Requires 4 times less memory than sid_t.
 */
typedef union
{
  struct {
    s16 sat : 7;    /**< SV identifier [-128..+127] */
    s16 code : 5;   /**< Code [-1..+31]*/
    s16 _res : 4;   /**< Reserved */
  };
  s16 sv_id;        /**< Packed data for binary operations */
} tp_csid_t;

/** Describes single tracking profile */
typedef struct {
  struct {
    float pll_bw;    /**< PLL bandwidth [Hz] */
    float fll_bw;    /**< FLL bandwidth [Hz]  */
    float dll_bw;    /**< DLL bandwidth [Hz] */
    tp_ctrl_e controller_type;   /**< Controller type */
    tp_tm_e mode;                /**< Tracking mode */
    track_cn0_est_e cn0_est;     /**< CN0 estimator */
  } profile;

  u8 ld_params;             /**< One of TP_LD_PARAMS_... constants */

  u16 lock_time_ms;         /**< Profile stabilization time [ms] */
  float cn0_low_threshold;  /**< Low CN0 threshold [dB-Hz] */
  float cn0_high_threshold; /**< High CN0 threshold [dB-Hz] */
  float acc_threshold;      /**< Acceleration threshold [g] */
  float cn0_dyn_threshold;  /**< High CN0 dynamics threshold [dB-Hz] */

  /** Next profile to activate once lock_time_ms is over */
  profile_indices_t next;

  /** Next profile to activate if TP_LOW_CN0 is set and CN0 is
      lower than this value */
  profile_indices_t next_cn0_low;

  /** Next profile to activate if TP_HIGH_CN0 is set and CN0 is
      higher than this value */
  profile_indices_t next_cn0_high;

  /** Next profile to activate if:
      TP_HIGH_DYN is set and high dynamics was detected
      OR
      TP_LOW_DYN is set and low dynamics was detected */
  profile_indices_t next_dyn;

  /** Next profile to activate if TP_NO_PLOCK is set and
      pessimistic lock was lost */
  profile_indices_t next_lock;

  u16 flags;              /**< Bit combination of tp_profile_flags_t */
} tp_profile_entry_t;

/**
 * Per-satellite entry.
 *
 * The system keeps some tracking information for all satellites. Generally
 * the entry is per satellite vehicle, not per signal.
 *
 * TODO Make entry to support multiple bands.
 */
typedef struct {
  /*
   * Fields are ordered from larger to smaller for minimal memory footprint.
   */

  float         cn0_offset;  /**< C/N0 offset in dB to tune thresholds */
  float         filt_cn0;    /**< C/N0 value for decision logic */
  float         filt_accel;  /**< SV acceleration value for decision logic [g] */

  u32           used: 1;           /**< Flag if the profile entry is in use */
  u32           olock: 1;          /**< PLL optimistic lock flag */
  u32           plock: 1;          /**< PLL pessimistic lock flag */
  u32           bsync: 1;          /**< Bit sync flag */
  u32           bsync_sticky: 1;   /**< Bit sync flag */
  u32           profile_update:1;  /**< Flag if the profile update is required */
  u32           low_cn0_count:5;   /**< State lock counter for C/N0 threshold */
  u32           high_cn0_count:5;  /**< State lock counter for C/N0 threshold */
  u32           accel_count:5;     /**< State lock counter for dynamics threshold */
  u32           accel_count_idx:2; /**< State lock value for dynamics threshold */
  u16           lock_time_ms:12;   /**< Profile lock count down timer */
  u16           cn0_est:2;

  tp_csid_t     csid;              /**< Compact satellite identifier */

  /** There is an acceleration if this parameter is non-zero [ms] */
  u16           acceleration_ends_after_ms;
  u16           print_time;        /**< Last debug print time */

  u32           time_snapshot_ms;  /**< Time snapshot [ms] */

  /** Bit sync delay [ms] or TP_DELAY_UNKNOWN */
  s16           bs_delay_ms;

  /** Pessimistic lock delay [ms] or TP_DELAY_UNKNOWN */
  s16           plock_delay_ms;

  /** Profiles switching table. */
  const tp_profile_entry_t* profiles;
  u8  cur_index;  /**< Active profile index */
  u8  next_index; /**< Next profile index */
} tp_profile_internal_t;

/**
 * GPS satellite profiles.
 */
static tp_profile_internal_t profiles_gps1[TP_MAX_SUPPORTED_SVS]
                                           PLATFORM_TRACK_DATA_PROFILES;

/**
 * C/N0 profile
 */
static const tp_cn0_params_t cn0_params_default = {
  .track_cn0_drop_thres = TP_DEFAULT_CN0_DROP_THRESHOLD,
  .track_cn0_use_thres = TP_DEFAULT_CN0_USE_THRESHOLD
};

/**
 * Converts compact SID into GNSS SID.
 *
 * \param[in] csid Compact SID
 *
 * \return GNSS SID
 */
static gnss_signal_t unpack_sid(tp_csid_t csid)
{
  gnss_signal_t res = {
    .sat = csid.sat,
    .code = (code_t)csid.code
  };
  return res;
}

/**
 * Converts GNSS SID into compact SID.
 *
 * \param[in] csid GNSS SID
 *
 * \return Compact SID
 */
static tp_csid_t pack_sid(gnss_signal_t sid)
{
  tp_csid_t res = {
    {
      .sat = (u16)sid.sat,
      .code = (s16)sid.code,
      ._res = 0
    }
  };
  return res;
}

/**
 * Lock detector parameters
 */
enum {
  TP_LD_PARAMS_DISABLE,
  TP_LD_PARAMS_EXTRAOPT,
  TP_LD_PARAMS_OPT,
  TP_LD_PARAMS_NORMAL,
  TP_LD_PARAMS_PESS,
  TP_LD_PARAMS_PLL_1MS,
  TP_LD_PARAMS_PLL_5MS,
  TP_LD_PARAMS_FLL_5MS
};

/**
 * Lock detector profiles
 */
static const tp_lock_detect_params_t ld_params[] = {
  /*   k1,    k2,  lp,  lo */
  { 0.02f, 1e-6f,   1,   1,}, /* TP_LD_PARAMS_DISABLE */
  { 0.02f,  0.8f,  50, 150,}, /* TP_LD_PARAMS_EXTRAOPT */
  { 0.02f,  1.1f,  50, 150,}, /* TP_LD_PARAMS_OPT */
  { 0.05f,  1.4f,  50, 150,}, /* TP_LD_PARAMS_NORMAL */
  { 0.10f,  1.4f,  50, 200,}, /* TP_LD_PARAMS_PESS */
  { 0.025f,  1.5f,  50, 150,}, /* TP_LD_PARAMS_PLL_1MS */
  { 0.025f,  1.5f,  50, 150,}, /* TP_LD_PARAMS_PLL_5MS */
  { 0.005f,  .6f,  50, 200,}  /* TP_LD_PARAMS_FLL_5MS */
};

/** Set this to 'true' to enable debugging of profile switching
    logic. Another useful change is to limit the number of
    tracking channels to 1 to avoid flooding the resulting log.
    To do this, set NAP_MAX_N_TRACK_CHANNELS to 1 in nap_hw.h */
static bool debug_profile_switching = false;

/** Tracking loop parameters template
 * Filled out at the trackig loop parameters switch event.
 */
static const tp_loop_params_t loop_params_template = {
  /** Code tracking noise bandwidth in Hz */
  .code_bw           = TP_LOOP_PARAM_PLACE_HOLDER,
  .code_zeta         = 0.707f,  /**< Code tracking loop damping ratio */
  .code_k            = 1.,      /**< Code tracking loop gain coefficient */
  /** carrier frequency /  chip rate */
  .carr_to_code      = TP_LOOP_PARAM_PLACE_HOLDER,
  /** Carrier tracking loop noise bandwidth in Hz */
  .carr_bw           = TP_LOOP_PARAM_PLACE_HOLDER,
  .carr_zeta         = 0.707f,  /**< Carrier tracking loop damping ratio */
  .carr_k            = 1,       /**< Carrier tracking loop gain coefficient */
  /** FLL noise bandwidth in Hz */
  .fll_bw            = TP_LOOP_PARAM_PLACE_HOLDER
};

/**
 * The tracking profiles switching table.
 *
 * The table describes a set of different profiles and
 * the logic controlling how different profiles are selected.
 * One entry of the table is one distinct profile, which targets
 * a specific condition. For example, low CN0, high acceleration,
 * transitional profile activated for a short time, while transitioning
 * to a target profile etc.
 *
 * Essentially, the table describes a finite state machine (FSM).
 * Each tracking channel has its own instance of the FSM.
 * Therefore, all tracking channels are independent and may have
 * different profiles active at any moment of time.
 * The actual profile switching is done in #check_for_profile_change()
 * function.
 *
 * The transition within the table happens as a result of
 * continous evaluation of four parameters:
 *
 * -# time
 * -# CN0 level
 * -# dynamics (acceleration)
 * -# availability of pessimistic lock
 *
 * Each profile has the field tp_profile_entry_t::flags, which
 * tells, which parameters affect the transition to a next profile.
 * The flags are a bit combination of tp_profile_flags_t.
 *
 * See more details at
 * https://swiftnav.hackpad.com/High-sensitivity-tracking-FLL-PLL-profile-switching-design-HDpuFC1BygA
 * Each entry of the array is a set of initialization parameters of
 * tp_profile_entry_t struct.
 */
static const tp_profile_entry_t gps_profiles[] = {
/*
  These are the short names of the numbers & parameters listed
  in the same order below.
  { { pll_bw,     fll_bw,        dll_bw,     controller,
            tracking mode,                      cn0_est },
      time,   cn0_low_thr, cn0_high_thr,        acc_thr,              ld_params,
      next,       cn0_low,     cn0_high,            dyn,                   lock,
     flags }
*/

  /* initial profiles */
  { {   40,             3,            1,   TP_CTRL_PLL3,
        TP_TM_GPS_INITIAL,      TRACK_CN0_EST_SECONDARY }, TP_LD_PARAMS_PLL_1MS,
        50,             0,            0,              0,                      0,
      IDX_NONE,  IDX_NONE,     IDX_NONE,       IDX_NONE,               IDX_NONE,
        0 },                                                             /* 0 */

  { {   40,             1,            1,   TP_CTRL_PLL3,
        TP_TM_GPS_INITIAL,      TRACK_CN0_EST_SECONDARY }, TP_LD_PARAMS_PLL_1MS,
        50,             0,            0,              0,                      0,
      IDX_NONE,  IDX_NONE,    IDX_NONE,       IDX_NONE,                IDX_NONE,
      TP_WAIT_BSYNC | TP_WAIT_PLOCK },                                   /* 1 */

  { {   40,             0,            1,   TP_CTRL_PLL3,
        TP_TM_GPS_INITIAL,      TRACK_CN0_EST_SECONDARY }, TP_LD_PARAMS_PLL_1MS,
        50,             0,            0,              0,                      0,
      IDX_NONE,  IDX_NONE,     IDX_NONE,       IDX_NONE,               IDX_SENS,
      TP_NO_PLOCK },                                                     /* 2 */

  { {   35,             0,            1,   TP_CTRL_PLL3,
        TP_TM_GPS_INITIAL,      TRACK_CN0_EST_SECONDARY }, TP_LD_PARAMS_PLL_1MS,
        50,             0,            0,              0,                      0,
       IDX_NONE, IDX_NONE,     IDX_NONE,       IDX_NONE,               IDX_SENS,
      TP_NO_PLOCK },                                                     /* 3 */

  { {   30,             0,            1,   TP_CTRL_PLL3,
        TP_TM_GPS_INITIAL,      TRACK_CN0_EST_SECONDARY }, TP_LD_PARAMS_PLL_1MS,
        50,             0,            0,              4,                      0,
      IDX_RECOVERY, IDX_NONE,  IDX_NONE, IDX_HIGH_CN0_DYN,             IDX_SENS,
      TP_NO_PLOCK | TP_HIGH_DYN | TP_USE_NEXT },                         /* 4 */

  /* recovery profiles */
  { {   30,             0,            1,   TP_CTRL_PLL3,
            TP_TM_GPS_5MS,        TRACK_CN0_EST_PRIMARY }, TP_LD_PARAMS_PLL_5MS,
        50,             0,            0,             2.,                      0,
      IDX_NONE,  IDX_NONE,     IDX_NONE, IDX_HIGH_CN0_DYN,             IDX_SENS,
      TP_NO_PLOCK | TP_HIGH_DYN },                                       /* 5 */

  { {   25,             0,            1,   TP_CTRL_PLL3,
            TP_TM_GPS_5MS,        TRACK_CN0_EST_PRIMARY }, TP_LD_PARAMS_PLL_5MS,
        50,             0,            0,             2.,                      0,
      IDX_NONE,  IDX_NONE,     IDX_NONE, IDX_HIGH_CN0_DYN,             IDX_SENS,
      TP_NO_PLOCK | TP_HIGH_DYN },                                       /* 6 */

  { {   20,             0,            1,   TP_CTRL_PLL3,
            TP_TM_GPS_5MS,        TRACK_CN0_EST_PRIMARY }, TP_LD_PARAMS_PLL_5MS,
        50,             0,            0,             2.,                      0,
      IDX_NONE,  IDX_NONE,     IDX_NONE, IDX_HIGH_CN0_DYN,             IDX_SENS,
      TP_NO_PLOCK | TP_HIGH_DYN },                                       /* 7 */

  { {   20,             0,            1,   TP_CTRL_PLL3,
            TP_TM_GPS_5MS,        TRACK_CN0_EST_PRIMARY }, TP_LD_PARAMS_PLL_5MS,
        50,             0,            0,             2.,                      0,
      IDX_NONE,  IDX_NONE,     IDX_NONE, IDX_HIGH_CN0_DYN,             IDX_SENS,
      TP_NO_PLOCK | TP_HIGH_DYN },                                       /* 8 */

  /* high range CN0 profile */
  { {   18,             0,            1,   TP_CTRL_PLL3,
            TP_TM_GPS_5MS,        TRACK_CN0_EST_PRIMARY }, TP_LD_PARAMS_PLL_5MS,
        50,             0,            0,             2.,                      0,
      IDX_NONE,  IDX_NONE,   IDX_NONE, IDX_HIGH_CN0_DYN,               IDX_SENS,
      TP_NO_PLOCK | TP_HIGH_DYN },                                       /* 9 */

  /* middle range CN0 profile */
  { {   18,             0,           .5,   TP_CTRL_PLL3,
           TP_TM_GPS_10MS,        TRACK_CN0_EST_PRIMARY }, TP_LD_PARAMS_PLL_5MS,
        50,             0,            0,            1.5,                      0,
      IDX_NONE,  IDX_NONE,     IDX_NONE, IDX_LOW_CN0_DYN,              IDX_SENS,
      TP_NO_PLOCK | TP_HIGH_DYN },                                      /* 10 */

  /* low range CN0 profiles */
  { {   15,             0,           .5,   TP_CTRL_PLL3,
           TP_TM_GPS_10MS,      TRACK_CN0_EST_PRIMARY },   TP_LD_PARAMS_PLL_5MS,
        50,             0,            0,            1.5,                      0,
      IDX_NONE,  IDX_NONE,     IDX_NONE, IDX_LOW_CN0_DYN,              IDX_SENS,
      TP_NO_PLOCK | TP_HIGH_DYN },                                      /* 11 */

  { {   14,             0,           .5,   TP_CTRL_PLL3,
           TP_TM_GPS_10MS,      TRACK_CN0_EST_PRIMARY },   TP_LD_PARAMS_PLL_5MS,
        50,             0,            0,            1.5,                      0,
      IDX_NONE,  IDX_NONE,     IDX_NONE, IDX_LOW_CN0_DYN,              IDX_SENS,
      TP_NO_PLOCK | TP_HIGH_DYN },                                      /* 12 */

  { {   12,             0,           .5,   TP_CTRL_PLL3,
           TP_TM_GPS_10MS,      TRACK_CN0_EST_PRIMARY },   TP_LD_PARAMS_PLL_5MS,
        50,             0,            0,            1.5,                      0,
      IDX_NONE,  IDX_NONE,     IDX_NONE, IDX_LOW_CN0_DYN,              IDX_SENS,
      TP_NO_PLOCK | TP_HIGH_DYN },                                      /* 13 */

  { {   10,             0,           .5,   TP_CTRL_PLL3,
           TP_TM_GPS_10MS,      TRACK_CN0_EST_PRIMARY },   TP_LD_PARAMS_PLL_5MS,
        50,             0,            0,            1.5,                      0,
      IDX_NONE,  IDX_NONE,     IDX_NONE, IDX_LOW_CN0_DYN,              IDX_SENS,
      TP_NO_PLOCK | TP_HIGH_DYN },                                      /* 14 */

  { {   10,             0,           .5,   TP_CTRL_PLL3,
           TP_TM_GPS_20MS,      TRACK_CN0_EST_PRIMARY },   TP_LD_PARAMS_PLL_5MS,
        50,             0,            0,            1.5,                      0,
      IDX_NONE,  IDX_NONE,     IDX_NONE, IDX_LOW_CN0_DYN,              IDX_SENS,
      TP_NO_PLOCK | TP_HIGH_DYN },                                      /* 15 */

  { {    8,             0,           .5,   TP_CTRL_PLL3,
           TP_TM_GPS_20MS,      TRACK_CN0_EST_PRIMARY },   TP_LD_PARAMS_PLL_5MS,
        50,             0,            0,            1.5,                      0,
      IDX_NONE,  IDX_NONE,     IDX_NONE, IDX_LOW_CN0_DYN,              IDX_SENS,
      TP_NO_PLOCK | TP_HIGH_DYN },                                      /* 16 */

  { {    7,             0,           .5,   TP_CTRL_PLL3,
           TP_TM_GPS_20MS,      TRACK_CN0_EST_PRIMARY },   TP_LD_PARAMS_PLL_5MS,
        50,             0,            0,            1.5,                      0,
      IDX_LOW_CN0_FIN, IDX_NONE, IDX_NONE, IDX_LOW_CN0_DYN,            IDX_SENS,
      TP_NO_PLOCK | TP_USE_NEXT | TP_HIGH_DYN },                        /* 17 */

  /* sensitivity profiles */
  { {    4,             3,            1,   TP_CTRL_FLL2,
           TP_TM_GPS_20MS,    TRACK_CN0_EST_SECONDARY },   TP_LD_PARAMS_FLL_5MS,
      4000,             0,          32.,            1.5,                    32.,
      IDX_SENS,  IDX_NONE, IDX_TRAN_CN0,   IDX_TRAN_DYN,               IDX_NONE,
      TP_HIGH_CN0_WAIT_PLOCK_N0_DYN | TP_HIGH_DYN_WAIT_PLOCK_CN0 |
      TP_WAIT_CN0 | TP_USE_NEXT },                                      /* 18 */

  /* sensitivity to low range CN0 transitional profiles  */
  { {   20,             1,            1,   TP_CTRL_PLL3,
           TP_TM_GPS_10MS,    TRACK_CN0_EST_SECONDARY },   TP_LD_PARAMS_PLL_5MS,
        50,           26.,            0,              0,                      0,
      IDX_NONE,  IDX_SENS,     IDX_NONE,       IDX_NONE,               IDX_NONE,
      TP_LOW_CN0 | TP_WAIT_PLOCK },                                     /* 19 */

  { {   20,             0,            1,   TP_CTRL_PLL3,
           TP_TM_GPS_10MS,    TRACK_CN0_EST_SECONDARY },   TP_LD_PARAMS_PLL_5MS,
        50,           26.,            0,            1.5,                      0,
      IDX_NONE,  IDX_SENS,     IDX_NONE, IDX_LOW_CN0_DYN,              IDX_NONE,
      TP_LOW_CN0 | TP_HIGH_DYN },                                       /* 20 */

  { {   18,             0,            1,   TP_CTRL_PLL3,
           TP_TM_GPS_10MS,    TRACK_CN0_EST_SECONDARY },   TP_LD_PARAMS_PLL_5MS,
        50,           26.,            0,            1.5,                      0,
      IDX_LOW_CN0_INI, IDX_SENS, IDX_NONE, IDX_LOW_CN0_DYN,            IDX_NONE,
      TP_LOW_CN0 | TP_HIGH_DYN | TP_USE_NEXT },                         /* 21 */

  /* sensitivity to dynamics transitional profiles */
  { {   30,             3,            1,   TP_CTRL_PLL3,
            TP_TM_GPS_5MS,    TRACK_CN0_EST_SECONDARY },   TP_LD_PARAMS_PLL_5MS,
        50,           30.,            0,              0,                      0,
    IDX_NONE,    IDX_SENS,     IDX_NONE,       IDX_NONE,               IDX_NONE,
      TP_LOW_CN0 | TP_WAIT_PLOCK },                                     /* 22 */

  { {   30,             1,            1,   TP_CTRL_PLL3,
            TP_TM_GPS_5MS,      TRACK_CN0_EST_SECONDARY }, TP_LD_PARAMS_PLL_5MS,
        50,           30.,            0,              0,                      0,
    IDX_LOW_CN0_DYN, IDX_SENS, IDX_NONE,       IDX_NONE,               IDX_NONE,
      TP_LOW_CN0 | TP_USE_NEXT },                                       /* 23 */

  /* dynamics profile for low CN0 */
  { {   30,             0,            1,   TP_CTRL_PLL3,
            TP_TM_GPS_5MS,      TRACK_CN0_EST_SECONDARY }, TP_LD_PARAMS_PLL_5MS,
        50,           30.,          41.,            1.0,                      0,
   IDX_LOW_CN0_DYN, IDX_SENS, IDX_HIGH_CN0_DYN, IDX_RECOVERY,          IDX_SENS,
      TP_NO_PLOCK | TP_LOW_CN0 | TP_HIGH_CN0 | TP_LOW_DYN |
      TP_USE_NEXT },                                                    /* 24 */

  /* dynamics profile for high CN0 */
  { {   30,             0,            1,   TP_CTRL_PLL3,
            TP_TM_GPS_DYN,      TRACK_CN0_EST_SECONDARY }, TP_LD_PARAMS_PLL_5MS,
        50,             38,           0,              1,                      0,
   IDX_HIGH_CN0_DYN, IDX_LOW_CN0_DYN, IDX_NONE, IDX_RECOVERY,          IDX_SENS,
      TP_NO_PLOCK | TP_LOW_CN0 | TP_LOW_DYN | TP_USE_NEXT }             /* 25 */
};

/**
 * Helper method for get tracking profile parameters.
 *
 * \param[in] sid SV identifier
 * \return tracking parameters structures array pointer
 */
static const tp_profile_entry_t* tp_profiles_from_id(gnss_signal_t sid)
{
  const tp_profile_entry_t *result = NULL;

  /* GPS and SBAS constellations use similar signal encoding scheme and thus
   * the tracker state machines are compatible.  */
  /* GLONASS constellation require different state machine due to different
   * data bit encoding and frame structure. */

  switch (sid_to_constellation(sid)) {
  case CONSTELLATION_GPS:
  case CONSTELLATION_SBAS:
    result = gps_profiles;
    break;

  case CONSTELLATION_GLO:
    assert(!"Unsupported constellation");
    break;

  case CONSTELLATION_INVALID:
  case CONSTELLATION_COUNT:
  default:
    assert(!"Invalid constellation");
    break;
  }

  return result;
}

/**
 * Allocates a new tracking profile structure for a satellite.
 *
 * Profiles identify physical GNSS satellites, not their signals.
 *
 * \param[in] sid  GNSS satellite signal identifier.
 *
 * \return Allocated profile pointer.
 * \retval NULL on error.
 */
static tp_profile_internal_t *allocate_profile(gnss_signal_t sid)
{
  size_t i;
  tp_profile_internal_t *res = NULL;

  /* Find unused entry */
  for (i = 0; i< TP_MAX_SUPPORTED_SVS; ++i) {
    if (!profiles_gps1[i].used) {
      res = &profiles_gps1[i];
      break;
    }
  }

  /* If unused entry is found, mark it allocated and make default
   * initialization */
  if (NULL != res) {
    res->used = true;
    res->csid = pack_sid(sid);
  }
  return res;
}

/**
 * Locates profile for a given GNSS signal identifier.
 *
 * \param[in] sig GNSS satellite signal identifier.
 *
 * \return Allocated profile pointer.
 * \retval NULL on error.
 */
static tp_profile_internal_t *find_profile(gnss_signal_t sid)
{
  size_t i;
  tp_profile_internal_t *res = NULL;
  tp_csid_t csid = pack_sid(sid);

  for (i = 0; i< TP_MAX_SUPPORTED_SVS; ++i) {
    if (profiles_gps1[i].used &&
        profiles_gps1[i].csid.sv_id == csid.sv_id) {
      res = &profiles_gps1[i];
      break;
    }
  }
  return res;
}

/**
 * Marks the signal as not tracked.
 *
 * The method locates profile for a signal and marks it as not tracked. If
 * necessary the profile is released.
 *
 * \param[in] sig GNSS satellite signal identifier.
 *
 * \return None
 */
static void delete_profile(gnss_signal_t sid)
{
  tp_profile_internal_t *profile = find_profile(sid);
  if (NULL != profile) {
    /* Currently we support only one signal in profile, so simply mark the
     * profile as released. */
    memset(profile, 0, sizeof(*profile));
  }
}

/**
 * Helper method to obtain tracking loop parameters.
 *
 * The method generates tracking loop parameters according to selected
 * configuration.
 *
 * \param[in]  profile GNSS satellite profile.
 * \param[out] config  Container for computed configuration.
 *
 * \return None
 */
static void get_profile_params(tp_profile_internal_t *profile,
                               tp_config_t           *config)
{
  const tp_profile_entry_t *cur_profile = &profile->profiles[profile->cur_index];
  double carr_to_code = code_to_carr_to_code(profile->csid.code);
  config->lock_detect_params = ld_params[cur_profile->ld_params];

  /* fill out the tracking loop parameters */
  config->loop_params = loop_params_template;
  config->loop_params.carr_to_code = carr_to_code;
  config->loop_params.code_bw = cur_profile->profile.dll_bw;
  config->loop_params.carr_bw = cur_profile->profile.pll_bw;
  config->loop_params.fll_bw = cur_profile->profile.fll_bw;
  config->loop_params.mode = cur_profile->profile.mode;
  config->loop_params.ctrl = cur_profile->profile.controller_type;

  /*
   * Alias detection is requires bit-aligned integration accumulator with equal
   * intervals.
   * The logic works with PLL in modes:
   * - 1+N modes with 5 and 10 ms.
   * - 1+N5 and 1+N10.
   */
  const tp_tm_e mode = config->loop_params.mode;
  const tp_ctrl_e ctrl = config->loop_params.ctrl;
  if ((TP_TM_GPS_5MS == mode || TP_TM_GPS_10MS == mode || TP_TM_GPS_20MS == mode)
      && (TP_CTRL_PLL2 == ctrl || TP_CTRL_PLL3 == ctrl)) {
    config->use_alias_detection = true;
  } else {
    config->use_alias_detection = false;
  }

  tp_get_cn0_params(unpack_sid(profile->csid), &config->cn0_params);
}

/**
 * Helper method to incorporate tracking loop information into statistics.
 *
 * \param[in,out] profile Satellite profile.
 * \param[in]     common_data Tracker common data
 * \param[in]     data    Data from tracking loop.
 *
 * \return None
 */
static void update_stats(tp_profile_internal_t *profile,
                         const tracker_common_data_t *common_data,
                         const tp_report_t *data)
{
  float cn0;
  u32 cur_time_ms = common_data->update_count;

  /* Profile lock time count down */
  if (profile->lock_time_ms > data->time_ms) {
    profile->lock_time_ms -= data->time_ms;
  } else {
    profile->lock_time_ms = 0;
  }

  /* Acceleration state lock time count down */
  if (profile->acceleration_ends_after_ms >= data->time_ms) {
    profile->acceleration_ends_after_ms -= data->time_ms;
  } else {
    profile->acceleration_ends_after_ms = 0;
  }

  /* Debug print interval count down */
  if (profile->print_time >= data->time_ms) {
    profile->print_time -= data->time_ms;
  } else {
    profile->print_time = 0;
  }

  if ((TP_DELAY_UNKNOWN == profile->bs_delay_ms) && data->bsync) {
    /* just got bit sync */
    profile->bs_delay_ms = cur_time_ms - profile->time_snapshot_ms;
  }

  if (TP_DELAY_UNKNOWN == profile->plock_delay_ms && data->plock) {
    /* just got pessimistic lock */
    profile->plock_delay_ms = cur_time_ms - profile->time_snapshot_ms;
  }

  if (TP_DELAY_UNKNOWN != profile->plock_delay_ms && !data->plock) {
    /* just lost pessimistic lock */
    profile->plock_delay_ms = TP_DELAY_UNKNOWN;
    profile->time_snapshot_ms = cur_time_ms;
  }

  profile->olock = data->olock;
  profile->plock = data->plock;
  profile->bsync_sticky |= data->bsync;

  cn0   = data->cn0;

  profile->filt_cn0 = cn0;

  float carr_freq = code_to_carr_freq(profile->csid.code);
  float acceleration_g = data->acceleration *
              (float) (GPS_C / STD_GRAVITY_ACCELERATION) / carr_freq;

  profile->filt_accel = acceleration_g;
}

/**
 * Internal helper for naming loop controller types.
 *
 * \param[in] v Loop controller type.
 *
 * \return Loop controller type literal.
 */
static const char *get_ctrl_str(tp_ctrl_e v)
{
  const char *str = "?";
  switch (v) {
  case TP_CTRL_PLL2: str = "PLL2"; break;
  case TP_CTRL_PLL3: str = "PLL3"; break;
  case TP_CTRL_FLL1: str = "FLL1"; break;
  case TP_CTRL_FLL2: str = "FLL2"; break;
  default: assert(!"Unknown loop controller type");
  }
  return str;
}

/**
 * Used to debug the profile switching logic.
 *
 * Set 'debug_profile_switching' to true to enable the profile
 * switching logging.
 * \param[in] state tracking loop state
 * \param[in] reason profile switching reason in a textual form
 */
static void log_switch(const tp_profile_internal_t *state, const char* reason)
{
  if (false == debug_profile_switching) {
    return;
  }

  const tp_profile_entry_t* cur_profile = &state->profiles[state->cur_index];
  const tp_profile_entry_t* next_profile = &state->profiles[state->next_index];

  log_info_sid(unpack_sid(state->csid),
    "%s: plock=%" PRId16 " bs=%" PRId16 " cn0=%.1f acc=%.1fg \
(mode,pll,fll,ctlr): (%s,%.1f,%.1f,%s)->(%s,%.1f,%.1f,%s)",
    reason,

    state->plock_delay_ms,
    state->bs_delay_ms,

    state->filt_cn0,
    state->filt_accel,

    tp_get_mode_str(cur_profile->profile.mode),
    cur_profile->profile.pll_bw,
    cur_profile->profile.fll_bw,
    get_ctrl_str(cur_profile->profile.controller_type),

    tp_get_mode_str(next_profile->profile.mode),
    next_profile->profile.pll_bw,
    next_profile->profile.fll_bw,
    get_ctrl_str(next_profile->profile.controller_type));
}

/**
 * Helper method to dump tracking statistics into log.
 *
 * The method logs average and RMS values for analyzes.
 *
 * \params[in] profile GNSS satellite profile
 *
 * \return None
 */
static void print_stats(tp_profile_internal_t *profile)
{
  if (profile->print_time > 0) {
    return;
  }

  profile->print_time = DEBUG_PRINT_TIME_INTERVAL_MS;

  const char *cn0_est_str = track_cn0_str(profile->cn0_est);

  const tp_profile_entry_t *cur_profile = &profile->profiles[profile->cur_index];
  tp_tm_e tracking_mode = cur_profile->profile.mode;
  int dll_ms = tp_get_dll_ms(tracking_mode);

  const char *m1 = tp_get_mode_str(tracking_mode);
  const char *c1 = get_ctrl_str(cur_profile->profile.controller_type);

  /*
   * PRINT: integration time, loop mode, controller mode,
   *        C/N0 estimator, C/N0 value, SNR value (dBm),
   *        PR rate, PR rate change,
   *        PLL lock detector ratio, FLL/DLL error
   */

  log_debug_sid(unpack_sid(profile->csid),
                "AVG: %dms %s %s CN0_%s=%.2f (%.2f) A=%.3f",
                dll_ms, m1, c1,
                cn0_est_str, profile->filt_cn0,
                TRACK_CN0_TO_SNR(profile->filt_cn0),
                profile->filt_accel
               );
}

/**
 * Detects the acceleration on/off condition.
 *
 * \params[in/out] state tracking loop state
 */
static void update_acceleration_status(tp_profile_internal_t *state)
{
  const tp_profile_entry_t *cur_profile = &state->profiles[state->cur_index];
  float acc_threshold_g = cur_profile->acc_threshold;
  float acceleration_g = fabsf(state->filt_accel);

  if ((acc_threshold_g > 0) && (acceleration_g > acc_threshold_g)) {
    state->acceleration_ends_after_ms = TP_ACCELERATION_MAX_AGE_MS;
  }
}

/**
 * Checks if CN0 estimator type needs to be changed
 *
 * \params[in/out] state tracking loop state
 */
static void check_for_cn0_estimator_change(tp_profile_internal_t *state)
{
  float cn0 = 0.f;
  const tp_profile_entry_t *cur_profile;

  if (TRACK_CN0_EST_PRIMARY == TRACK_CN0_EST_SECONDARY) {
    return;
  }

  cn0 = state->filt_cn0;
  cur_profile = &state->profiles[state->cur_index];
  u8 cn0_ms = tp_get_cn0_ms(cur_profile->profile.mode);

  if (TRACK_CN0_EST_PRIMARY == state->cn0_est) {
    if (cn0 < track_cn0_get_pri2sec_threshold(cn0_ms) ||
        TRACK_CN0_EST_SECONDARY == cur_profile->profile.cn0_est) {
      state->cn0_est = TRACK_CN0_EST_SECONDARY;
      log_debug_sid(unpack_sid(state->csid),
                    "Changed C/N0 estimator to secondary");
    }
  } else if (TRACK_CN0_EST_SECONDARY == state->cn0_est) {
    if (cn0 > track_cn0_get_sec2pri_threshold(cn0_ms) &&
        TRACK_CN0_EST_PRIMARY == cur_profile->profile.cn0_est) {
      state->cn0_est = TRACK_CN0_EST_PRIMARY;
      log_debug_sid(unpack_sid(state->csid),
                    "Changed C/N0 estimator to primary");
    }
  } else {
    assert(!"Unsupported CN0 estimator identifier");
  }
}

/**
 * Internal method for profile switch request.
 *
 * Sets the requested profile as the current one.
 *
 * \params[in/out] state tracking loop state
 * \params[in] index Index of profile to activate
 * \params[in] reason Textual reason of profile switch
 * \retval true Profile switch requested
 * \retval false No profile switch requested
 */
static bool profile_switch_requested(tp_profile_internal_t *state,
                                     profile_indices_t index,
                                     const char* reason)
{
  if (index == state->cur_index) {
    return false;
  }

  assert(index != IDX_NONE);
  assert((size_t)index < ARR_SIZE(gps_profiles));

  state->lock_time_ms = state->profiles[index].lock_time_ms;
  state->profile_update = true;
  state->next_index = index;

  log_switch(state, reason);

  return true;
}

/**
 * Internal method for evaluating profile change conditions.
 *
 * This method analyzes collected statistics and selects appropriate tracking
 * parameter changes.
 *
 * \params[in/out] state tracking loop state
 */
static void check_for_profile_change(tp_profile_internal_t *state)
{
  const tp_profile_entry_t *cur_profile;
  u16 flags;
  bool acceleration_detected;

  cur_profile = &state->profiles[state->cur_index];
  flags = cur_profile->flags;

  state->profile_update = false;

  check_for_cn0_estimator_change(state);

  update_acceleration_status(state);
  acceleration_detected = (0 != state->acceleration_ends_after_ms);

  if ((0 != (flags & TP_LOW_CN0)) &&
      (state->filt_cn0 < cur_profile->cn0_low_threshold) &&
      profile_switch_requested(state, cur_profile->next_cn0_low, "low cn0")) {
    return;
  }

  if ((0 != (flags & TP_NO_PLOCK)) && !state->plock &&
      profile_switch_requested(state, cur_profile->next_lock, "no plock")) {
    return;
  }

  if ((0 != (flags & TP_HIGH_DYN)) &&
       acceleration_detected &&
       profile_switch_requested(state, cur_profile->next_dyn, "high dyn")) {
    return;
  }

  if ((0 != (flags & TP_WAIT_BSYNC)) && !state->bsync_sticky) {
    return;
  }

  if (0 != (flags & TP_WAIT_PLOCK) && !state->plock) {
    return;
  }

  if (0 != (flags & TP_WAIT_CN0) &&
      (state->filt_cn0 <= cur_profile->cn0_high_threshold)) {
    return;
  }

  if (state->lock_time_ms > 0) {
    return; /* tracking loop has not settled yet */
  }

  if ((0 != (flags & TP_HIGH_DYN_WAIT_PLOCK_CN0)) &&
       acceleration_detected &&
       state->plock &&
       (state->filt_cn0 > cur_profile->cn0_dyn_threshold) &&
       profile_switch_requested(state, cur_profile->next_dyn, "high dyn")) {
    return;
  }

  if ((0 != (flags & TP_LOW_DYN)) &&
      !acceleration_detected &&
      profile_switch_requested(state, cur_profile->next_dyn, "low dyn")) {
    return;
  }

  if ((0 != (flags & TP_HIGH_CN0)) &&
      (state->filt_cn0 > cur_profile->cn0_high_threshold) &&
      profile_switch_requested(state, cur_profile->next_cn0_high, "high cno")) {
    return;
  }

  if ((0 != (flags & TP_HIGH_CN0_WAIT_PLOCK_N0_DYN)) &&
      state->plock &&
      !acceleration_detected &&
      (state->filt_cn0 > cur_profile->cn0_high_threshold) &&
      profile_switch_requested(state, cur_profile->next_cn0_high, "high cno")) {
    return;
  }

  if (0 != (flags & TP_USE_NEXT)) {
    assert(cur_profile->next != IDX_NONE);
    profile_switch_requested(state, cur_profile->next, "next");
  } else {
    profile_switch_requested(state, state->cur_index + 1, "next");
  }
}

/**
 * Helper method for computing C/N0 offset.
 *
 * The method computes C/N0 offset for tracking loop in accordance to
 * integration period and tracking parameters.
 *
 * \param[in] profile GNSS satellite profile
 *
 * \return Computed C/N0 offset in dB/Hz.
 */
static float compute_cn0_offset(const tp_profile_internal_t *profile)
{
  tp_tm_e mode = profile->profiles[profile->cur_index].profile.mode;

  u8 cn0_ms = tp_get_cn0_ms(mode);
  float cn0_offset = track_cn0_get_offset(cn0_ms);

  return cn0_offset;
}

/**
 * Initializes the subsystem.
 *
 * This method shall be invoked before any other methods from the subsystem.
 *
 * \return 0  On success.
 * \return -1 On error.
 */
tp_result_e tp_init(void)
{
  memset(profiles_gps1, 0, sizeof(profiles_gps1));

  return TP_RESULT_SUCCESS;
}

/**
 * Registers GNSS satellite in facility.
 *
 * The method registers GNSS signal and returns initial tracking parameters.
 *
 * \param[in]  sid    GNSS signal identifier.
 * \param[in]  data   Initial parameters.
 * \param[out] config Container for initial tracking parameters.
 *
 * \retval TP_RESULT_SUCCESS The satellite has been registered and initial
 *                           profile is returned.
 * \retval TP_RESULT_ERROR   On error.
 *
 * \sa tp_tracking_stop()
 */
tp_result_e tp_tracking_start(gnss_signal_t sid,
                              const tp_report_t *data,
                              tp_config_t *config)
{
  tp_result_e res = TP_RESULT_ERROR;

  if (NULL != config) {
    tp_profile_internal_t *profile = allocate_profile(sid);
    if (NULL != profile) {
      profile->filt_cn0 = data->cn0;
      profile->filt_accel = 0;

      profile->cur_index = 0;
      profile->profiles = tp_profiles_from_id(sid);
      profile->bsync_sticky = 0;

      profile->cn0_est = profile->profiles[profile->cur_index].profile.cn0_est;

      /* let's be pessimistic and assume, that tracking starts when
         receiver faces an acceleration */
      profile->acceleration_ends_after_ms = TP_ACCELERATION_MAX_AGE_MS;

      profile->profile_update = 0;

      profile->print_time = DEBUG_PRINT_TIME_INTERVAL_MS;

      profile->time_snapshot_ms = 0;

      profile->bs_delay_ms = TP_DELAY_UNKNOWN;
      profile->plock_delay_ms = TP_DELAY_UNKNOWN;

      get_profile_params(profile, config);

      res = TP_RESULT_SUCCESS;
    } else {
      log_error_sid(sid, "Can't allocate tracking profile");
    }
  }
  return res;
}

/**
 * Marks GNSS satellite as untracked.
 *
 * The method shall be invoked when tracking loop is terminated.
 *
 * \param[in] sid  GNSS signal identifier. This identifier must be registered
 *                 with a call to #tp_tracking_start().
 *
 * \retval TP_RESULT_SUCCESS On success.
 * \retval TP_RESULT_ERROR   On error.
 */
tp_result_e tp_tracking_stop(gnss_signal_t sid)
{
  tp_result_e res = TP_RESULT_ERROR;
  delete_profile(sid);
  res = TP_RESULT_SUCCESS;
  return res;
}

/**
 * Retrieves new tracking profile if available.
 *
 * \param[in]  sid    GNSS signal identifier. This identifier must be registered
 *                    with a call to #tp_tracking_start().
 * \param[out] config Container for new tracking parameters.
 * \param[in]  commit Commit the mode change happened.
 *
 * \retval TP_RESULT_SUCCESS New tracking profile has been retrieved. The
 *                           tracking loop shall reconfigure it's components
 *                           and, possibly, change the operation mode.
 * \retval TP_RESULT_NO_DATA New tracking profile is not available. No further
 *                           actions are needed.
 * \retval TP_RESULT_ERROR   On error.
 */
tp_result_e tp_get_profile(gnss_signal_t sid, tp_config_t *config, bool commit)
{
  tp_result_e res = TP_RESULT_ERROR;
  tp_profile_internal_t *profile = find_profile(sid);
  if (NULL != config && NULL != profile) {

    if (profile->profile_update) {
      /* Do transition of current profile */
      if (commit) {
        profile->profile_update = 0;

        profile->cur_index = profile->next_index;
        profile->cn0_offset = compute_cn0_offset(profile);
      }

      /* Return data */
      get_profile_params(profile, config);

      res = TP_RESULT_SUCCESS;
    } else {
      res = TP_RESULT_NO_DATA;
    }
  }
  return res;
}

/**
 * Method for obtaining current C/N0 thresholds.
 *
 * \param[in]  sid    GNSS signal identifier. This identifier must be registered
 *                    with a call to #tp_tracking_start().
 * \param[out] cn0_params Container for C/N0 limits.
 *
 * \retval TP_RESULT_SUCCESS C/N0 thresholds have been retrieved.
 * \retval TP_RESULT_ERROR   On error.
 */
tp_result_e tp_get_cn0_params(gnss_signal_t sid, tp_cn0_params_t *cn0_params)
{
  tp_result_e res = TP_RESULT_ERROR;
  tp_profile_internal_t *profile = find_profile(sid);
  if (NULL != cn0_params && NULL != profile) {
    *cn0_params = cn0_params_default;

    /* Correction: higher integration time lowers thresholds linearly. For
     * example, 20ms integration has threshold by 13 dB lower, than for 1ms
     * integration. */
    cn0_params->track_cn0_drop_thres -= profile->cn0_offset;

    cn0_params->est = (track_cn0_est_e)profile->cn0_est;

    if (cn0_params->track_cn0_drop_thres < TP_HARD_CN0_DROP_THRESHOLD) {
      cn0_params->track_cn0_drop_thres = TP_HARD_CN0_DROP_THRESHOLD;
    }
    if (cn0_params->track_cn0_use_thres < TP_HARD_CN0_DROP_THRESHOLD) {
      cn0_params->track_cn0_use_thres = TP_HARD_CN0_DROP_THRESHOLD;
    }
    res = TP_RESULT_SUCCESS;
  }
  return res;
}

/**
 * Method to check if there is a pending profile change.
 *
 * \param[in] sid GNSS satellite id.
 *
 * \retval true  New profile is available.
 * \retval false No profile change is required.
 */
bool tp_has_new_profile(gnss_signal_t sid)
{
  bool res = false;
  tp_profile_internal_t *profile = find_profile(sid);
  if (NULL != profile) {
    check_for_profile_change(profile);
    res = profile->profile_update != 0;
  }
  return res;
}

/**
 * Helper to obtain loop parameters for the next integration interval.
 *
 * \param[in] sid GNSS satellite id.
 *
 * \return Loop parameters for the next integration interval
 */
u8 tp_get_next_loop_params_ms(gnss_signal_t sid)
{
  tp_profile_internal_t *profile = find_profile(sid);
  u8 ms = 1;

  if (NULL != profile) {
    ms = tp_get_dll_ms(profile->profiles[profile->next_index].profile.mode);
  }

  return ms;
}

/**
 * Updates track profile data with supplied information.
 *
 * The method takes tracking loop data and merges it with previously collected
 * information from other tracking loops.
 *
 * \param[in] sid  GNSS signal identifier. This identifier must be registered
 *                 with a call to #tp_tracking_start().
 * \param[in] common_data Tracker common data
 * \param[in] data Tracking loop report. This data is taken for analysis and
 *                 can be asynchronously.
 *
 * \retval TP_RESULT_SUCCESS on success.
 * \retval TP_RESULT_ERROR   on error.
 */
tp_result_e tp_report_data(gnss_signal_t sid,
                           const tracker_common_data_t *common_data,
                           const tp_report_t *data)
{
  tp_result_e res = TP_RESULT_ERROR;
  tp_profile_internal_t *profile = find_profile(sid);
  if (NULL != data && NULL != profile && NULL != common_data) {
    /* For now, we support only GPS L1 tracking data, and handle all data
     * synchronously.
     *
     * TODO schedule a message to own thread.
     */

    update_stats(profile, common_data, data);
    print_stats(profile);

    res = TP_RESULT_SUCCESS;
  }
  return res;
}
