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

#include "track.h"

#include <libswiftnav/constants.h>
#include <libswiftnav/track.h>

#include <board.h>
#include <chconf_board.h>
#include <nap/nap_common.h>
#include <nap/nap_hw.h>
#include <platform_cn0.h>
#include <platform_track.h>
#include <signal.h>

#include <assert.h>
#include <math.h>
#include <string.h>

/** C/N0 threshold when we can't say if we are still tracking */
#define TP_HARD_CN0_DROP_THRESHOLD_DBHZ (18.f)

/** Default C/N0 threshold in dB/Hz for bit polarity ambiguity */
#define TP_DEFAULT_CN0_AMBIGUITY_THRESHOLD_DBHZ (30.f)
/** Default C/N0 threshold in dB/Hz for dropping track (for 1 ms integration) */
#define TP_DEFAULT_CN0_DROP_THRESHOLD_DBHZ (31.f)
/** C/N0 threshold for measurements use */
/** +1 to make it slightly higher than the CN0 drop threshold to avoid
    any race condition */
#define TP_DEFAULT_CN0_USE_THRESHOLD_DBHZ (TP_HARD_CN0_DROP_THRESHOLD_DBHZ + 1)

/** Revert acceleration flag, if last acceleration
   has been seen earlier than this time [ms] */
#define TP_ACCELERATION_MAX_AGE_MS (2000)

/** Indices of specific entries in gnss_track_profiles[] table below */
typedef enum {
  /** Placeholder for an index. Indicates an unused index field. */
  IDX_NONE = -1,
  IDX_NONAME_0,
  IDX_NONAME_1,
  IDX_NONAME_2,
  IDX_NONAME_3,
  IDX_DLL_RECOVERY2,
  IDX_NONAME_23,
  IDX_NONAME_24,
  IDX_DLL_RECOVERY,
  IDX_NONAME_26,
  IDX_NONAME_27,
  IDX_NONAME_28,
  IDX_NONAME_29,
  IDX_NONAME_30,
  IDX_NONAME_31,
  IDX_NONAME_32,
  IDX_VERY_HIGH_CN0,
  IDX_HIGH_CN0,
  IDX_MID_CN0,

  /** Low CN0 range profiles initial index */
  IDX_LOW_CN0_INI,
  IDX_NONAME_11,
  IDX_NONAME_12,
  IDX_NONAME_13,
  IDX_NONAME_14,
  IDX_NONAME_15,
  /** Low CN0 range profiles final index */
  IDX_LOW_CN0_FIN,

  /** Sensitivity profile index */
  IDX_SENS,

  /** Sensitivity to low range CN0 transitional profiles index */
  IDX_TRAN_CN0,

  IDX_NONAME_19,
  IDX_NONAME_20,

  /** Sensitivity to dynamics transitional profiles index */
  IDX_TRAN_DYN,

  IDX_NONAME_22,

  /** Dynamics profile for low CN0 index */
  IDX_LOW_CN0_DYN,
  /** Dynamics profile for low CN0 index */
  IDX_HIGH_CN0_DYN
} profile_indices_t;

typedef enum {
  TP_LOW_CN0 = (1 << 0),    /**< Watch low CN0 value */
  TP_HIGH_CN0 = (1 << 1),   /**< Watch high CN0 value */
  TP_LOW_DYN = (1 << 2),    /**< Watch low dynamics */
  TP_HIGH_DYN = (1 << 3),   /**< Watch high dynamics */
  TP_NO_PLOCK = (1 << 4),   /**< Watch no pessimistic lock condition */
  TP_WAIT_BSYNC = (1 << 5), /**< Wait for bit sync */
  TP_WAIT_PLOCK = (1 << 6), /**< Wait for pessimistic lock */
  TP_WAIT_CN0 = (1 << 7),   /**< Wait for CN0 to grow above a threshold */
  TP_USE_NEXT = (1 << 8),   /**< Use next index to choose next profile */

  /** Watch high CN0 value, once pessimistic lock is acquired and no dynamics */
  TP_HIGH_CN0_WAIT_PLOCK_N0_DYN = (1 << 9),

  /** Watch high dynamics, once pessimistic lock is acquired
      and CN0 value is above a threshold */
  TP_HIGH_DYN_WAIT_PLOCK_CN0 = (1 << 10),

  /** Do not use carrier aiding */
  TP_UNAIDED = (1 << 11)
} tp_profile_flags_t;

/** Tracking loop parameter placeholder */
#define TP_LOOP_PARAM_PLACE_HOLDER 0.f

/** Time interval in ms for printing channel statistics (when DEBUG is
 * enabled)*/
#define DEBUG_PRINT_TIME_INTERVAL_MS (20000)

/** Describes single tracking profile */
typedef struct tp_profile_entry {
  struct {
    float pll_bw;              /**< PLL bandwidth [Hz] */
    float fll_bw;              /**< FLL bandwidth [Hz]  */
    float dll_bw;              /**< DLL bandwidth [Hz] */
    tp_ctrl_e controller_type; /**< Controller type */
    tp_tm_e gps_track_mode;    /**< GPS Tracking mode */
    tp_tm_e glo_track_mode;    /**< GLO Tracking mode */
    track_cn0_est_e cn0_est;   /**< CN0 estimator */
  } profile;

  u8 ld_params; /**< One of TP_LD_PARAMS_... constants */

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

  u16 flags; /**< Bit combination of tp_profile_flags_t */
} tp_profile_entry_t;

/**
 * C/N0 profile
 */
static const tp_cn0_params_t cn0_params_default = {
    .track_cn0_drop_thres_dbhz = TP_DEFAULT_CN0_DROP_THRESHOLD_DBHZ,
    .track_cn0_use_thres_dbhz = TP_DEFAULT_CN0_USE_THRESHOLD_DBHZ,
    .track_cn0_ambiguity_thres_dbhz = TP_DEFAULT_CN0_AMBIGUITY_THRESHOLD_DBHZ};

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
  TP_LD_PARAMS_PLL_2MS,
  TP_LD_PARAMS_PLL_5MS,
  TP_LD_PARAMS_FLL_5MS
};

/**
 * Lock detector profiles
 */
static const tp_lock_detect_params_t ld_params[] = {
    /*   k1,    k2,  lp,  lo */
    {
        0.02f, 1e-6f, 1, 1,
    }, /* TP_LD_PARAMS_DISABLE */
    {
        0.02f, 0.8f, 50, 150,
    }, /* TP_LD_PARAMS_EXTRAOPT */
    {
        0.02f, 1.1f, 50, 150,
    }, /* TP_LD_PARAMS_OPT */
    {
        0.05f, 1.4f, 50, 150,
    }, /* TP_LD_PARAMS_NORMAL */
    {
        0.10f, 1.4f, 50, 200,
    }, /* TP_LD_PARAMS_PESS */
    {
        0.025f, 1.5f, 50, 150,
    }, /* TP_LD_PARAMS_PLL_1MS */
    {
        0.025f, 1.5f, 50, 150,
    }, /* TP_LD_PARAMS_PLL_2MS */
    {
        0.025f, 1.5f, 50, 150,
    }, /* TP_LD_PARAMS_PLL_5MS */
    {
        0.005f, .6f, 50, 200,
    } /* TP_LD_PARAMS_FLL_5MS */
};

/** Tracking loop parameters template
 * Filled out at the trackig loop parameters switch event.
 */
static const tp_loop_params_t loop_params_template = {
    /** Code tracking noise bandwidth in Hz */
    .code_bw = TP_LOOP_PARAM_PLACE_HOLDER,
    .code_zeta = 0.707f, /**< Code tracking loop damping ratio */
    .code_k = 1.,        /**< Code tracking loop gain coefficient */
    /** carrier frequency /  chip rate */
    .carr_to_code = TP_LOOP_PARAM_PLACE_HOLDER,
    /** Carrier tracking loop noise bandwidth in Hz */
    .carr_bw = TP_LOOP_PARAM_PLACE_HOLDER,
    .carr_zeta = 0.707f, /**< Carrier tracking loop damping ratio */
    .carr_k = 1,         /**< Carrier tracking loop gain coefficient */
    /** FLL noise bandwidth in Hz */
    .fll_bw = TP_LOOP_PARAM_PLACE_HOLDER};

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
/* clang-format off */
static const tp_profile_entry_t gnss_track_profiles[] = {
/*
  These are the short names of the numbers & parameters listed
  in the same order below.
  { { pll_bw,     fll_bw,        dll_bw,     controller,      tracking_mode_gps,
        tracking_mode_glo,                    cn0_est },              ld_params,
   time_ms,   cn0_low_thr, cn0_high_thr,        acc_thr,            cn0_dyn_thr,
      next,       cn0_low,     cn0_high,            dyn,                   lock,
     flags }
*/

  /* initial profiles */
  [IDX_NONAME_0] =
  { {   40,             3,           10,   TP_CTRL_PLL3,          TP_TM_INITIAL,
            TP_TM_INITIAL,    TRACK_CN0_EST_SECONDARY },   TP_LD_PARAMS_PLL_1MS,
        50,             0,            0,              0,                      0,
      IDX_NONE,  IDX_NONE,     IDX_NONE,       IDX_NONE,               IDX_NONE,
      TP_UNAIDED },

  [IDX_NONAME_1] =
  { {   40,             1,           10,   TP_CTRL_PLL3,          TP_TM_INITIAL,
            TP_TM_INITIAL,    TRACK_CN0_EST_SECONDARY },   TP_LD_PARAMS_PLL_1MS,
        50,             0,            0,              0,                      0,
      IDX_NONE,  IDX_NONE,    IDX_NONE,       IDX_NONE,                IDX_NONE,
      TP_WAIT_BSYNC | TP_WAIT_PLOCK | TP_UNAIDED },

  [IDX_NONAME_2] =
  { {   40,             0,           10,   TP_CTRL_PLL3,          TP_TM_INITIAL,
            TP_TM_INITIAL,    TRACK_CN0_EST_SECONDARY },   TP_LD_PARAMS_PLL_1MS,
        60,             0,            0,              0,                      0,
      IDX_NONE,  IDX_NONE,     IDX_NONE,       IDX_NONE,               IDX_NONE,
      TP_UNAIDED },

  [IDX_NONAME_3] =
  { {   35,             0,           10,   TP_CTRL_PLL3,          TP_TM_INITIAL,
            TP_TM_INITIAL,    TRACK_CN0_EST_SECONDARY },   TP_LD_PARAMS_PLL_1MS,
        60,             0,            0,              0,                      0,
      IDX_NONE,  IDX_NONE,     IDX_NONE,       IDX_NONE,               IDX_NONE,
      TP_UNAIDED },

  [IDX_DLL_RECOVERY2] =
  { {   30,             0,           10,   TP_CTRL_PLL3,          TP_TM_INITIAL,
            TP_TM_INITIAL,    TRACK_CN0_EST_SECONDARY },   TP_LD_PARAMS_PLL_1MS,
        60,            30,            0,              0,                      0,
      IDX_NONE,  IDX_SENS,     IDX_NONE,       IDX_NONE,               IDX_NONE,
      TP_LOW_CN0 },

  [IDX_NONAME_23] =
  { {   25,             0,           10,   TP_CTRL_PLL3,          TP_TM_INITIAL,
            TP_TM_INITIAL,      TRACK_CN0_EST_PRIMARY },   TP_LD_PARAMS_PLL_1MS,
        60,            30,            0,              0,                      0,
      IDX_NONE,  IDX_SENS,     IDX_NONE,       IDX_NONE,               IDX_NONE,
      TP_LOW_CN0 },

  [IDX_NONAME_24] =
  { {   20,             0,            8,   TP_CTRL_PLL3,          TP_TM_INITIAL,
            TP_TM_INITIAL,      TRACK_CN0_EST_PRIMARY },   TP_LD_PARAMS_PLL_1MS,
        60,            30,            0,              0,                      0,
      IDX_NONE,  IDX_SENS,     IDX_NONE,       IDX_NONE,               IDX_NONE,
      TP_LOW_CN0 },

  [IDX_DLL_RECOVERY] =
  { {   18,             0,            5,   TP_CTRL_PLL3,          TP_TM_INITIAL,
            TP_TM_INITIAL,      TRACK_CN0_EST_PRIMARY },   TP_LD_PARAMS_PLL_1MS,
       300,            30,            0,              0,                      0,
      IDX_NONE,  IDX_SENS,     IDX_NONE,       IDX_NONE,               IDX_NONE,
      TP_LOW_CN0 },

  [IDX_NONAME_26] =
  { {   18,             0,            5,   TP_CTRL_PLL3,          TP_TM_INITIAL,
            TP_TM_INITIAL,      TRACK_CN0_EST_PRIMARY },   TP_LD_PARAMS_PLL_1MS,
       300,            30,            0,              0,                      0,
      IDX_NONE,  IDX_SENS,     IDX_NONE,       IDX_NONE,               IDX_NONE,
      TP_LOW_CN0 },

  [IDX_NONAME_27] =
  { {   18,             0,            5,   TP_CTRL_PLL3,          TP_TM_INITIAL,
            TP_TM_INITIAL,      TRACK_CN0_EST_PRIMARY },   TP_LD_PARAMS_PLL_1MS,
       300,            30,            0,              0,                      0,
      IDX_NONE,  IDX_SENS,     IDX_NONE,       IDX_NONE,               IDX_NONE,
      TP_LOW_CN0 },

  [IDX_NONAME_28] =
  { {   18,             0,            5,   TP_CTRL_PLL3,          TP_TM_INITIAL,
            TP_TM_INITIAL,      TRACK_CN0_EST_PRIMARY },   TP_LD_PARAMS_PLL_1MS,
       300,            30,            0,              0,                      0,
      IDX_NONE,  IDX_SENS,     IDX_NONE,       IDX_NONE,               IDX_NONE,
      TP_LOW_CN0 },

  [IDX_NONAME_29] =
  { {   18,             0,            5,   TP_CTRL_PLL3,          TP_TM_INITIAL,
            TP_TM_INITIAL,      TRACK_CN0_EST_PRIMARY },   TP_LD_PARAMS_PLL_1MS,
       300,            30,            0,              0,                      0,
      IDX_NONE,  IDX_SENS,     IDX_NONE,       IDX_NONE,               IDX_NONE,
      TP_LOW_CN0 },

  [IDX_NONAME_30] =
  { {   18,             0,            4,   TP_CTRL_PLL3,          TP_TM_INITIAL,
            TP_TM_INITIAL,      TRACK_CN0_EST_PRIMARY },   TP_LD_PARAMS_PLL_1MS,
       300,            30,            0,              0,                      0,
      IDX_NONE,  IDX_SENS,     IDX_NONE,       IDX_NONE,               IDX_NONE,
      TP_LOW_CN0 },

  [IDX_NONAME_31] =
  { {   18,             0,            3,   TP_CTRL_PLL3,          TP_TM_INITIAL,
            TP_TM_INITIAL,      TRACK_CN0_EST_PRIMARY },   TP_LD_PARAMS_PLL_1MS,
       300,            30,            0,              0,                      0,
      IDX_NONE,  IDX_SENS,     IDX_NONE,       IDX_NONE,               IDX_NONE,
      TP_LOW_CN0 },

  [IDX_NONAME_32] =
  { {   18,             0,            2,   TP_CTRL_PLL3,          TP_TM_INITIAL,
            TP_TM_INITIAL,      TRACK_CN0_EST_PRIMARY },   TP_LD_PARAMS_PLL_1MS,
       300,            30,            0,              0,                      0,
      IDX_NONE,  IDX_SENS,     IDX_NONE,       IDX_NONE,               IDX_NONE,
      TP_LOW_CN0 },

  /* Very high range CN0 profile. Suitable for pedestrian use case.
   * Tolerates large jerk values. */
  [IDX_VERY_HIGH_CN0] =
  { {   20,              0,           1,   TP_CTRL_PLL3,              TP_TM_1MS,
                 TP_TM_1MS,     TRACK_CN0_EST_PRIMARY },   TP_LD_PARAMS_PLL_1MS,
       300,             43,           0,            1.5,                      0,
      IDX_VERY_HIGH_CN0, IDX_HIGH_CN0, IDX_NONE, IDX_HIGH_CN0_DYN,     IDX_NONE,
      TP_LOW_CN0 | TP_USE_NEXT },

  /* high range CN0 profile */
  [IDX_HIGH_CN0] =
  { {   20,             0,            1,   TP_CTRL_PLL3,              TP_TM_5MS,
                TP_TM_5MS,      TRACK_CN0_EST_PRIMARY },   TP_LD_PARAMS_PLL_5MS,
        50,            38,           46,             2.,                      0,
      IDX_HIGH_CN0, IDX_MID_CN0, IDX_VERY_HIGH_CN0, IDX_HIGH_CN0_DYN,  IDX_NONE,
      TP_LOW_CN0 | TP_HIGH_CN0 | TP_HIGH_DYN | TP_USE_NEXT },

  /* middle range CN0 profile */
  [IDX_MID_CN0] =
  { {   18,             0,            1,   TP_CTRL_PLL3,             TP_TM_10MS,
               TP_TM_10MS,      TRACK_CN0_EST_PRIMARY },   TP_LD_PARAMS_PLL_5MS,
        50,            33,           41,            1.5,                      0,
      IDX_MID_CN0, IDX_LOW_CN0_INI, IDX_HIGH_CN0, IDX_LOW_CN0_DYN,     IDX_NONE,
      TP_LOW_CN0 | TP_HIGH_CN0 | TP_HIGH_DYN | TP_USE_NEXT },

  /* low range CN0 profiles */
  [IDX_LOW_CN0_INI] =
  { {   15,             0,            1,   TP_CTRL_PLL3,             TP_TM_10MS,
               TP_TM_10MS,      TRACK_CN0_EST_PRIMARY },   TP_LD_PARAMS_PLL_5MS,
        50,            25,           36,            1.5,                      0,
      IDX_NONE,  IDX_SENS,  IDX_MID_CN0, IDX_LOW_CN0_DYN,              IDX_NONE,
      TP_LOW_CN0 | TP_HIGH_CN0 | TP_HIGH_DYN },

  [IDX_NONAME_11] =
  { {   14,             0,            1,   TP_CTRL_PLL3,             TP_TM_10MS,
               TP_TM_10MS,      TRACK_CN0_EST_PRIMARY },   TP_LD_PARAMS_PLL_5MS,
        50,            25,           36,            1.5,                      0,
      IDX_NONE,  IDX_SENS,  IDX_MID_CN0, IDX_LOW_CN0_DYN,              IDX_NONE,
      TP_LOW_CN0 | TP_HIGH_CN0 | TP_HIGH_DYN },

  [IDX_NONAME_12] =
  { {   12,             0,            1,   TP_CTRL_PLL3,             TP_TM_10MS,
               TP_TM_10MS,      TRACK_CN0_EST_PRIMARY },   TP_LD_PARAMS_PLL_5MS,
        50,            25,           36,            1.5,                      0,
      IDX_NONE,  IDX_SENS,  IDX_MID_CN0, IDX_LOW_CN0_DYN,              IDX_NONE,
      TP_LOW_CN0 | TP_HIGH_CN0 | TP_HIGH_DYN },

  [IDX_NONAME_13] =
  { {   10,             0,            1,   TP_CTRL_PLL3,             TP_TM_10MS,
               TP_TM_10MS,      TRACK_CN0_EST_PRIMARY },   TP_LD_PARAMS_PLL_5MS,
        50,            25,           36,            1.5,                      0,
      IDX_NONE,  IDX_SENS,  IDX_MID_CN0, IDX_LOW_CN0_DYN,              IDX_NONE,
      TP_LOW_CN0 | TP_HIGH_CN0 | TP_HIGH_DYN },

  [IDX_NONAME_14] =
  { {   10,             0,            1,   TP_CTRL_PLL3,             TP_TM_20MS,
               TP_TM_10MS,      TRACK_CN0_EST_PRIMARY },   TP_LD_PARAMS_PLL_5MS,
        50,            25,           36,            1.5,                      0,
      IDX_NONE,  IDX_SENS,  IDX_MID_CN0, IDX_LOW_CN0_DYN,              IDX_NONE,
      TP_LOW_CN0 | TP_HIGH_CN0 | TP_HIGH_DYN },

  [IDX_NONAME_15] =
  { {    8,             0,            1,   TP_CTRL_PLL3,             TP_TM_20MS,
               TP_TM_10MS,      TRACK_CN0_EST_PRIMARY },   TP_LD_PARAMS_PLL_5MS,
        50,            25,           36,            1.5,                      0,
      IDX_NONE,  IDX_SENS,  IDX_MID_CN0, IDX_LOW_CN0_DYN,              IDX_NONE,
      TP_LOW_CN0 | TP_HIGH_CN0 | TP_HIGH_DYN },

  [IDX_LOW_CN0_FIN] =
  { {    7,             0,           .5,   TP_CTRL_PLL3,             TP_TM_20MS,
               TP_TM_10MS,      TRACK_CN0_EST_PRIMARY },   TP_LD_PARAMS_PLL_5MS,
        50,            25,           36,            1.5,                      0,
      IDX_LOW_CN0_FIN, IDX_SENS, IDX_MID_CN0, IDX_LOW_CN0_DYN,         IDX_NONE,
      TP_LOW_CN0 | TP_HIGH_CN0 | TP_USE_NEXT | TP_HIGH_DYN },

  /* sensitivity profiles */
  [IDX_SENS] =
  { {    0,             1,            1,   TP_CTRL_FLL2,             TP_TM_20MS,
               TP_TM_10MS,    TRACK_CN0_EST_SECONDARY },   TP_LD_PARAMS_FLL_5MS,
      4000,             0,          32.,            1.5,                    32.,
      IDX_SENS,  IDX_NONE, IDX_TRAN_CN0,   IDX_TRAN_DYN,               IDX_NONE,
      TP_HIGH_CN0_WAIT_PLOCK_N0_DYN | TP_HIGH_DYN_WAIT_PLOCK_CN0 |
      TP_WAIT_CN0 | TP_USE_NEXT },

  /* sensitivity to low range CN0 transitional profiles  */
  [IDX_TRAN_CN0] =
  { {   20,             1,            1,   TP_CTRL_PLL3,             TP_TM_10MS,
               TP_TM_10MS,    TRACK_CN0_EST_SECONDARY },   TP_LD_PARAMS_PLL_5MS,
        50,           26.,            0,              0,                      0,
      IDX_NONE,  IDX_SENS,     IDX_NONE,       IDX_NONE,               IDX_NONE,
      TP_LOW_CN0 | TP_WAIT_PLOCK },

  [IDX_NONAME_19] =
  { {   20,             0,            1,   TP_CTRL_PLL3,             TP_TM_10MS,
               TP_TM_10MS,    TRACK_CN0_EST_SECONDARY },   TP_LD_PARAMS_PLL_5MS,
        50,           26.,            0,            1.5,                      0,
      IDX_NONE,  IDX_SENS,     IDX_NONE, IDX_LOW_CN0_DYN,              IDX_NONE,
      TP_LOW_CN0 | TP_HIGH_DYN },

  [IDX_NONAME_20] =
  { {   18,             0,            1,   TP_CTRL_PLL3,             TP_TM_10MS,
               TP_TM_10MS,    TRACK_CN0_EST_SECONDARY },   TP_LD_PARAMS_PLL_5MS,
        50,           26.,            0,            1.5,                      0,
      IDX_DLL_RECOVERY, IDX_SENS, IDX_NONE, IDX_LOW_CN0_DYN,           IDX_NONE,
      TP_LOW_CN0 | TP_HIGH_DYN | TP_USE_NEXT },

  /* sensitivity to dynamics transitional profiles */
  [IDX_TRAN_DYN] =
  { {   30,             3,            1,   TP_CTRL_PLL3,              TP_TM_5MS,
                TP_TM_5MS,    TRACK_CN0_EST_SECONDARY },   TP_LD_PARAMS_PLL_5MS,
        50,           30.,            0,              0,                      0,
    IDX_NONE,    IDX_SENS,     IDX_NONE,       IDX_NONE,               IDX_NONE,
      TP_LOW_CN0 | TP_WAIT_PLOCK },

  [IDX_NONAME_22] =
  { {   30,             1,            1,   TP_CTRL_PLL3,              TP_TM_5MS,
                TP_TM_5MS,    TRACK_CN0_EST_SECONDARY },   TP_LD_PARAMS_PLL_5MS,
        50,           30.,            0,              0,                      0,
    IDX_LOW_CN0_DYN, IDX_SENS, IDX_NONE,       IDX_NONE,               IDX_NONE,
      TP_LOW_CN0 | TP_USE_NEXT },

  /* dynamics profile for low CN0 */
  [IDX_LOW_CN0_DYN] =
  { {   30,             0,            1,   TP_CTRL_PLL3,              TP_TM_5MS,
                TP_TM_5MS,    TRACK_CN0_EST_SECONDARY },   TP_LD_PARAMS_PLL_5MS,
        50,           30.,          41.,            1.0,                      0,
   IDX_LOW_CN0_DYN, IDX_SENS, IDX_HIGH_CN0_DYN, IDX_DLL_RECOVERY2,     IDX_NONE,
      TP_LOW_CN0 | TP_HIGH_CN0 | TP_LOW_DYN | TP_USE_NEXT },

  /* dynamics profile for high CN0 */
  [IDX_HIGH_CN0_DYN] =
  { {   30,             0,            1,   TP_CTRL_PLL3,              TP_TM_1MS,
                TP_TM_1MS,      TRACK_CN0_EST_SECONDARY }, TP_LD_PARAMS_PLL_5MS,
        50,            30,            0,              1,                      0,
   IDX_HIGH_CN0_DYN, IDX_SENS, IDX_NONE, IDX_DLL_RECOVERY2,            IDX_NONE,
      TP_LOW_CN0 | TP_LOW_DYN | TP_USE_NEXT }
};
/* clang-format on */

/**
 * Helper method to get tracking profiles array.
 *
 * \param[in] mesid ME signal ID
 * \return Tracking profiles array pointer
 */
static const tp_profile_entry_t *mesid_to_profiles(
    const me_gnss_signal_t mesid) {
  const tp_profile_entry_t *profiles = NULL;

  /* GPS and SBAS constellations use similar signal encoding scheme and thus
     share the same tracking profiles.
     For GLONASS signals we limit the maximum integration time to 10 ms.
     Otherwise we use the same set of tracking profiles. */

  switch (mesid_to_constellation(mesid)) {
    case CONSTELLATION_GPS:
    case CONSTELLATION_SBAS:
    case CONSTELLATION_GLO:
      profiles = gnss_track_profiles;
      break;

    case CONSTELLATION_BDS2:
    case CONSTELLATION_GAL:
    case CONSTELLATION_QZS:
    case CONSTELLATION_INVALID:
    case CONSTELLATION_COUNT:
    default:
      assert(!"Invalid constellation");
      break;
  }

  return profiles;
}

/** Return track mode for the given code.
 * \param code The track mode is returned for this code.
 * \param profile The profile details having track modes for different codes.
 * \return The track mode.
 */
tp_tm_e track_mode_by_code(code_t code,
                           const struct tp_profile_entry *profile) {
  tp_tm_e track_mode = TP_TM_INITIAL;

  switch (code) {
    case CODE_GPS_L1CA:
    case CODE_GPS_L2CM:
    case CODE_GPS_L2CL:
      track_mode = profile->profile.gps_track_mode;
      break;
    case CODE_GLO_L1CA:
    case CODE_GLO_L2CA:
      track_mode = profile->profile.glo_track_mode;
      break;
    case CODE_SBAS_L1CA:
    case CODE_INVALID:
    case CODE_GPS_L1P:
    case CODE_GPS_L2P:
    case CODE_GPS_L2CX:
    case CODE_GPS_L5I:
    case CODE_GPS_L5Q:
    case CODE_GPS_L5X:
    case CODE_BDS2_B11:
    case CODE_BDS2_B2:
    case CODE_GAL_E1B:
    case CODE_GAL_E1C:
    case CODE_GAL_E1X:
    case CODE_GAL_E6B:
    case CODE_GAL_E6C:
    case CODE_GAL_E6X:
    case CODE_GAL_E7I:
    case CODE_GAL_E7Q:
    case CODE_GAL_E7X:
    case CODE_GAL_E8:
    case CODE_GAL_E5I:
    case CODE_GAL_E5Q:
    case CODE_GAL_E5X:
    case CODE_QZS_L1CA:
    case CODE_QZS_L2CM:
    case CODE_QZS_L2CL:
    case CODE_QZS_L2CX:
    case CODE_QZS_L5I:
    case CODE_QZS_L5Q:
    case CODE_QZS_L5X:
    case CODE_COUNT:
    default:
      assert(0);
      break;
  }

  return track_mode;
}

/**
 * Helper method to obtain tracking loop parameters.
 *
 * The method generates tracking loop parameters according to selected
 * configuration.
 *
 * \param[in]  mesid   ME signal identifier.
 * \param[in]  profile GNSS satellite profile.
 * \param[out] config  Container for computed configuration.
 *
 * \return None
 */
static void get_profile_params(const me_gnss_signal_t mesid,
                               const tp_profile_t *profile,
                               tp_config_t *config) {
  const tp_profile_entry_t *cur_profile =
      &profile->profiles[profile->cur_index];
  config->lock_detect_params = ld_params[cur_profile->ld_params];

  u16 flags = cur_profile->flags;
  double carr_to_code = 0.0;
  if (0 == (flags & TP_UNAIDED)) {
    carr_to_code = mesid_to_carr_to_code(mesid);
  }

  /* fill out the tracking loop parameters */
  config->loop_params = loop_params_template;
  config->loop_params.carr_to_code = carr_to_code;
  config->loop_params.code_bw = cur_profile->profile.dll_bw;
  config->loop_params.carr_bw = cur_profile->profile.pll_bw;
  config->loop_params.fll_bw = cur_profile->profile.fll_bw;
  config->loop_params.mode = track_mode_by_code(mesid.code, cur_profile);
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
  if ((TP_TM_5MS == mode || TP_TM_10MS == mode || TP_TM_20MS == mode) &&
      (TP_CTRL_PLL2 == ctrl || TP_CTRL_PLL3 == ctrl)) {
    config->use_alias_detection = true;
  } else {
    config->use_alias_detection = false;
  }

  tp_profile_get_cn0_params(profile, &config->cn0_params);
}

/**
 * Helper method to incorporate tracking loop information into statistics.
 *
 * \param[in,out] tracker_channel Tracker channel data
 * \param[in,out] profile     Satellite profile.
 * \param[in]     data        Data from tracking loop.
 *
 * \return None
 */
static void update_stats(tracker_channel_t *tracker_channel,
                         tp_profile_t *profile,
                         const tp_report_t *data) {
  float cn0;
  u32 cur_time_ms = tracker_channel->update_count;
  const me_gnss_signal_t mesid = tracker_channel->mesid;

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

  cn0 = data->cn0;

  profile->filt_cn0 = cn0;

  float carr_freq = mesid_to_carr_freq(mesid);
  float acceleration_g = data->acceleration *
                         (float)(GPS_C / STD_GRAVITY_ACCELERATION) / carr_freq;

  profile->filt_accel = acceleration_g;
}

/**
 * Internal helper for naming loop controller types.
 *
 * \param[in] v Loop controller type.
 *
 * \return Loop controller type literal.
 */
static const char *get_ctrl_str(tp_ctrl_e v) {
  const char *str = "?";
  switch (v) {
    case TP_CTRL_PLL2:
      str = "PLL2";
      break;
    case TP_CTRL_PLL3:
      str = "PLL3";
      break;
    case TP_CTRL_FLL1:
      str = "FLL1";
      break;
    case TP_CTRL_FLL2:
      str = "FLL2";
      break;
    default:
      assert(!"Unknown loop controller type");
  }
  return str;
}

/**
 * Used to debug the profile switching logic.
 *
 * The function generate log output only when debug level logging is enabled.
 *
 * \param[in] mesid  ME signal identifier.
 * \param[in] state  Tracking loop state
 * \param[in] reason Profile switching reason in a textual form
 *
 * \return None
 */
static void log_switch(const me_gnss_signal_t mesid,
                       const tp_profile_t *state,
                       const char *reason) {
  const tp_profile_entry_t *cur_profile = &state->profiles[state->cur_index];
  const tp_profile_entry_t *next_profile = &state->profiles[state->next_index];
  tp_tm_e cur_track_mode = track_mode_by_code(mesid.code, cur_profile);
  tp_tm_e next_track_mode = track_mode_by_code(mesid.code, next_profile);

  log_debug_mesid(mesid,
                  "%s: plock=%" PRId16 " bs=%" PRId16
                  " cn0=%.1f acc=%.1fg "
                  "(mode,pll,fll,ctrl): (%s,%.1f,%.1f,%s)->(%s,%.1f,%.1f,%s)",
                  reason,
                  state->plock_delay_ms,
                  state->bs_delay_ms,
                  state->filt_cn0,
                  state->filt_accel,
                  /* old state */
                  tp_get_mode_str(cur_track_mode),
                  cur_profile->profile.pll_bw,
                  cur_profile->profile.fll_bw,
                  get_ctrl_str(cur_profile->profile.controller_type),
                  /* new state */
                  tp_get_mode_str(next_track_mode),
                  next_profile->profile.pll_bw,
                  next_profile->profile.fll_bw,
                  get_ctrl_str(next_profile->profile.controller_type));
}

/**
 * Helper method to dump tracking statistics into log.
 *
 * The method logs average and RMS values for analyzes.
 *
 * \param[in]     mesid   ME signal identifier.
 * \param[in,out] profile GNSS satellite profile.
 *
 * \return None
 */
static void print_stats(const me_gnss_signal_t mesid, tp_profile_t *profile) {
  if (profile->print_time > 0) {
    return;
  }

  profile->print_time = DEBUG_PRINT_TIME_INTERVAL_MS;

  const char *cn0_est_str = track_cn0_str(profile->cn0_est);

  const tp_profile_entry_t *cur_profile =
      &profile->profiles[profile->cur_index];
  tp_tm_e tracking_mode = track_mode_by_code(mesid.code, cur_profile);
  int dll_ms = tp_get_dll_ms(tracking_mode);

  const char *m1 = tp_get_mode_str(tracking_mode);
  const char *c1 = get_ctrl_str(cur_profile->profile.controller_type);

  /*
   * PRINT: integration time, loop mode, controller mode,
   *        C/N0 estimator, C/N0 value, SNR value (dBm),
   *        PR rate, PR rate change,
   *        PLL lock detector ratio, FLL/DLL error
   */

  log_debug_mesid(mesid,
                  "AVG: %dms %s %s CN0_%s=%.2f (%.2f) A=%.3f",
                  dll_ms,
                  m1,
                  c1,
                  cn0_est_str,
                  profile->filt_cn0,
                  TRACK_CN0_TO_SNR(profile->filt_cn0),
                  profile->filt_accel);
}

/**
 * Detects the acceleration on/off condition.
 *
 * \params[in,out] state tracking loop state
 */
static void update_acceleration_status(tp_profile_t *state) {
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
 * \param[in]     mesid ME signal identifier.
 * \param[in,out] state Tracking loop state
 */
static void check_for_cn0_estimator_change(const me_gnss_signal_t mesid,
                                           tp_profile_t *state) {
  float cn0 = 0.f;
  const tp_profile_entry_t *cur_profile;

  if (TRACK_CN0_EST_PRIMARY == TRACK_CN0_EST_SECONDARY) {
    return;
  }

  cn0 = state->filt_cn0;
  cur_profile = &state->profiles[state->cur_index];
  tp_tm_e cur_track_mode = track_mode_by_code(mesid.code, cur_profile);
  u8 cn0_ms = tp_get_cn0_ms(cur_track_mode);

  if (TRACK_CN0_EST_PRIMARY == state->cn0_est) {
    if (cn0 < track_cn0_get_pri2sec_threshold(cn0_ms) ||
        TRACK_CN0_EST_SECONDARY == cur_profile->profile.cn0_est) {
      state->cn0_est = TRACK_CN0_EST_SECONDARY;
      log_debug_mesid(mesid, "Changed C/N0 estimator to secondary");
    }
  } else if (TRACK_CN0_EST_SECONDARY == state->cn0_est) {
    if (cn0 > track_cn0_get_sec2pri_threshold(cn0_ms) &&
        TRACK_CN0_EST_PRIMARY == cur_profile->profile.cn0_est) {
      state->cn0_est = TRACK_CN0_EST_PRIMARY;
      log_debug_mesid(mesid, "Changed C/N0 estimator to primary");
    }
  } else {
    assert(!"Unsupported CN0 estimator identifier");
  }
}

/** Integration time is not explicitly available in gnss_track_profiles.
 *  This API infers the integration time from the profiles' array index.
 *  \param mesid ME signal ID
 *  \param state Profiles array.
 *  \param index Profiles array index.
 *  \return Integration time of the profile at the given index [ms]
 */
static u8 profile_integration_time(const me_gnss_signal_t mesid,
                                   const tp_profile_t *state,
                                   const profile_indices_t index) {
  static u8 int_times[] = {[TP_TM_INITIAL] = 1,
                           [TP_TM_1MS] = 1,
                           [TP_TM_2MS] = 2,
                           [TP_TM_5MS] = 5,
                           [TP_TM_10MS] = 10,
                           [TP_TM_20MS] = 20};

  tp_tm_e track_mode;
  if (IS_GPS(mesid)) {
    track_mode = state->profiles[index].profile.gps_track_mode;
  } else if (IS_GLO(mesid)) {
    track_mode = state->profiles[index].profile.glo_track_mode;
  } else {
    assert(!"Unsupported constellation");
  }
  assert(track_mode < ARRAY_SIZE(int_times));
  u8 int_time = int_times[track_mode];
  assert(int_time != 0);
  assert(int_time <= 20);

  return int_time;
}

/**
 * Internal method for profile switch request.
 *
 * Sets the requested profile as the current one.
 *
 * \param[in]     mesid  ME signal identifier.
 * \param[in,out] state  Tracking loop state
 * \param[in]     index  Index of profile to activate
 * \param[in]     reason Textual reason of profile switch
 *
 * \retval true Profile switch requested
 * \retval false No profile switch requested
 */
static bool profile_switch_requested(const me_gnss_signal_t mesid,
                                     tp_profile_t *state,
                                     profile_indices_t index,
                                     const char *reason) {
  if (index == state->cur_index) {
    return false;
  }

  assert(index != IDX_NONE);
  assert((size_t)index < ARRAY_SIZE(gnss_track_profiles));

  u8 int_time = profile_integration_time(mesid, state, index);
  float pll_bw = state->profiles[index].profile.pll_bw;
  if ((pll_bw > 0) && (int_time > max_pll_integration_time_ms)) {
    return false; /* setting prevents us doing longer integration time */
  }

  state->dll_init = false;
  const tp_profile_entry_t *cur = &state->profiles[state->cur_index];
  const tp_profile_entry_t *next = &state->profiles[index];
  if ((0 != (cur->flags & TP_UNAIDED)) && (0 == (next->flags & TP_UNAIDED))) {
    /* Unaided DLL velocity causes instability when switching to aided DLL */
    state->dll_init = true;
  }

  state->lock_time_ms = state->profiles[index].lock_time_ms;
  state->profile_update = true;
  state->next_index = index;

  log_switch(mesid, state, reason);

  return true;
}

/**
 * Internal method for evaluating profile change conditions.
 *
 * This method analyzes collected statistics and selects appropriate tracking
 * parameter changes.
 *
 * \param[in]     mesid ME signal identifier.
 * \param[in,out] state Tracking loop state
 *
 * \return None
 */
static void check_for_profile_change(const me_gnss_signal_t mesid,
                                     tp_profile_t *state) {
  const tp_profile_entry_t *cur_profile;
  u16 flags;
  bool acceleration_detected;

  cur_profile = &state->profiles[state->cur_index];
  flags = cur_profile->flags;

  state->profile_update = false;

  check_for_cn0_estimator_change(mesid, state);

  update_acceleration_status(state);
  acceleration_detected = (0 != state->acceleration_ends_after_ms);

  if ((0 != (flags & TP_LOW_CN0)) &&
      (state->filt_cn0 < cur_profile->cn0_low_threshold) &&
      profile_switch_requested(
          mesid, state, cur_profile->next_cn0_low, "low cn0")) {
    return;
  }

  if ((0 != (flags & TP_NO_PLOCK)) && !state->plock &&
      profile_switch_requested(
          mesid, state, cur_profile->next_lock, "no plock")) {
    return;
  }

  if ((0 != (flags & TP_HIGH_DYN)) && acceleration_detected &&
      profile_switch_requested(
          mesid, state, cur_profile->next_dyn, "high dyn")) {
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

  if ((0 != (flags & TP_HIGH_DYN_WAIT_PLOCK_CN0)) && acceleration_detected &&
      state->plock && (state->filt_cn0 > cur_profile->cn0_dyn_threshold) &&
      profile_switch_requested(
          mesid, state, cur_profile->next_dyn, "high dyn")) {
    return;
  }

  if ((0 != (flags & TP_LOW_DYN)) && !acceleration_detected &&
      profile_switch_requested(
          mesid, state, cur_profile->next_dyn, "low dyn")) {
    return;
  }

  if ((0 != (flags & TP_HIGH_CN0)) &&
      (state->filt_cn0 > cur_profile->cn0_high_threshold) &&
      profile_switch_requested(
          mesid, state, cur_profile->next_cn0_high, "high cno")) {
    return;
  }

  if ((0 != (flags & TP_HIGH_CN0_WAIT_PLOCK_N0_DYN)) && state->plock &&
      !acceleration_detected &&
      (state->filt_cn0 > cur_profile->cn0_high_threshold) &&
      profile_switch_requested(
          mesid, state, cur_profile->next_cn0_high, "high cno")) {
    return;
  }

  if (0 != (flags & TP_USE_NEXT)) {
    assert(cur_profile->next != IDX_NONE);
    profile_switch_requested(mesid, state, cur_profile->next, "next");
  } else {
    profile_switch_requested(mesid, state, state->cur_index + 1, "next");
  }
}

/**
 * Helper method for computing C/N0 offset.
 *
 * The method computes C/N0 offset for tracking loop in accordance to
 * integration period and tracking parameters.
 *
 * \param[in] mesid   ME signal ID
 * \param[in] profile GNSS satellite profile
 *
 * \return Computed C/N0 offset in dB/Hz.
 */
static float compute_cn0_offset(const me_gnss_signal_t mesid,
                                const tp_profile_t *profile) {
  const struct tp_profile_entry *cur_profile;
  tp_tm_e mode;

  cur_profile = &profile->profiles[profile->cur_index];
  mode = track_mode_by_code(mesid.code, cur_profile);

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
tp_result_e tp_init(void) { return TP_RESULT_SUCCESS; }

/**
 * Registers GNSS satellite in facility.
 *
 * The method registers GNSS signal and returns initial tracking parameters.
 *
 * \param[in]  mesid   ME signal identifier.
 * \param[out] profile Profile data to initialize.
 * \param[in]  data    Initial parameters.
 * \param[out] config  Container for initial tracking parameters.
 *
 * \retval TP_RESULT_SUCCESS The satellite has been registered and initial
 *                           profile is returned.
 * \retval TP_RESULT_ERROR   On error.
 *
 * \sa tp_tracking_stop()
 */
tp_result_e tp_profile_init(const me_gnss_signal_t mesid,
                            tp_profile_t *profile,
                            const tp_report_t *data,
                            tp_config_t *config) {
  tp_result_e res = TP_RESULT_ERROR;

  if (NULL != config && NULL != profile) {
    memset(profile, 0, sizeof(*profile));

    profile->filt_cn0 = data->cn0;
    profile->filt_accel = 0;

    profile->cur_index = 0;
    profile->profiles = mesid_to_profiles(mesid);
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

    get_profile_params(mesid, profile, config);

    res = TP_RESULT_SUCCESS;
  } else {
    assert(!"Invalid argument");
  }
  return res;
}

/**
 * Retrieves new tracking profile if available.
 *
 * \param[in]     mesid    ME signal identifier.
 * \param[in,out] profile  Tracking profile data to read and update.
 * \param[out]    config   Container for new tracking parameters.
 * \param[in]     commit   Commit the mode change happened.
 *
 * \retval TP_RESULT_SUCCESS New tracking profile has been retrieved. The
 *                           tracking loop shall reconfigure it's components
 *                           and, possibly, change the operation mode.
 * \retval TP_RESULT_NO_DATA New tracking profile is not available. No further
 *                           actions are needed.
 * \retval TP_RESULT_ERROR   On error.
 */
tp_result_e tp_profile_get_config(const me_gnss_signal_t mesid,
                                  tp_profile_t *profile,
                                  tp_config_t *config,
                                  bool commit) {
  tp_result_e res = TP_RESULT_ERROR;
  if (NULL != config && NULL != profile) {
    if (profile->profile_update) {
      /* Do transition of current profile */
      if (commit) {
        profile->profile_update = 0;

        profile->cur_index = profile->next_index;
        profile->cn0_offset = compute_cn0_offset(mesid, profile);
      }

      /* Return data */
      get_profile_params(mesid, profile, config);

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
 * \param[in]  profile    Tracking profile data to check
 * \param[out] cn0_params Container for C/N0 limits.
 *
 * \retval TP_RESULT_SUCCESS C/N0 thresholds have been retrieved.
 * \retval TP_RESULT_ERROR   On error.
 */
tp_result_e tp_profile_get_cn0_params(const tp_profile_t *profile,
                                      tp_cn0_params_t *cn0_params) {
  if ((NULL == cn0_params) || (NULL == profile)) {
    return TP_RESULT_ERROR;
  }

  *cn0_params = cn0_params_default;

  /* Correction: higher integration time lowers thresholds linearly. For
   * example, 20ms integration has threshold by 13 dB lower, than for 1ms
   * integration. */
  cn0_params->track_cn0_drop_thres_dbhz -= profile->cn0_offset;

  cn0_params->est = (track_cn0_est_e)profile->cn0_est;

  float threshold_dbhz = TP_HARD_CN0_DROP_THRESHOLD_DBHZ;

  if (cn0_params->track_cn0_drop_thres_dbhz < threshold_dbhz) {
    cn0_params->track_cn0_drop_thres_dbhz = threshold_dbhz;
  }
  if (cn0_params->track_cn0_use_thres_dbhz < threshold_dbhz) {
    cn0_params->track_cn0_use_thres_dbhz = threshold_dbhz;
  }
  if (cn0_params->track_cn0_ambiguity_thres_dbhz < threshold_dbhz) {
    cn0_params->track_cn0_ambiguity_thres_dbhz = threshold_dbhz;
  }

  return TP_RESULT_SUCCESS;
}

/**
 * Method to check if there is a pending profile change.
 *
 * \param[in] mesid   ME signal identifier.
 * \param[in] profile Tracking profile data to check
 *
 * \retval true  New profile is available.
 * \retval false No profile change is required.
 */
bool tp_profile_has_new_profile(const me_gnss_signal_t mesid,
                                tp_profile_t *profile) {
  bool res = false;
  if (NULL != profile) {
    check_for_profile_change(mesid, profile);
    res = profile->profile_update != 0;
  }
  return res;
}

/**
 * Helper to obtain loop parameters for the next integration interval.
 *
 * \param[in] mesid   ME signal identifier.
 * \param[in] profile Tracking profile data to check
 *
 * \return Loop parameters for the next integration interval
 */
u8 tp_profile_get_next_loop_params_ms(const me_gnss_signal_t mesid,
                                      const tp_profile_t *profile) {
  assert(NULL != profile);
  const struct tp_profile_entry *next_profile;
  next_profile = &profile->profiles[profile->next_index];
  tp_tm_e track_mode = track_mode_by_code(mesid.code, next_profile);

  return tp_get_dll_ms(track_mode);
}

/**
 * Updates track profile data with supplied information.
 *
 * The method takes tracking loop data and merges it with previously collected
 * information from other tracking loops.
 *
 * \param[in]     mesid       ME signal identifier.
 * \param[in,out] profile     Tracking profile data to update
 * \param[in]     data        Tracking loop report.
 */
void tp_profile_report_data(tracker_channel_t *tracker_channel,
                            tp_profile_t *profile,
                            const tp_report_t *data) {
  assert(tracker_channel);
  assert(profile);
  assert(data);

  update_stats(tracker_channel, profile, data);
  print_stats(tracker_channel->mesid, profile);
}
