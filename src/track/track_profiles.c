/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <libswiftnav/constants.h>

#include <board.h>
#include <chconf_board.h>
#include <nap/nap_common.h>
#include <nap/nap_hw.h>
#include <platform_cn0.h>
#include <platform_track.h>

#include <assert.h>
#include <math.h>
#include <string.h>

#include "gnss_capabilities/gnss_capabilities.h"
#include "lock_detector/lock_detector.h"
#include "signal_db/signal_db.h"
#include "track/track_cfg.h"
#include "track/track_common.h"
#include "track/track_flags.h"

/* 35ms is a result of experimenting.
   The threshold is needed to avoid spontaneous transitions
   to sensitivity profile, when signal is reasonably strong */
#define TP_WEAK_SIGNAL_THRESHOLD_MS 35

/** Unknown delay indicator */
#define TP_DELAY_UNKNOWN -1

/** Indices of specific entries in gnss_track_profiles[] table below */
typedef enum {
  /** Placeholder for an index. Indicates an unused index field. */
  IDX_NONE = -1,
  IDX_INIT_0,
  IDX_INIT_1,
  IDX_INIT_2,
  IDX_2MS,
  IDX_5MS,
  IDX_10MS,
  IDX_20MS,
  IDX_SENS
} profile_indices_t;

typedef enum {
  TP_LOW_CN0 = (1 << 0),    /**< Watch low CN0 value */
  TP_HIGH_CN0 = (1 << 1),   /**< Watch high CN0 value */
  TP_WAIT_BSYNC = (1 << 5), /**< Wait for bit sync */
  TP_WAIT_PLOCK = (1 << 6), /**< Wait for phase lock */
  TP_WAIT_FLOCK = (1 << 7), /**< Wait for frequency lock */
  TP_USE_NEXT = (1 << 8),   /**< Use next index to choose next profile */

  /** Do not use carrier aiding */
  TP_UNAIDED = (1 << 11)
} tp_profile_flags_t;

/**
 * Lock detector parameter set enumeration
 */
typedef enum {
  TP_LD_PARAMS_PHASE_INI,
  TP_LD_PARAMS_PHASE_1MS,
  TP_LD_PARAMS_PHASE_2MS,
  TP_LD_PARAMS_PHASE_5MS,
  TP_LD_PARAMS_PHASE_10MS,
  TP_LD_PARAMS_PHASE_20MS,
  TP_LD_PARAMS_FREQ_INI,
  TP_LD_PARAMS_FREQ_1MS,
  TP_LD_PARAMS_FREQ_2MS,
  TP_LD_PARAMS_FREQ_5MS,
  TP_LD_PARAMS_FREQ_10MS,
  TP_LD_PARAMS_FREQ_20MS,
} tp_ld_e;

/** Time interval in ms for printing channel statistics (when DEBUG is
 * enabled)*/
#define DEBUG_PRINT_TIME_INTERVAL_MS (20000)
#define BW_DYN -1

/** Describes single tracking profile */
typedef struct tp_profile_entry {
  struct {
    float pll_bw;              /**< PLL bandwidth [Hz] */
    float fll_bw;              /**< FLL bandwidth [Hz]  */
    float dll_bw;              /**< DLL bandwidth [Hz] */
    tp_ctrl_e controller_type; /**< Controller type */
    tp_tm_e tm_20ms;           /**< typical GPS and QZSS Tracking mode */
    tp_tm_e tm_10ms;           /**< typical GLO Tracking mode */
    tp_tm_e tm_2ms;            /**< typical SBAS Tracking mode */
    tp_tm_e tm_nh20ms;         /**< typical BDS and GPS L5 Tracking mode */
    tp_tm_e tm_sc4;            /**< E7 tracking mode */
  } profile;

  tp_ld_e ld_phase_params; /**< Phase lock detector parameter set */
  tp_ld_e ld_freq_params;  /**< Frequency lock detector parameter set */

  u16 lock_time_ms;         /**< Profile stabilization time [ms] */
  float cn0_low_threshold;  /**< Low CN0 threshold [dB-Hz] */
  float cn0_high_threshold; /**< High CN0 threshold [dB-Hz] */

  /** Next profile to activate once lock_time_ms is over */
  profile_indices_t next;

  /** Next profile to activate if TP_LOW_CN0 is set and CN0 is
      lower than this value */
  profile_indices_t next_cn0_low;

  /** Next profile to activate if TP_HIGH_CN0 is set and CN0 is
      higher than this value */
  profile_indices_t next_cn0_high;

  u16 flags; /**< Bit combination of tp_profile_flags_t */
} tp_profile_entry_t;

/**
 * C/N0 profile
 */
static const tp_cn0_params_t cn0_params_default = {
    .track_cn0_drop_thres_dbhz = TP_DEFAULT_CN0_DROP_THRESHOLD_DBHZ,
    .track_cn0_use_thres_dbhz = TP_DEFAULT_CN0_USE_THRESHOLD_DBHZ,
    .track_cn0_ambiguity_thres_dbhz = TP_DEFAULT_CN0_AMBIGUITY_THRESHOLD_DBHZ};

#define UNUSED 0.

/**
 * Lock detector profiles
 */
/* clang-format off */
static const tp_lock_detect_params_t ld_params_gps[] = {
                                  /* k1,     k2, lp */
    [TP_LD_PARAMS_PHASE_INI]  = { 0.09f,    1.f, 50 },
    [TP_LD_PARAMS_PHASE_1MS]  = { 0.29f,   0.7f, 50 },
    [TP_LD_PARAMS_PHASE_2MS]  = { 0.28f,   0.8f, 50 },
    [TP_LD_PARAMS_PHASE_5MS]  = { 0.26f,   1.0f, 50 },
    [TP_LD_PARAMS_PHASE_10MS] = { 0.22f,   1.4f, 50 },
    [TP_LD_PARAMS_PHASE_20MS] = { 0.01f,   1.4f, 50 },
    [TP_LD_PARAMS_FREQ_INI]   = { 0.07f, UNUSED, 50 },
    [TP_LD_PARAMS_FREQ_1MS]   = { 0.07f, UNUSED, 50 },
    [TP_LD_PARAMS_FREQ_2MS]   = { 0.07f, UNUSED, 40 },
    [TP_LD_PARAMS_FREQ_5MS]   = { 0.08f, UNUSED, 20 },
    [TP_LD_PARAMS_FREQ_10MS]  = {  0.1f, UNUSED, 15 },
    [TP_LD_PARAMS_FREQ_20MS]  = {  0.1f, UNUSED, 10 },
};
static const tp_lock_detect_params_t ld_params_glo[] = {
                                  /* k1,     k2, lp */
    [TP_LD_PARAMS_PHASE_INI]  = { 0.09f,    1.f, 50 },
    [TP_LD_PARAMS_PHASE_1MS]  = { 0.25f,   1.0f, 50 },
    [TP_LD_PARAMS_PHASE_2MS]  = { 0.22f,   1.0f, 50 },
    [TP_LD_PARAMS_PHASE_5MS]  = { 0.20f,   1.2f, 50 },
    [TP_LD_PARAMS_PHASE_10MS] = { 0.20f,   1.4f, 50 },
    [TP_LD_PARAMS_PHASE_20MS] = { 0.01f,   1.4f, 50 },
    [TP_LD_PARAMS_FREQ_INI]   = { 0.07f, UNUSED, 50 },
    [TP_LD_PARAMS_FREQ_1MS]   = { 0.07f, UNUSED, 50 },
    [TP_LD_PARAMS_FREQ_2MS]   = { 0.07f, UNUSED, 40 },
    [TP_LD_PARAMS_FREQ_5MS]   = { 0.08f, UNUSED, 20 },
    [TP_LD_PARAMS_FREQ_10MS]  = {  0.1f, UNUSED, 15 },
    [TP_LD_PARAMS_FREQ_20MS]  = {  0.1f, UNUSED, 10 },
};
static const tp_lock_detect_params_t ld_params_sbas[] = {
                                  /* k1,     k2, lp */
    [TP_LD_PARAMS_PHASE_INI]  = { 0.09f,    1.f, 50 },
    [TP_LD_PARAMS_PHASE_1MS]  = { 0.09f,    .5f, 50 },
    [TP_LD_PARAMS_PHASE_2MS]  = { 0.08f,    .5f, 50 },
    [TP_LD_PARAMS_PHASE_5MS]  = { 0.06f,   1.0f, 50 },
    [TP_LD_PARAMS_PHASE_10MS] = { 0.02f,   1.4f, 50 },
    [TP_LD_PARAMS_PHASE_20MS] = { 0.01f,   1.4f, 50 },
    [TP_LD_PARAMS_FREQ_INI]   = { 0.07f, UNUSED, 50 },
    [TP_LD_PARAMS_FREQ_1MS]   = { 0.07f, UNUSED, 50 },
    [TP_LD_PARAMS_FREQ_2MS]   = { 0.07f, UNUSED, 40 },
    [TP_LD_PARAMS_FREQ_5MS]   = { 0.08f, UNUSED, 20 },
    [TP_LD_PARAMS_FREQ_10MS]  = {  0.1f, UNUSED, 15 },
    [TP_LD_PARAMS_FREQ_20MS]  = {  0.1f, UNUSED, 10 },
};
static const tp_lock_detect_params_t ld_params_bds2[] = {
                                  /* k1,     k2, lp */
    [TP_LD_PARAMS_PHASE_INI]  = { 0.09f,    1.f, 50 },
    [TP_LD_PARAMS_PHASE_1MS]  = { 0.09f,    .5f, 50 },
    [TP_LD_PARAMS_PHASE_2MS]  = { 0.08f,    .5f, 50 },
    [TP_LD_PARAMS_PHASE_5MS]  = { 0.06f,   1.0f, 50 },
    [TP_LD_PARAMS_PHASE_10MS] = { 0.02f,   1.4f, 50 },
    [TP_LD_PARAMS_PHASE_20MS] = { 0.01f,   1.4f, 50 },
    [TP_LD_PARAMS_FREQ_INI]   = { 0.07f, UNUSED, 50 },
    [TP_LD_PARAMS_FREQ_1MS]   = { 0.07f, UNUSED, 50 },
    [TP_LD_PARAMS_FREQ_2MS]   = { 0.07f, UNUSED, 40 },
    [TP_LD_PARAMS_FREQ_5MS]   = { 0.08f, UNUSED, 20 },
    [TP_LD_PARAMS_FREQ_10MS]  = {  0.1f, UNUSED, 15 },
    [TP_LD_PARAMS_FREQ_20MS]  = {  0.1f, UNUSED, 10 },
};
static const tp_lock_detect_params_t ld_params_qzss[] = {
                                  /* k1,     k2, lp */
    [TP_LD_PARAMS_PHASE_INI]  = { 0.09f,    1.f, 50 },
    [TP_LD_PARAMS_PHASE_1MS]  = { 0.09f,    .5f, 50 },
    [TP_LD_PARAMS_PHASE_2MS]  = { 0.08f,    .5f, 50 },
    [TP_LD_PARAMS_PHASE_5MS]  = { 0.06f,   1.0f, 50 },
    [TP_LD_PARAMS_PHASE_10MS] = { 0.02f,   1.4f, 50 },
    [TP_LD_PARAMS_PHASE_20MS] = { 0.01f,   1.4f, 50 },
    [TP_LD_PARAMS_FREQ_INI]   = { 0.07f, UNUSED, 50 },
    [TP_LD_PARAMS_FREQ_1MS]   = { 0.07f, UNUSED, 50 },
    [TP_LD_PARAMS_FREQ_2MS]   = { 0.07f, UNUSED, 40 },
    [TP_LD_PARAMS_FREQ_5MS]   = { 0.08f, UNUSED, 20 },
    [TP_LD_PARAMS_FREQ_10MS]  = {  0.1f, UNUSED, 15 },
    [TP_LD_PARAMS_FREQ_20MS]  = {  0.1f, UNUSED, 10 },
};
static const tp_lock_detect_params_t ld_params_gal[] = {
                                  /* k1,     k2, lp */
    [TP_LD_PARAMS_PHASE_INI]  = { 0.09f,    1.f, 50 },
    [TP_LD_PARAMS_PHASE_1MS]  = { 0.09f,    .5f, 50 },
    [TP_LD_PARAMS_PHASE_2MS]  = { 0.08f,    .5f, 50 },
    [TP_LD_PARAMS_PHASE_5MS]  = { 0.06f,   1.0f, 50 },
    [TP_LD_PARAMS_PHASE_10MS] = { 0.02f,   1.4f, 50 },
    [TP_LD_PARAMS_PHASE_20MS] = { 0.01f,   1.4f, 50 },
    [TP_LD_PARAMS_FREQ_INI]   = { 0.07f, UNUSED, 50 },
    [TP_LD_PARAMS_FREQ_1MS]   = { 0.07f, UNUSED, 50 },
    [TP_LD_PARAMS_FREQ_2MS]   = { 0.07f, UNUSED, 40 },
    [TP_LD_PARAMS_FREQ_5MS]   = { 0.08f, UNUSED, 20 },
    [TP_LD_PARAMS_FREQ_10MS]  = {  0.1f, UNUSED, 15 },
    [TP_LD_PARAMS_FREQ_20MS]  = {  0.1f, UNUSED, 10 }
};
/* clang-format on */

/** Tracking loop parameter placeholder */
#define TP_LOOP_PARAM_PLACE_HOLDER 0.f

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
 * One entry of the table is one distinct profile.
 *
 * Essentially, the table describes a finite state machine (FSM).
 * Each tracking channel has its own instance of the FSM.
 * Therefore, all tracking channels are independent and may have
 * different profiles active at any moment of time.
 * The actual profile switching is done in #profile_switch_requested()
 * function.
 *
 * The transition within the table happens as a result of
 * continous evaluation of two parameters:
 *
 * -# time
 * -# CN0 level
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
  { { pll_bw,      fll_bw,       dll_bw,     controller,
      tracking_mode_gps, tracking_mode_glo, tracking_mode_sbas, tracking_mode_bds2 },
      ld_phase_params,   ld_freq_params,
    time_ms,   cn0_low_thr,   cn0_high_thr,
       next,       cn0_low,       cn0_high,
     flags }
*/

  [IDX_INIT_0] =
{ {     10,           7,           20,   TP_CTRL_PLL3,
        TP_TM_INITIAL,  TP_TM_INITIAL,  TP_TM_INITIAL,  TP_TM_INITIAL,  TP_TM_INITIAL},
        TP_LD_PARAMS_PHASE_INI, TP_LD_PARAMS_FREQ_INI,
       100,             0,            0,
      IDX_NONE,  IDX_NONE,     IDX_NONE,
      TP_UNAIDED | TP_WAIT_FLOCK},

  [IDX_INIT_1] =
  { { BW_DYN,      BW_DYN,           20,   TP_CTRL_PLL3,
        TP_TM_INITIAL,  TP_TM_INITIAL,  TP_TM_INITIAL,  TP_TM_INITIAL,  TP_TM_INITIAL },
        TP_LD_PARAMS_PHASE_INI, TP_LD_PARAMS_FREQ_INI,
       100,             0,            0,
      IDX_NONE,  IDX_NONE,     IDX_NONE,
      TP_WAIT_BSYNC | TP_WAIT_PLOCK | TP_UNAIDED },

  [IDX_INIT_2] =
  { { BW_DYN,      BW_DYN,            5,   TP_CTRL_PLL3,
      TP_TM_1MS_20MS,  TP_TM_1MS_10MS,  TP_TM_1MS_2MS,  TP_TM_1MS_NH20MS,  TP_TM_1MS_SC4 },
      TP_LD_PARAMS_PHASE_INI, TP_LD_PARAMS_FREQ_INI,
      100,             0,            0,
      IDX_NONE, IDX_NONE,     IDX_NONE,
      TP_WAIT_PLOCK },

  [IDX_2MS] =
  { { BW_DYN,      BW_DYN,            2,   TP_CTRL_PLL3,
      TP_TM_2MS_20MS,  TP_TM_2MS_10MS,  TP_TM_2MS_2MS,  TP_TM_2MS_NH20MS,  TP_TM_2MS_SC4 },
      TP_LD_PARAMS_PHASE_2MS, TP_LD_PARAMS_FREQ_2MS,
        40,          43,          0,
      IDX_2MS,     IDX_5MS,     IDX_NONE,
      TP_USE_NEXT | TP_LOW_CN0},

  [IDX_5MS] =
  { { BW_DYN,      BW_DYN,            1,   TP_CTRL_PLL3,
      TP_TM_5MS_20MS,  TP_TM_5MS_10MS,  TP_TM_2MS_2MS,  TP_TM_5MS_NH20MS,  TP_TM_4MS_SC4 },
      TP_LD_PARAMS_PHASE_5MS, TP_LD_PARAMS_FREQ_5MS,
      40,          35,          46,
      IDX_5MS,    IDX_10MS,     IDX_2MS,
      TP_USE_NEXT | TP_LOW_CN0 | TP_HIGH_CN0},

  [IDX_10MS] =
  { { BW_DYN,      BW_DYN,            1,   TP_CTRL_PLL3,
      TP_TM_10MS_20MS,  TP_TM_10MS_10MS,  TP_TM_2MS_2MS, TP_TM_10MS_NH20MS,  TP_TM_10MS_SC4 },
      TP_LD_PARAMS_PHASE_10MS, TP_LD_PARAMS_FREQ_10MS,
      40,          32,          38,
      IDX_10MS,    IDX_20MS,     IDX_5MS,
      TP_LOW_CN0 | TP_HIGH_CN0 | TP_USE_NEXT },

  [IDX_20MS] =
  { { BW_DYN,      BW_DYN,           .5,   TP_CTRL_PLL3,
      TP_TM_20MS_20MS,  TP_TM_10MS_10MS,  TP_TM_2MS_2MS,  TP_TM_20MS_NH20MS,  TP_TM_20MS_SC4 },
      TP_LD_PARAMS_PHASE_20MS, TP_LD_PARAMS_FREQ_20MS,
      40,        THRESH_SENS_DBHZ,   35,
      IDX_20MS,   IDX_SENS,     IDX_10MS,
      TP_LOW_CN0 | TP_HIGH_CN0 | TP_USE_NEXT },

  /* sensitivity profile */
  [IDX_SENS] =
  { {      0,         1.0,           .5,   TP_CTRL_PLL3,
      TP_TM_20MS_20MS,  TP_TM_10MS_10MS,  TP_TM_2MS_2MS,  TP_TM_20MS_NH20MS,  TP_TM_20MS_SC4 },
      TP_LD_PARAMS_PHASE_20MS, TP_LD_PARAMS_FREQ_20MS,
      300,             0,          32,
      IDX_SENS,  IDX_NONE,     IDX_20MS,
      TP_HIGH_CN0 | TP_USE_NEXT }
};
/* clang-format on */

/**
 * Helper method to get tracking profiles array.
 *
 * \param[in] mesid ME signal ID
 * \param[in] num_profiles number of profiles
 * \return Tracking profiles array pointer
 */
static const tp_profile_entry_t *mesid_to_profiles(const me_gnss_signal_t mesid,
                                                   size_t *num_profiles) {
  const tp_profile_entry_t *profiles = NULL;

  /* GPS and SBAS constellations use similar signal encoding scheme and thus
     share the same tracking profiles.
     For GLONASS signals we limit the maximum integration time to 10 ms.
     Otherwise we use the same set of tracking profiles. */

  switch (mesid_to_constellation(mesid)) {
    case CONSTELLATION_GPS:
    case CONSTELLATION_GLO:
    case CONSTELLATION_SBAS:
    case CONSTELLATION_BDS:
    case CONSTELLATION_QZS:
    case CONSTELLATION_GAL:
      profiles = gnss_track_profiles;
      if (num_profiles) {
        *num_profiles = ARRAY_SIZE(gnss_track_profiles);
      }
      break;

    case CONSTELLATION_INVALID:
    case CONSTELLATION_COUNT:
    default:
      assert(!"Invalid constellation");
      break;
  }

  return profiles;
}

/** Return track mode for the given code.
 * \param mesid ME signal ID
 * \param entry The profile details having track modes for different codes.
 * \return The track mode.
 */
static tp_tm_e get_track_mode(me_gnss_signal_t mesid,
                              const struct tp_profile_entry *entry) {
  tp_tm_e track_mode = TP_TM_INITIAL;

  if (IS_GPS(mesid) || IS_QZSS(mesid)) {
    if ((CODE_GPS_L5I == mesid.code) || (CODE_GPS_L5Q == mesid.code) ||
        (CODE_QZS_L5I == mesid.code) || (CODE_QZS_L5Q == mesid.code)) {
      track_mode = entry->profile.tm_nh20ms;
    } else {
      track_mode = entry->profile.tm_20ms;
    }
  } else if (IS_GLO(mesid)) {
    track_mode = entry->profile.tm_10ms;
  } else if (IS_SBAS(mesid)) {
    track_mode = entry->profile.tm_2ms;
  } else if (IS_BDS2(mesid)) {
    if (bds_d2nav(mesid)) {
      track_mode = entry->profile.tm_2ms;
    } else {
      track_mode = entry->profile.tm_nh20ms;
    }
  } else if (IS_GAL(mesid)) {
    if ((CODE_GAL_E1B == mesid.code) || (CODE_GAL_E7I == mesid.code)) {
      track_mode = entry->profile.tm_sc4;
    }
  } else {
    log_error_mesid(mesid, "unknown entry?");
    assert(0);
  }
  return track_mode;
}

static float compute_pll_bw(float cn0, u8 T_ms) {
  float y[2] = {PLL_BW_MIN, PLL_BW_MAX};   /* bw */
  float x[2] = {ADJ_CN0_MIN, ADJ_CN0_MAX}; /* cn0 */

  float m = (y[1] - y[0]) / (x[1] - x[0]);

  float bw = (cn0 - x[0]) * m + y[0];

  /* Form bandwidth * integration time product. */
  float bwt = bw * (float)T_ms / SECS_MS;

  /* Limit bandwidth so that loop stability criteria is satisfied. */
  if (bwt > TL_BWT_MAX) {
    bw = TL_BWT_MAX * SECS_MS / T_ms;
  }

  if (bw < PLL_BW_MIN) {
    bw = PLL_BW_MIN;
  }

  return bw;
}

static float compute_fll_bw(float cn0, u8 T_ms) {
  float y[2] = {FLL_BW_MIN, FLL_BW_MAX};   /* bw */
  float x[2] = {ADJ_CN0_MIN, ADJ_CN0_MAX}; /* cn0 */

  float m = (y[1] - y[0]) / (x[1] - x[0]);

  float bw = (cn0 - x[0]) * m + y[0];

  /* Form bandwidth * integration time product. */
  float bwt = bw * (float)T_ms / SECS_MS;

  /* Limit bandwidth so that loop stability criteria is satisfied. */
  if (bwt > TL_BWT_MAX) {
    bw = TL_BWT_MAX * SECS_MS / T_ms;
  }

  return bw;
}

static u8 get_profile_index(code_t code,
                            const tp_profile_entry_t *profiles,
                            size_t num_profiles,
                            float cn0) {
  if (code_requires_direct_acq(code) || is_gal(code)) {
    /* signals from ACQ always go through init profiles,
     * and also if they are Galileo as right now
     * the NAP secondary code stripping still has problems with FW */
    return 0;
  }

  /* the bit/symbol sync is known so we can start with non-init profiles */
  for (size_t i = 0; i < num_profiles; i++) {
    if (profiles[i].cn0_low_threshold <= 0) {
      continue;
    }
    if (cn0 > profiles[i].cn0_low_threshold) {
      return i;
    }
  }
  return IDX_SENS;
}

static struct profile_vars get_profile_vars(const me_gnss_signal_t mesid,
                                            float cn0) {
  size_t num_profiles = 0;
  const tp_profile_entry_t *profiles = mesid_to_profiles(mesid, &num_profiles);
  assert(profiles);

  u8 index = get_profile_index(mesid.code, profiles, num_profiles, cn0);

  struct profile_vars vars = {0};
  vars.index = index;

  const tp_profile_entry_t *entry = &profiles[index];
  if (entry->profile.pll_bw >= 0) { /* fixed PLL BW */
    vars.pll_bw = entry->profile.pll_bw;
  } else { /* dynamic PLL BW */
    tp_tm_e track_mode = get_track_mode(mesid, entry);
    u8 pll_t_ms = tp_get_pll_ms(track_mode);
    vars.pll_bw = compute_pll_bw(cn0, pll_t_ms);
  }

  if (entry->profile.fll_bw >= 0) { /* fixed FLL BW */
    vars.fll_bw = entry->profile.fll_bw;
  } else { /* dynamic FLL BW */
    tp_tm_e track_mode = get_track_mode(mesid, entry);
    u8 fll_t_ms = tp_get_flll_ms(track_mode);
    vars.fll_bw = compute_fll_bw(cn0, fll_t_ms);
  }

  return vars;
}

/**
 * Helper method to obtain tracking loop parameters.
 *
 * The method generates tracking loop parameters according to selected
 * configuration.
 *
 * \param tracker[in,out] Tracker channel data
 *
 * \return None
 */
void tp_profile_update_config(tracker_t *tracker) {
  const me_gnss_signal_t mesid = tracker->mesid;
  tp_profile_t *profile = &tracker->profile;
  const tp_profile_entry_t *cur_profile =
      &profile->profiles[profile->cur.index];

  if (IS_GPS(mesid)) {
    profile->ld_phase_params = ld_params_gps[cur_profile->ld_phase_params];
    profile->ld_freq_params = ld_params_gps[cur_profile->ld_freq_params];
  } else if (IS_GLO(mesid)) {
    profile->ld_phase_params = ld_params_glo[cur_profile->ld_phase_params];
    profile->ld_freq_params = ld_params_glo[cur_profile->ld_freq_params];
  } else if (IS_SBAS(mesid)) {
    profile->ld_phase_params = ld_params_sbas[cur_profile->ld_phase_params];
    profile->ld_freq_params = ld_params_sbas[cur_profile->ld_freq_params];
  } else if (IS_BDS2(mesid)) {
    profile->ld_phase_params = ld_params_bds2[cur_profile->ld_phase_params];
    profile->ld_freq_params = ld_params_bds2[cur_profile->ld_freq_params];
  } else if (IS_QZSS(mesid)) {
    profile->ld_phase_params = ld_params_qzss[cur_profile->ld_phase_params];
    profile->ld_freq_params = ld_params_qzss[cur_profile->ld_freq_params];
  } else if (IS_GAL(mesid)) {
    profile->ld_phase_params = ld_params_gal[cur_profile->ld_phase_params];
    profile->ld_freq_params = ld_params_gal[cur_profile->ld_freq_params];
  } else {
    assert(!"Unsupported constellation");
  }

  /* fill out the tracking loop parameters */
  profile->loop_params = loop_params_template;

  u16 flags = cur_profile->flags;
  double carr_to_code = 0.0;
  if (0 == (flags & TP_UNAIDED)) {
    carr_to_code = mesid_to_carr_to_code(mesid);
  }

  /* fill out the rest of tracking loop parameters */
  profile->loop_params.carr_to_code = carr_to_code;
  profile->loop_params.carr_bw = profile->cur.pll_bw;
  profile->loop_params.fll_bw = profile->cur.fll_bw;
  profile->loop_params.code_bw = cur_profile->profile.dll_bw;
  profile->loop_params.mode = get_track_mode(mesid, cur_profile);
  profile->loop_params.ctrl = cur_profile->profile.controller_type;

  tracker->flags &= ~TRACKER_FLAG_SENSITIVITY_MODE;
  if (profile->cur.pll_bw <= 0) {
    tracker->flags |= TRACKER_FLAG_SENSITIVITY_MODE;
  }

  const tp_tm_e mode = profile->loop_params.mode;
  profile->use_alias_detection =
      (TP_TM_10MS_10MS == mode) || (TP_TM_20MS_20MS == mode);
  tp_profile_get_cn0_params(profile, &profile->cn0_params);
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
 * \param tracker Tracker channel data
 * \param[in] reason Profile switching reason in a textual form
 *
 * \return None
 */
static void log_switch(tracker_t *tracker, const char *reason) {
  const me_gnss_signal_t mesid = tracker->mesid;
  const tp_profile_t *state = &tracker->profile;
  const tp_profile_entry_t *cur_profile = &state->profiles[state->cur.index];
  const tp_profile_entry_t *next_profile = &state->profiles[state->next.index];
  tp_tm_e cur_track_mode = get_track_mode(mesid, cur_profile);
  tp_tm_e next_track_mode = get_track_mode(mesid, next_profile);

  log_debug_mesid(mesid,
                  "%s:"
                  " cn0=%.1f "
                  "(mode,pll,fll,ctrl): (%s,%.1f,%.1f,%s)->(%s,%.1f,%.1f,%s)",
                  reason,
                  state->filt_cn0,
                  /* old state */
                  tp_get_mode_str(cur_track_mode),
                  state->cur.pll_bw,
                  state->cur.fll_bw,
                  get_ctrl_str(cur_profile->profile.controller_type),
                  /* new state */
                  tp_get_mode_str(next_track_mode),
                  state->next.pll_bw,
                  state->next.fll_bw,
                  get_ctrl_str(next_profile->profile.controller_type));
}

static bool pll_bw_changed(tracker_t *tracker, profile_indices_t index) {
  tp_profile_t *state = &tracker->profile;
  const tp_profile_entry_t *entry = &state->profiles[index];
  float pll_bw;

  if (entry->profile.pll_bw >= 0) { /* fixed PLL BW */
    pll_bw = entry->profile.pll_bw;
  } else { /* dynamic PLL BW */
    tp_tm_e track_mode = get_track_mode(tracker->mesid, entry);
    u8 pll_t_ms = tp_get_pll_ms(track_mode);
    pll_bw = compute_pll_bw(tracker->cn0, pll_t_ms);
  }

  /* Simple hysteresis to avoid too often PLL retunes */
  float pll_bw_diff = fabsf(pll_bw - state->cur.pll_bw);
  if ((pll_bw_diff < (state->cur.pll_bw * .20f)) || (pll_bw_diff < .5f)) {
    state->next.pll_bw = state->cur.pll_bw;
    return false;
  }

  if ((pll_bw > 0) && (pll_bw < state->cur.pll_bw)) {
    /* Reducing the PLL BW by more than 12 percent at a time could lead
       to PLL instabilities */
    if ((state->cur.pll_bw - pll_bw) > (0.12 * state->cur.pll_bw)) {
      pll_bw = (1 - 0.12) * state->cur.pll_bw;
    }
  }

  state->next.pll_bw = pll_bw;

  return true;
}

static bool fll_bw_changed(tracker_t *tracker, profile_indices_t index) {
  tp_profile_t *state = &tracker->profile;
  const tp_profile_entry_t *entry = &state->profiles[index];

  float fll_bw;
  if (entry->profile.fll_bw >= 0) { /* fixed FLL BW */
    fll_bw = entry->profile.fll_bw;
  } else { /* dynamic FLL BW */
    float cn0 = tracker->cn0;
    tp_tm_e track_mode = get_track_mode(tracker->mesid, entry);
    u8 fll_t_ms = tp_get_flll_ms(track_mode);
    fll_bw = compute_fll_bw(cn0, fll_t_ms);
  }

  /* Simple hysteresis to avoid too often FLL retunes */
  float fll_bw_diff = fabsf(fll_bw - state->cur.fll_bw);
  if ((fll_bw_diff < (state->cur.fll_bw * .10f)) || (fll_bw_diff < .3)) {
    state->next.fll_bw = state->cur.fll_bw;
    return false;
  }

  if (fll_bw < state->cur.fll_bw) {
    /* Reducing the FLL BW by more than 20 percent at a time could lead
       to FLL instabilities */
    if ((state->cur.fll_bw - fll_bw) > (0.20 * state->cur.fll_bw)) {
      fll_bw = (1 - 0.20) * state->cur.fll_bw;
    }
  }

  state->next.fll_bw = fll_bw;
  return true;
}

/**
 * Internal method for profile switch request.
 *
 * Sets the requested profile as the current one.
 *
 * \param tracker Tracker channel data
 * \param[in]     index  Index of profile to activate
 * \param[in]     reason Textual reason of profile switch
 *
 * \retval true Profile switch requested
 * \retval false No profile switch requested
 */
static bool profile_switch_requested(tracker_t *tracker,
                                     profile_indices_t index,
                                     const char *reason) {
  assert(index != IDX_NONE);
  assert((size_t)index < ARRAY_SIZE(gnss_track_profiles));

  tp_profile_t *state = &tracker->profile;
  const tp_profile_entry_t *next = &state->profiles[index];

  bool pll_changed = pll_bw_changed(tracker, index);
  bool fll_changed = fll_bw_changed(tracker, index);

  if ((index == state->cur.index) && !pll_changed && !fll_changed) {
    return false;
  }

  state->dll_init = false;
  const tp_profile_entry_t *cur = &state->profiles[state->cur.index];
  if ((0 != (cur->flags & TP_UNAIDED)) && (0 == (next->flags & TP_UNAIDED))) {
    /* Unaided DLL velocity causes instability when switching to aided DLL */
    state->dll_init = true;
  }

  state->profile_update = true;
  state->next.index = index;
  state->lock_time_ms = next->lock_time_ms;

  log_switch(tracker, reason);

  return true;
}

static bool low_cn0_profile_switch_requested(tracker_t *tracker) {
  tp_profile_t *state = &tracker->profile;
  const tp_profile_entry_t *cur_profile = &state->profiles[state->cur.index];

  if ((state->filt_cn0 < cur_profile->cn0_low_threshold) &&
      profile_switch_requested(tracker, cur_profile->next_cn0_low, "low cn0")) {
    return true;
  }

  bool confirmed = (0 != (tracker->flags & TRACKER_FLAG_CONFIRMED));
  if (!confirmed) {
    return false;
  }

  bool settled = (tracker->age_ms >= tracker->settle_time_ms);
  if (!settled) {
    return false;
  }

  if (tracker->cn0_est.weak_signal_ms > 0) {
    if ((state->filt_cn0 > THRESH_20MS_DBHZ) &&
        profile_switch_requested(tracker, IDX_SENS, "low cn0: instant")) {
      /* filt_cn0 reports a reasonably strong signal, but
         weak_signal_ms derived from raw CN0 says there is no signal.
         So we expedite the transition to sensitivity profile. */
      return true;
    }
    if ((tracker->cn0_est.weak_signal_ms >= TP_WEAK_SIGNAL_THRESHOLD_MS) &&
        profile_switch_requested(tracker, IDX_SENS, "low cn0: delay")) {
      return true;
    }
  }
  if ((state->filt_cn0 < THRESH_SENS_DBHZ) &&
      profile_switch_requested(tracker, IDX_SENS, "low cn0: sens")) {
    return true;
  }
  return false;
}

/**
 * Method to check if there is a pending profile change.
 *
 * \param tracker Tracker channel data
 *
 * \retval true  New profile is available.
 * \retval false No profile change is required.
 */
bool tp_profile_has_new_profile(tracker_t *tracker) {
  const tp_profile_entry_t *cur_profile;
  u16 flags;
  tp_profile_t *state = &tracker->profile;

  cur_profile = &state->profiles[state->cur.index];
  flags = cur_profile->flags;

  state->profile_update = false;

  if (0 != (flags & TP_LOW_CN0) && low_cn0_profile_switch_requested(tracker)) {
    return true;
  }

  bool bsync = (0 != (tracker->flags & TRACKER_FLAG_BIT_SYNC));
  if ((0 != (flags & TP_WAIT_BSYNC)) && !bsync) {
    return profile_switch_requested(tracker, state->cur.index, "wbsync");
  }

  bool flock = (0 != (tracker->flags & TRACKER_FLAG_HAS_FLOCK));
  if (0 != (flags & TP_WAIT_FLOCK) && !flock) {
    return profile_switch_requested(tracker, state->cur.index, "wflock");
  }

  bool plock = (0 != (tracker->flags & TRACKER_FLAG_HAS_PLOCK));
  if (0 != (flags & TP_WAIT_PLOCK) && !plock) {
    return profile_switch_requested(tracker, state->cur.index, "wplock");
  }

  if (state->lock_time_ms > 0) {
    return false; /* tracking loop has not settled yet */
  }

  if ((0 != (flags & TP_HIGH_CN0)) &&
      (state->filt_cn0 > cur_profile->cn0_high_threshold) &&
      profile_switch_requested(
          tracker, cur_profile->next_cn0_high, "high cno")) {
    return true;
  }

  if (0 != (flags & TP_USE_NEXT)) {
    assert(cur_profile->next != IDX_NONE);
    return profile_switch_requested(tracker, cur_profile->next, "next");
  } else {
    return profile_switch_requested(tracker, state->cur.index + 1, "next");
  }

  return false;
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

  cur_profile = &profile->profiles[profile->cur.index];
  mode = get_track_mode(mesid, cur_profile);

  u8 cn0_ms = tp_get_cn0_ms(mode);
  float cn0_offset = track_cn0_get_offset(cn0_ms);

  return cn0_offset;
}

/**
 * Registers GNSS satellite in facility.
 *
 * The method registers GNSS signal and returns initial tracking parameters.
 *
 * \param[in,out]  tracker Tracker channel data
 * \param[in]  data    Initial parameters.
 */
void tp_profile_init(tracker_t *tracker, const tp_report_t *data) {
  assert(tracker);

  tp_profile_t *profile = &tracker->profile;
  me_gnss_signal_t mesid = tracker->mesid;

  memset(profile, 0, sizeof(*profile));

  profile->filt_cn0 = data->cn0;
  profile->profiles = mesid_to_profiles(mesid, /* num_profiles = */ NULL);

  profile->cur = get_profile_vars(mesid, data->cn0);

  profile->profile_update = 0;

  profile->cn0_offset = compute_cn0_offset(mesid, profile);

  tp_profile_update_config(tracker);

  log_switch(tracker, "init");
}

void tp_profile_switch(tracker_t *tracker) {
  tp_profile_t *profile = &tracker->profile;
  assert(profile->profile_update);

  /* Do transition of current profile */
  profile->profile_update = 0;

  profile->cur = profile->next;
  profile->cn0_offset = compute_cn0_offset(tracker->mesid, profile);
}

/**
 * Method for obtaining current C/N0 thresholds.
 *
 * \param[in]  profile    Tracking profile data to check
 * \param[out] cn0_params Container for C/N0 limits.
 */
void tp_profile_get_cn0_params(const tp_profile_t *profile,
                               tp_cn0_params_t *cn0_params) {
  assert(cn0_params);
  assert(profile);

  *cn0_params = cn0_params_default;

  /* Correction: higher integration time lowers thresholds linearly. For
   * example, 20ms integration has threshold by 13 dB lower, than for 1ms
   * integration. */
  cn0_params->track_cn0_drop_thres_dbhz -= profile->cn0_offset;

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
}

/**
 * Updates track profile data with supplied information.
 *
 * The method takes tracking loop data and merges it with previously collected
 * information from other tracking loops.
 *
 * \param[in,out] profile     Tracking profile data to update
 * \param[in]     data        Tracking loop report.
 */
void tp_profile_report_data(tp_profile_t *profile, const tp_report_t *data) {
  assert(profile);
  assert(data);

  /* Profile lock time count down */
  if (profile->lock_time_ms > data->time_ms) {
    profile->lock_time_ms -= data->time_ms;
  } else {
    profile->lock_time_ms = 0;
  }

  profile->filt_cn0 = data->cn0;
}

tp_tm_e tp_profile_get_next_track_mode(const tp_profile_t *profile,
                                       me_gnss_signal_t mesid) {
  const tp_profile_entry_t *profile_entry;
  profile_entry = &profile->profiles[profile->next.index];
  return get_track_mode(mesid, profile_entry);
}
