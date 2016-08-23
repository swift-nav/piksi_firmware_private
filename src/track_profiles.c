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

#ifndef DEBUG
#define DEBUG 1
#endif

#include <platform_signal.h>
#include "track_profiles.h"
#include "track_profile_utils.h"
#include "chconf_board.h"

#include <libswiftnav/constants.h>
#include <libswiftnav/track.h>

#include <board.h>
#include <platform_signal.h>
#include <platform_cn0.h>
#include <nap/nap_common.h>
#include <nap/nap_hw.h>

#include <string.h>
#include <assert.h>

#include <nap/nap_hw.h>

/*
 * Configuration section: select which features are enabled here.
 */
//#define TP_USE_1MS_PROFILES
//#define TP_USE_2MS_PROFILES
#define TP_USE_5MS_PROFILES
#define TP_USE_10MS_PROFILES
#define TP_USE_20MS_PROFILES
#define TP_USE_20MS_PROFILES_FLL
// #define TP_USE_40MS_PROFILES

/*
 * Configure default PLL mode for longer integration periods:
 * - Simple pipelining (TP_TM_PIPELINING)
 * - One plus N (TP_TM_ONE_PLUS_N)
 * - 1 millisecond integrations (TP_TM_SPLIT)
 * - 5 millisecond integrations (TP_TM_ONE_PLUS_N5)
 * - 10 millisecond integrations (TP_TM_ONE_PLUS_N10)
 */

#define TP_TM_5MS_MODE  TP_TM_ONE_PLUS_N
#define TP_TM_10MS_MODE TP_TM_ONE_PLUS_N5
#define TP_TM_20MS_MODE TP_TM_ONE_PLUS_N5

/** Maximum number of supported satellite vehicles */
#define TP_MAX_SUPPORTED_SVS NUM_GPS_L1CA_TRACKERS
/** Helper macro for array size computation */
#define ARR_SIZE(x) (sizeof(x)/sizeof((x)[0]))

/** Default C/N0 threshold in dB/Hz for keeping track (for 1 ms integration) */
#define TP_DEFAULT_CN0_USE_THRESHOLD  (37.f)
/** Default C/N0 threshold in dB/Hz for dropping track (for 1 ms integration) */
#define TP_DEFAULT_CN0_DROP_THRESHOLD (31.f)

/** C/N0 threshold when we can't say if we are still tracking */
#define TP_HARD_CN0_DROP_THRESHOLD (10.f)

#define PCN0(x) TRACK_CN0_ADJUST(x)

/** C/N0 threshold state lock counter */
#define TP_SNR_STATE_COUNT_LOCK (/*31*/3)
/** PLL lock threshold for state freeze */
#define TP_LOCK_THRESHOLD (4.f)
/** Dynamics threshold for acceleration (stable) */
#define TP_ACCEL_THRESHOLD_LOW  (2.f)
/** Dynamics threshold for acceleration (unstable) */
#define TP_ACCEL_THRESHOLD_HIGH (9.f)
/** Dynamics threshold state lock counter */
#define TP_ACCEL_THRESHOLD_LOCK (31)

/** Profile lock time duration in ms. */
#define TP_CHANGE_LOCK_COUNTDOWN_MS (1250)
/** Profile change evaluation interval duration in ms. */
#define TP_CHANGE_LOCK_COUNTDOWN2_MS (500)

#define LPF_CUTOFF_HZ 0.6f

#define INTEG_PERIOD_1_MS  1
#define INTEG_PERIOD_5_MS  5
#define INTEG_PERIOD_10_MS 10
#define INTEG_PERIOD_20_MS 20

static const u8 integration_periods[] = {
  INTEG_PERIOD_1_MS,
  INTEG_PERIOD_5_MS,
  INTEG_PERIOD_10_MS,
  INTEG_PERIOD_20_MS
};

#define INTEG_PERIODS_NUM (sizeof(integration_periods) / \
                           sizeof(integration_periods[0]))

/** LP filter parameters: one pair per integration time */
static lp1_filter_params_t lp1_filter_params[INTEG_PERIODS_NUM] _BCKP = {
  { +3.067484e-01, 6.533742e-01 }, /* INTEG_PERIOD_1_MS */
  { -4.524422e-01, 2.737789e-01 }, /* INTEG_PERIOD_5_MS */
  { -6.827997e-01, 1.586001e-01 }, /* INTEG_PERIOD_10_MS */
  { -8.277396e-01, 8.613020e-02 }, /* INTEG_PERIOD_20_MS */
};

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

  float         cn0_offset;        /**< C/N0 offset in dB to tune thresholds */
  float         filt_cn0;          /**< C/N0 value for decision logic */
  float         filt_accel;        /**< SV acceleration value for decision logic */
  lp1_filter_t  filt_speed;        /**< Filter for speed */

  u32           used: 1;           /**< Flag if the profile entry is in use */
  u32           olock: 1;          /**< PLL optimistic lock flag */
  u32           plock: 1;          /**< PLL pessimistic lock flag */
  u32           bsync: 1;          /**< Bit sync flag */
  u32           profile_update:1;  /**< Flag if the profile update is required */
  u32           cur_profile_i:3;   /**< Index of the currently active profile (integration) */
  u32           cur_profile_d:2;   /**< Index of the currently active profile (dynamics) */
  u32           next_profile_i:3;  /**< Index of the next selected profile (integration) */
  u32           next_profile_d:2;  /**< Index of the next selected profile (dynamics)  */
  u32           low_cn0_count:5;   /**< State lock counter for C/N0 threshold */
  u32           high_cn0_count:5;  /**< State lock counter for C/N0 threshold */
  u32           accel_count:5;     /**< State lock counter for dynamics threshold */
  u32           accel_count_idx:2; /**< State lock value for dynamics threshold */
  u16           lock_time_ms:12;   /**< Profile lock count down timer */
  u16           cn0_est:2;

  tp_csid_t     csid;              /**< Compact satellite identifier */
  u16           print_time;        /**< Last debug print time */
} tp_profile_internal_t;

/**
 * GPS satellite profiles.
 */
static tp_profile_internal_t profiles_gps1[TP_MAX_SUPPORTED_SVS] _BCKP;

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
    .sat = (u16)sid.sat,
    .code = (s16)sid.code
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
  TP_LD_PARAMS_PESS
};

/**
 * Lock detector profiles
 */
static const tp_lock_detect_params_t ld_params[] = {
  /*   k1,    k2,  lp,  lo */
  { 0.02f, 1e-6f,   1,   1,}, /* TP_LD_PARAMS_DISABLE */
  { 0.02f,  0.8f,  50, 150,}, /* LD_PARAMS_EXTRAOPT */
  { 0.02f,  1.1f,  50, 150,}, /* LD_PARAMS_OPT */
  { 0.05f,  1.4f,  50, 150,}, /* LD_PARAMS_NORMAL */
  { 0.10f,  1.4f,  50, 200,}  /* TP_LD_PARAMS_PESS */
};

/**
 * Enumeration for the vertical dimension of profile matrix.
 *
 * Each entry here shall correspond to appropriate line in #profile_matrix.
 */
enum
{
  TP_PROFILE_ROW_INI=0,
#ifdef TP_USE_1MS_PROFILES
  TP_PROFILE_ROW_1MS,
#endif
#ifdef TP_USE_2MS_PROFILES
  TP_PROFILE_ROW_2MS,
#endif
#ifdef TP_USE_5MS_PROFILES
  TP_PROFILE_ROW_5MS,
#endif
#ifdef TP_USE_10MS_PROFILES
  TP_PROFILE_ROW_10MS,
#endif
#ifdef TP_USE_20MS_PROFILES
  TP_PROFILE_ROW_20MS,
#endif
#ifdef TP_USE_20MS_PROFILES_FLL
  TP_PROFILE_ROW_20MS_FLL,
#endif
#ifdef TP_USE_40MS_PROFILES
  TP_PROFILE_ROW_40MS,
#endif
  TP_PROFILE_ROW_COUNT,
  TP_PROFILE_ROW_FIRST = 1
};

/**
 * Dynamic profiles supported by the configuration.
 *
 * In the matrix of tracking loop parameters dynamics is identified by column
 * number.
 */
enum
{
  TP_PROFILE_DYN_LOW = 0,  /**< Low dynamics. Most stable parameters. */
  TP_PROFILE_DYN_MED,      /**< Medium dynamics. Averaged parameters. */
  TP_PROFILE_DYN_HIGH,     /**< High dynamics. Most relaxed parameters. */
  TP_PROFILE_DYN_COUNT,    /**< Total dynamic profile count */
  TP_PROFILE_DYN_INI = TP_PROFILE_DYN_MED /**< Initial dynamic profile */
};

/**
 * Vector of possible loop parameters.
 *
 * Entries do not have to have particular order, but the entry index shall
 * match the TP_LP_IDX_XYZ enumeration value.
 */
static const tp_loop_params_t loop_params[] = {
 /*
  * DLL              PLL          FLL MS LOOP_MODE     CONTROLLER
  * BW Zeta  K  C2C  BW   Zeta K  BW
  */
  /* "(1 ms, (1, 0.7, 1, 1540), (40, 0.7, 1, 5))" */
  { 1, 0.7f, 1, 1540, 16, .7f, 1, 20, 1, TP_TM_INITIAL, TP_CTRL_PLL2 }, /*TP_LP_IDX_INI*/

#ifdef TP_USE_1MS_PROFILES
  { 1, 1.f, 1, 1540, 12, 1.f, 1, 0, 1, TP_TM_PIPELINING, TP_CTRL_PLL2 }, /*TP_LP_IDX_1MS_S*/
  { 1, 1.f, 1, 1540, 20, 1.f, 1, 0, 1, TP_TM_PIPELINING, TP_CTRL_PLL2 }, /*TP_LP_IDX_1MS_N*/
  /* (1 ms, (1, 0.7, 1, 1540), (40, 0.7, 1, 5)) */
  { 1, .7f, 1, 1540, 40, .7f, 1, 0, 1, TP_TM_PIPELINING, TP_CTRL_PLL2 }, /*TP_LP_IDX_1MS_U*/
#endif /* TP_USE_1MS_PROFILES */

#ifdef TP_USE_2MS_PROFILES
  /* (2 ms, (1, 0.7, 1, 1540), (50, 0.7, 1, 0)) */
  { 1, 1.f, 1, 1540, 14, 1.f, 1, 0, 2, TP_TM_PIPELINING, TP_CTRL_PLL2 }, /* TP_LP_IDX_2MS*/
#endif /* TP_USE_2MS_PROFILES */

#ifdef TP_USE_5MS_PROFILES
  /* "(5 ms, (1, 0.7, 1, 1540), (50, 0.7, 1, 0))" */
  { 1, 0.7f, 1, 1540, 16, 1.f, 1, 0, 5, TP_TM_5MS_MODE, TP_CTRL_PLL2 }, /*TP_LP_IDX_5MS_S*/
  /* "(5 ms, (1, 0.7, 1, 1540), (50, 0.7, 1, 0))" */
  { 1, 0.7f, 1, 1540, 16, .7f, 1, 2, 5, TP_TM_5MS_MODE, TP_CTRL_PLL2 }, /*TP_LP_IDX_5MS_N*/
  /* "(5 ms, (1, 0.7, 1, 1540), (50, 0.7, 1, 0))" */
  { 1, 0.7f, 1, 1540, 16, .7f, 1, 10, 5, TP_TM_5MS_MODE, TP_CTRL_PLL2 }, /*TP_LP_IDX_5MS_U*/
#endif /* TP_USE_5MS_PROFILES */

#ifdef TP_USE_10MS_PROFILES
  /*  "(10 ms, (1, 0.7, 1, 1540), (30, 0.7, 1, 0))" */
  { 1, .7f, 1, 1540, 16, .7f, 1., 0, 10, TP_TM_10MS_MODE, TP_CTRL_PLL2 }, /*TP_LP_IDX_10MS*/
#endif /* TP_USE_10MS_PROFILES */

#ifdef TP_USE_20MS_PROFILES
  /*  "(20 ms, (1, 0.7, 1, 1540), (12, 0.7, 1, 0))" */
  { 1, .7f, 1, 1540, 7, .7f, 1.f, 1, 20, TP_TM_20MS_MODE, TP_CTRL_PLL2 }, /*TP_LP_IDX_20MS_S*/
  /*  "(20 ms, (1, 0.7, 1, 1540), (12, 0.7, 1, 0))" */
  { 1, .7f, 1, 1540, 8, .7f, 1.f, 1, 20, TP_TM_20MS_MODE, TP_CTRL_PLL2 },/*TP_LP_IDX_20MS_N*/
  /*  "(20 ms, (1, 0.7, 1, 1540), (12, 0.7, 1, 0))" */
  { 1, .7f, 1, 1540, 10, .7f, 1.f, 2, 20, TP_TM_20MS_MODE, TP_CTRL_PLL2 },/*TP_LP_IDX_20MS_U*/
#endif /* TP_USE_20MS_PROFILES */

#ifdef TP_USE_20MS_PROFILES_FLL
  /* FLL-assisted PLL. K_c = 1.2 */
  { .6, .7f, 1, 1540, 4, .7f, 1.f, 1, 20, TP_TM_20MS_MODE, TP_CTRL_FLL1 }, /*TP_LP_IDX_20MS_FLL*/
#endif

#ifdef TP_USE_40MS_PROFILES
  /*  "(40 ms, (1, 0.7, 1, 1540), (8, 0.7, 1, 0))" */
  { .5f, .7f, 1, 1540, 1, .7f, 1.f, 2, 40, TP_TM_ONE_PLUS_N2, TP_CTRL_FLL }, /*TP_LP_IDX_40MS_S*/
  { .7f, .7f, 1, 1540, 4, .7f, 1.f, 2, 40, TP_TM_ONE_PLUS_N2, TP_CTRL_FLL }, /*TP_LP_IDX_40MS_N*/
  { 1, .7f, 1, 1540, 4, .7f, 1.f, 1, 40, TP_TM_ONE_PLUS_N2, TP_CTRL_FLL }, /*TP_LP_IDX_40MS_U*/
#endif /* TP_USE_40MS_PROFILES */
};

/**
 * Enumeration of available profiles.
 *
 * Each entry here shall correspond to appropriate line in #loop_params. The
 * entries are used to index profiles in #profile_matrix table.
 */
enum
{
  TP_LP_IDX_INI,     /**< Initial state: very high noise bandwidth, high
                      * dynamics. */
#ifdef TP_USE_1MS_PROFILES
  TP_LP_IDX_1MS_S, /**< 1MS pipelining integration; stable */
  TP_LP_IDX_1MS_N, /**< 1MS pipelining integration; normal */
  TP_LP_IDX_1MS_U, /**< 1MS pipelining integration; unstable */
#endif /* TP_USE_1MS_PROFILES */

#ifdef TP_USE_2MS_PROFILES
  TP_LP_IDX_2MS,     /**< 2MS pipelining integration. */
#endif /* TP_USE_2MS_PROFILES */

#ifdef TP_USE_5MS_PROFILES
  TP_LP_IDX_5MS_S,   /**< 5MS 1+N integration; stable. */
  TP_LP_IDX_5MS_N,   /**< 5MS 1+N integration; normal. */
  TP_LP_IDX_5MS_U,   /**< 5MS 1+N integration; unstable. */
#endif /* TP_USE_5MS_PROFILES */

#ifdef TP_USE_10MS_PROFILES
  TP_LP_IDX_10MS,    /**< 10MS 1+N integration. */
#endif /* TP_USE_10MS_PROFILES */

#ifdef TP_USE_20MS_PROFILES
  TP_LP_IDX_20MS_S, /**< 20MS 1+N integration; stable. */
  TP_LP_IDX_20MS_N, /**< 20MS 1+N integration; normal. */
  TP_LP_IDX_20MS_U, /**< 20MS 1+N integration; unstable. */
#endif /* TP_USE_20MS_PROFILES */

#ifdef TP_USE_20MS_PROFILES_FLL
  TP_LP_IDX_20MS_FLL, /**< 20MS FLL-assisted DLL */
#endif /* TP_USE_20MS_PROFILES_FLL */

#ifdef TP_USE_40MS_PROFILES
  TP_LP_IDX_40MS_S,   /**< 40MS 1+N2 integration; stable. */
  TP_LP_IDX_40MS_N,   /**< 40MS 1+N2 integration; normal. */
  TP_LP_IDX_40MS_U,   /**< 40MS 1+N2 integration; unstable. */
#endif /* TP_USE_40MS_PROFILES */
};

typedef struct
{
  u8 cn0_min;
  u8 cn0_max;
  u8 ld_params;
  u8 loop_params[TP_PROFILE_DYN_COUNT];
} tp_loop_params_row_t;

/**
 * State transition matrix.
 *
 * Matrix is two-dimensional: first dimension enumerates integration times,
 * second dimension is the dynamics profile.
 */
static const tp_loop_params_row_t profile_matrix[] = {
  {PCN0(38), PCN0(60), TP_LD_PARAMS_NORMAL, {TP_LP_IDX_INI,  TP_LP_IDX_INI,  TP_LP_IDX_INI}},

#ifdef TP_USE_1MS_PROFILES
  {PCN0(38), PCN0(60), TP_LD_PARAMS_NORMAL, {TP_LP_IDX_1MS_S,  TP_LP_IDX_1MS_N,  TP_LP_IDX_1MS_U}},
#endif

#ifdef TP_USE_2MS_PROFILES
  {PCN0(35), PCN0(43), TP_LD_PARAMS_NORMAL, {TP_LP_IDX_2MS, TP_LP_IDX_2MS, TP_LP_IDX_2MS}},
#endif /* TP_USE_2MS_PROFILES */

#ifdef TP_USE_5MS_PROFILES
  {PCN0(37), PCN0(41), TP_LD_PARAMS_NORMAL, {TP_LP_IDX_5MS_S, TP_LP_IDX_5MS_N, TP_LP_IDX_5MS_U}},
#endif /* TP_USE_5MS_PROFILES */

#ifdef TP_USE_10MS_PROFILES
  {PCN0(30), PCN0(39), TP_LD_PARAMS_NORMAL, {TP_LP_IDX_10MS, TP_LP_IDX_10MS, TP_LP_IDX_10MS}},
#endif /* TP_USE_10MS_PROFILES */

#ifdef TP_USE_20MS_PROFILES
  {PCN0(25), PCN0(33), TP_LD_PARAMS_NORMAL, {TP_LP_IDX_20MS_S, TP_LP_IDX_20MS_N, TP_LP_IDX_20MS_U}},
#endif /* TP_USE_20MS_PROFILES */

#ifdef TP_USE_20MS_PROFILES_FLL
  {PCN0(20), PCN0(29), TP_LD_PARAMS_DISABLE, {TP_LP_IDX_20MS_FLL, TP_LP_IDX_20MS_FLL, TP_LP_IDX_20MS_FLL}},
#endif /* TP_USE_20MS_PROFILES_FLL */

#ifdef TP_USE_40MS_PROFILES
  {PCN0(20), PCN0(27), TP_LD_PARAMS_NORMAL, {TP_LP_IDX_40MS_S, TP_LP_IDX_40MS_N, TP_LP_IDX_40MS_U}},
#endif /* TP_USE_40MS_PROFILES */
};

/**
 * Helper method for computing GNSS satellite speed from doppler.
 *
 * The method converts doppler frequency shift relative vector speed towards
 * the line of sight.
 *
 * \params[in] sid  GNSS satellite signal identifier.
 * \params[in] data Satellite tracking loop report data.
 *
 * \returns Speed in meters per second, or 0. on error.
 */
static double compute_speed(gnss_signal_t sid, const tp_report_t *data)
{
  double speed_mps = 0.;
  double doppler_hz = data->carr_freq; /* Carrier frequency is actually a
                                        * doppler frequency shift */

  switch (sid.code) {
  case CODE_GPS_L1CA:
    speed_mps = -(double)GPS_L1_LAMBDA * doppler_hz;
    break;
  case CODE_GPS_L2CM:
  default:
    /* Do not support */
    break;
  }

  return speed_mps;
}

static const lp1_filter_params_t *get_lp1_params(u8 int_ms, lp1_filter_params_t *p)
{
  const lp1_filter_params_t *pparams = NULL;

  for (u32 i = 0; i < INTEG_PERIODS_NUM; i++) {
    if (int_ms == integration_periods[i]) {
      pparams = &lp1_filter_params[i];
      break;
    }
  }

  if (NULL == pparams) {
    float loop_freq = 1e3f / int_ms;
    lp1_filter_compute_params(p, LPF_CUTOFF_HZ, loop_freq);
    pparams = p;
  }

  return pparams;
}


/**
 * Helper method to (re-)initialize filters in satellite profile.
 *
 * \param[in,out] profile Satellite profile
 */
static void init_profile_filters(tp_profile_internal_t *profile, float speed0)
{
  u8 idx = profile_matrix[profile->cur_profile_i].loop_params[profile->cur_profile_d];
  const tp_loop_params_t *lp = &loop_params[idx];

  lp1_filter_params_t p;
  const lp1_filter_params_t *pp = get_lp1_params(lp->coherent_ms, &p);
  lp1_filter_init(&profile->filt_speed, pp, speed0);
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
  if (NULL != profile)
    /* Currently we support only one signal in profile, so simply mark the
     * profile as released. */
    memset(profile, 0, sizeof(*profile));
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
  u8 loop_profile_idx = profile_matrix[profile->cur_profile_i].loop_params[profile->cur_profile_d];
  u8 ld_params_idx = profile_matrix[profile->cur_profile_i].ld_params;

  config->lock_detect_params = ld_params[ld_params_idx];
  config->loop_params = loop_params[loop_profile_idx];

  /*
   * Alias detection is requires bit-aligned integration accumulator with equal
   * intervals.
   * The logic works with PLL in modes:
   * - 1+N modes with 5 and 10 ms.
   * - 1+N5 and 1+N10.
   */
  const tp_tm_e mode = config->loop_params.mode;
  const tp_ctrl_e ctrl = config->loop_params.ctrl;
  const u8 int_ms = config->loop_params.coherent_ms;
  if ((mode == TP_TM_ONE_PLUS_N5 || mode == TP_TM_ONE_PLUS_N10 ||
       (mode == TP_TM_ONE_PLUS_N && (int_ms == 5 || int_ms == 10))) &&
      (ctrl == TP_CTRL_PLL2 || ctrl == TP_CTRL_PLL3))
    config->use_alias_detection = true;
  else
    config->use_alias_detection = false;

  tp_get_cn0_params(unpack_sid(profile->csid), &config->cn0_params);
}

/**
 * Helper method to incorporate tracking loop information into statistics.
 *
 * \param[in,out] profile Satellite profile.
 * \param[in]     data    Data from tracking loop.
 *
 * \return None
 */
static void update_stats(tp_profile_internal_t *profile,
                         const tp_report_t *data)
{
  float loop_freq = 1000 / data->time_ms;
  float speed, accel, cn0;

  /* Profile lock time count down */
  if (profile->lock_time_ms > data->time_ms) {
    profile->lock_time_ms -= data->time_ms;
  } else {
    profile->lock_time_ms = 0;
  }
  profile->print_time += data->time_ms;

  profile->olock = data->olock;
  profile->plock = data->plock;
  profile->bsync = data->bsync;

  /* Compute products */
  speed = compute_speed(unpack_sid(profile->csid), data);

  /* Update moving average counters */
  lp1_filter_params_t p;
  const lp1_filter_params_t *pp = get_lp1_params(data->time_ms, &p);
  float speed0 = profile->filt_speed.yn;
  speed = lp1_filter_update(&profile->filt_speed, pp, speed);
  accel = (speed0 - speed) * loop_freq;
  cn0   = data->cn0;

  profile->filt_accel = accel;
  profile->filt_cn0 = cn0;
}

static const char *get_ctrl_str(tp_ctrl_e v)
{
  const char *str = "?";
  switch (v) {
  case TP_CTRL_PLL2: str = "PLL2"; break;
  case TP_CTRL_PLL3: str = "PLL3"; break;
  case TP_CTRL_FLL1: str = "FLL1"; break;
  case TP_CTRL_FLL2: str = "FLL2"; break;
  default: assert(false);
  }
  return str;
}

static const char *get_mode_str(tp_tm_e v)
{
  const char *str = "?";
  switch (v) {
  case TP_TM_INITIAL: str = "INI"; break;
  case TP_TM_IMMEDIATE: str = "IMD"; break;
  case TP_TM_PIPELINING: str = "PIP"; break;
  case TP_TM_SPLIT: str = "SPL"; break;
  case TP_TM_ONE_PLUS_N: str = "1+N"; break;
  case TP_TM_ONE_PLUS_N5: str = "1+5N"; break;
  case TP_TM_ONE_PLUS_N10: str = "1+10N"; break;
  case TP_TM_ONE_PLUS_N20: str = "1+20N"; break;
  default: assert(false);
  }
  return str;
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
  if (profile->print_time < 20000)
    return;

  profile->print_time = 0;

  u8 lp_idx = profile_matrix[profile->cur_profile_i].loop_params[profile->cur_profile_d];

  const char *cn0_est_str = track_cn0_str(profile->cn0_est);
  const char *c1 = get_ctrl_str(loop_params[lp_idx].ctrl);
  const char *m1 = get_mode_str(loop_params[lp_idx].mode);

  /*
   * PRINT: integration time, loop mode, controller mode,
   *        C/N0 estimator, C/N0 value, SNR value (dBm),
   *        PR rate, PR rate change,
   *        PLL lock detector ratio, FLL/DLL error
   */
  log_debug_sid(unpack_sid(profile->csid),
                "AVG: %dms %s %s CN0_%s=%.2f (%.2f) A=%.3f",
                (int)loop_params[lp_idx].coherent_ms, m1, c1,
                cn0_est_str, profile->filt_cn0,
                TRACK_CN0_TO_SNR(profile->filt_cn0),
                profile->filt_accel
               );
}

/**
 * Internal method for evaluating profile change conditions.
 *
 * This method analyzes collected statistics and selects appropriate tracking
 * parameter changes.
 */
static void check_for_profile_change(tp_profile_internal_t *profile)
{
  /** TODO refactor as needed */
  /** TODO add TCXO drift support */

  bool        must_change_profile = false;
  bool        must_keep_profile   = false;
  u8          next_profile_i      = 0;
  u8          next_profile_d      = 0;
  const char *reason              = "cn0 OK";
  const char *reason2             = "dynamics OK";
  float       cn0 = 0.;
  float       acc = 0.;

  cn0 = profile->filt_cn0;
  acc = profile->filt_accel;

  /*
   * Switching C/N0 estimator: C/N0 estimator is switched when the C/N0 value
   * gets close to operation limit or the loop type is not compatible with
   * the current estimator.
   * Example: BL estimator requires high C/N0 values and PLL mode. MM estimator
   * shows incorrect values when SNR is high, but good with low SNR and
   * compatible with FLL mode.
   */
  u8 lp_idx = profile_matrix[profile->cur_profile_i].loop_params[profile->cur_profile_d];
  tp_ctrl_e ctrl = loop_params[lp_idx].ctrl;

  switch (profile->cn0_est) {
  case TRACK_CN0_EST_PRIMARY:
    if (cn0 < TRACK_CN0_PRI2SEC_THRESHOLD - profile->cn0_offset ||
        ctrl == TP_CTRL_FLL1 || ctrl == TP_CTRL_FLL2) {
      profile->cn0_est = TRACK_CN0_EST_SECONDARY;
      log_debug_sid(unpack_sid(profile->csid),
                    "Changed C/N0 estimator to secondary");
    }
    break;

  case TRACK_CN0_EST_SECONDARY:
    if (cn0 > TRACK_CN0_SEC2PRI_THRESHOLD - profile->cn0_offset &&
        ctrl != TP_CTRL_FLL1 && ctrl != TP_CTRL_FLL2) {
      profile->cn0_est = TRACK_CN0_EST_PRIMARY;
      log_debug_sid(unpack_sid(profile->csid),
                    "Changed C/N0 estimator to primary");
    }
    break;

  default:
    assert(false);
  }

  /* When we have a lock, and lock ratio is good, do not change the mode */
  // must_keep_profile = profile->olock && profile->filt_val[2] > 4.f;

  /* First, check if the profile change is required:
   * - There must be no scheduled profile change.
   * - For an upgrade from initial state, a bit sync shall be achieved.
   */
  if (!profile->profile_update) {
    if (profile->cur_profile_i == 0 &&
        profile->bsync &&
        profile->olock) {
      /* Transition from 1ms integration into 2 to 20 ms integration */
      must_change_profile = true;
      next_profile_i = 1;
      reason = "bit sync";
      profile->high_cn0_count = 0;
      profile->low_cn0_count = 0;
      profile->lock_time_ms = TP_CHANGE_LOCK_COUNTDOWN_MS;
    }
    if (!must_keep_profile &&
        !must_change_profile &&
        profile->cur_profile_i != TP_LP_IDX_INI &&
        profile->lock_time_ms == 0) {
      /* When running over 1ms integration, there are four transitions
       * possible:
       * - increase integration time
       * - reduce integration time
       * - tighten loop parameters
       * - loosen loop parameters
       */
      /* Current C/N0 integration adjustments assume the integration times
       * are around factor of 2. This means there is ~3 dB/Hz gain/loss when
       * increasing/decreasing integration times.
       */
      if (profile->cur_profile_i > TP_PROFILE_ROW_FIRST && profile->cur_profile_i < TP_PROFILE_ROW_COUNT ) {
        u8 cn0_limit1 = profile_matrix[profile->cur_profile_i].cn0_max;
        u8 cn0_limit2 = profile_matrix[profile->cur_profile_i - 1].cn0_min;
        if (cn0 >= cn0_limit1 && cn0 >= cn0_limit2 /* && loc < TP_LOCK_THRESHOLD */) {
        /* SNR is high - look for relaxing profile */
          profile->high_cn0_count++;
          profile->low_cn0_count = 0;

          if (profile->high_cn0_count == TP_SNR_STATE_COUNT_LOCK) {
            reason="High C/N0";
            profile->high_cn0_count = 0;
            profile->lock_time_ms = TP_CHANGE_LOCK_COUNTDOWN_MS;
            must_change_profile = true;
            next_profile_i = profile->cur_profile_i - 1;
            next_profile_d = profile->cur_profile_d;
          }
        }
      }
      if (profile->cur_profile_i < TP_PROFILE_ROW_COUNT - 1) {
        u8 cn0_limit = profile_matrix[profile->cur_profile_i].cn0_min;
        if (cn0 < cn0_limit) {
        /* SNR is low - look for more restricting profile */
          profile->high_cn0_count = 0;
          profile->low_cn0_count++;
          if (profile->low_cn0_count == TP_SNR_STATE_COUNT_LOCK) {
            reason="Low C/N0";
            profile->low_cn0_count = 0;
            profile->lock_time_ms = TP_CHANGE_LOCK_COUNTDOWN_MS;
            must_change_profile = true;
            next_profile_i = profile->cur_profile_i + 1;
            next_profile_d = profile->cur_profile_d;
          }
        }
      }
    }
  }
  /* Compute dynamics state.
   * Dynamics is evaluated as a delayed locked-in state as a function of
   * averaged acceleration.
   * At the moment three dynamic states are supported: low, medium and high
   * dynamics. They are separated by an acceleration trigger.
   */
  if (!must_keep_profile && profile->lock_time_ms == 0) {
    const char *dyn_reason = "";
    u8 dyn_idx = TP_PROFILE_DYN_INI;
    if (acc < TP_ACCEL_THRESHOLD_LOW) {
      dyn_idx = TP_PROFILE_DYN_LOW;
      dyn_reason = "Lower dynamics";
    } else if (acc > TP_ACCEL_THRESHOLD_HIGH) {
      dyn_idx = TP_PROFILE_DYN_HIGH;
      dyn_reason = "High dynamics";
    } else {
      dyn_idx = TP_PROFILE_DYN_MED;
      dyn_reason = "Normal dynamics";
    }
    if (profile->accel_count_idx == dyn_idx) {
      /* When the computed state matches last state, check if the state is
       * already active or will be active in next stage */
      if (!must_change_profile && profile->cur_profile_d == dyn_idx) {
        /* State is already active */
      } else if (must_change_profile && profile->next_profile_d == dyn_idx) {
        /* Next state is already selected */
      } else {
        profile->accel_count++;
        if (profile->accel_count == TP_ACCEL_THRESHOLD_LOCK) {
          /* State lock achieved. Reset counters. */
          profile->accel_count = 0;
          reason2 = dyn_reason;
          if (must_change_profile) {
            /* Profile change is already pending, update the state if the
             * dynamics is not lower. Switching to longer integration time with
             * higher dynamics may lead to loosing the track */
            if (dyn_idx >= profile->cur_profile_d)
              next_profile_d = dyn_idx;
          } else {
            /* Profile change due to dynamics state change only */
            /* Good PLL lock protection: no switch if lock is high */
            // if (loc < TP_LOCK_THRESHOLD)
            {
              next_profile_i = profile->cur_profile_i;
              next_profile_d = dyn_idx;
              must_change_profile = true;
              profile->lock_time_ms = TP_CHANGE_LOCK_COUNTDOWN_MS;
            }
          }
        }
      }
    } else {
      /* Dynamics state change reset: the previous dynamics state lock
       * to be reset*/
      profile->accel_count = 0;
      profile->accel_count_idx = dyn_idx;
    }
  } else {
    /* Dynamics state change is not permitted due to the lock */
    profile->accel_count = 0;
    profile->accel_count_idx = profile->cur_profile_d;
  }

  if (must_change_profile) {
    /* Perform profile change */
    /* Profile update scheduling:
     * - Mark profile as pending for change
     * - Specify profile configuration index
     * - Log the information
     */
    profile->profile_update = true;
    profile->next_profile_i = next_profile_i;
    profile->next_profile_d = next_profile_d;

    u8 lp1_idx = profile_matrix[profile->cur_profile_i].loop_params[profile->cur_profile_d];
    u8 lp2_idx = profile_matrix[profile->next_profile_i].loop_params[profile->next_profile_d];

    if (lp1_idx == lp2_idx && profile->cur_profile_i == profile->next_profile_i) {
      profile->cur_profile_i = profile->next_profile_i;
      profile->profile_update = false;
    }

    const char *c1 = get_ctrl_str(loop_params[lp1_idx].ctrl);
    const char *m1 = get_mode_str(loop_params[lp1_idx].mode);
    const char *c2 = get_ctrl_str(loop_params[lp2_idx].ctrl);
    const char *m2 = get_mode_str(loop_params[lp2_idx].mode);

    log_info_sid(unpack_sid(profile->csid),
                 "Profile change: %dms %s %s [%d][%d]->%dms %s %s [%d][%d] r=%s (%.2f)/%s (%.2f)",
                 (int)loop_params[lp1_idx].coherent_ms, m1, c1,
                 profile->cur_profile_i, profile->cur_profile_d,
                 (int)loop_params[lp2_idx].coherent_ms, m2, c2,
                 profile->next_profile_i, profile->next_profile_d,
                 reason, cn0,
                 reason2, acc
                 );
  } else if (profile->lock_time_ms == 0) {
    // profile->lock_time_ms = TP_CHANGE_LOCK_COUNTDOWN2_MS;
  }
}

/**
 * Helper method for computing C/N0 offset.
 *
 * The method computes C/N0 offset for tracking loop in accordance to
 * integration period and tracking parameters.
 *
 * \param[in] profile_i SNR index for profile
 * \param[in] profile_d Dynamics index for profile
 *
 * \return Computed C/N0 offset in dB/Hz.
 */
static float compute_cn0_profile_offset(u8 profile_i, u8 profile_d)
{
  u8 profile_idx = profile_matrix[profile_i].loop_params[profile_d];
  const tp_loop_params_t *lp = &loop_params[profile_idx];
  float cn0_offset = 0;

  u8 cn0_ms = tp_get_cn0_ms(lp->mode, lp->coherent_ms);
  cn0_offset = 10.f * log10f(cn0_ms);

  return cn0_offset;
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
  return compute_cn0_profile_offset(profile->cur_profile_i,
                                    profile->cur_profile_d);
}

/**
 * Initializes the subsystem.
 *
 * This method shall be invoked before any other methods from the subsystem.
 *
 * \return 0  On success.
 * \return -1 On error.
 */
tp_result_e tp_init()
{
  memset(profiles_gps1, 0, sizeof(profiles_gps1));

  for(u32 i = 0; i < INTEG_PERIODS_NUM; i++) {
    float loop_freq = 1e3f / integration_periods[i];
    lp1_filter_compute_params(&lp1_filter_params[i], LPF_CUTOFF_HZ, loop_freq);
  }

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
      profile->cn0_est = TRACK_CN0_EST_PRIMARY;

      float speed0 = compute_speed(sid, data);
      profile->filt_cn0 = data->cn0;
      profile->filt_accel = 0;

      profile->cur_profile_i = TP_PROFILE_ROW_INI;
      profile->next_profile_i = TP_PROFILE_ROW_INI;
      profile->cur_profile_d = TP_PROFILE_DYN_INI;
      profile->next_profile_d = TP_PROFILE_DYN_INI;

      init_profile_filters(profile, speed0);

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
        profile->cur_profile_i = profile->next_profile_i;
        profile->cur_profile_d = profile->next_profile_d;
        profile->cn0_offset = compute_cn0_offset(profile);
        init_profile_filters(profile, profile->filt_speed.yn);
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
const tp_loop_params_t *tp_get_next_loop_params(gnss_signal_t sid)
{
  const tp_loop_params_t *res = &loop_params[0];
  tp_profile_internal_t *profile = find_profile(sid);
  if (NULL != profile) {
    u8 lp_idx = profile_matrix[profile->next_profile_i].
        loop_params[profile->next_profile_d];
    res = &loop_params[lp_idx];
  }
  return res;
}

/**
 * Updates track profile data with supplied information.
 *
 * The method takes tracking loop data and merges it with previously collected
 * information from other tracking loops.
 *
 * \param[in] sid  GNSS signal identifier. This identifier must be registered
 *                 with a call to #tp_tracking_start().
 * \param[in] data Tracking loop report. This data is taken for analysis and
 *                 can be asynchronously.
 *
 * \retval TP_RESULT_SUCCESS on success.
 * \retval TP_RESULT_ERROR   on error.
 */
tp_result_e tp_report_data(gnss_signal_t sid, const tp_report_t *data)
{
  tp_result_e res = TP_RESULT_ERROR;
  tp_profile_internal_t *profile = find_profile(sid);
  if (NULL != data && NULL != profile) {
    /* For now, we support only GPS L1 tracking data, and handle all data
     * synchronously.
     *
     * TODO schedule a message to own thread.
     */

    update_stats(profile, data);
    print_stats(profile);

    res = TP_RESULT_SUCCESS;
  }
  return res;
}
