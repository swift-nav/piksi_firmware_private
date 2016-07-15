/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "track_gps_l1ca.h"
#include "track_gps_l2cm.h" /* for L1C/A to L2 CM tracking handover */
#include "track_api.h"
#include "track_cn0.h"

#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/track.h>

#include <string.h>
#include <assert.h>

#include "settings.h"
#include "signal.h"
#include "board.h"
#include "platform_signal.h"
#include "track_profiles.h"

/* Convert milliseconds to L1C/A chips */
#define L1CA_TRACK_MS_TO_CHIPS(ms) ((ms) * GPS_L1CA_CHIPS_NUM)

/*
 * Initial tracking: loop selection
 *
 * The initial tracking doesn't have bit lock and is using pipelining. The
 * coefficient computation differs from non-pipelining tracking modes.
 */

/*
 * Main tracking: PLL loop selection
 */
/*
 * Second order PLL:
 * - Optional first order FLL assist to PLL
 * - Optional PLL assist to DLL
 */
#define tl_pll2_state_t        aided_tl_state_t
#define tl_pll2_init           aided_tl_init
#define tl_pll2_retune         aided_tl_retune
#define tl_pll2_update         aided_tl_update
#define tl_pll2_adjust         aided_tl_adjust
#define tl_pll2_get_dll_error  aided_tl_get_dll_error

#if 0
/* PLL-assisted DLL. FLL and DLL are second order, PLL is third order */
#define tl_pll3_state_t        aided_tl_state3_t
#define tl_pll3_init           aided_tl_init3
#define tl_pll3_retune         aided_tl_retune3
#define tl_pll3_update         aided_tl_update3
#define tl_pll3_adjust         aided_tl_adjust3
#define tl_pll3_get_dll_error  aided_tl_get_dll_error3
#elif 1
/* PLL-assisted DLL. FLL and DLL are second order, PLL is third order */
#define tl_pll3_state_t        aided_tl_state3b_t
#define tl_pll3_init           aided_tl_init3b
#define tl_pll3_retune         aided_tl_retune3b
#define tl_pll3_update         aided_tl_update3b
#define tl_pll3_adjust         aided_tl_adjust3b
#define tl_pll3_get_dll_error  aided_tl_get_dll_error3b
#endif

/*
 * Main tracking: FLL loop selection
 */

/* FLL-assisted DLL. FLL is first order and DLL is second order */
#define tl_fll1_state_t        aided_tl_state_fll1_t
#define tl_fll1_init           aided_tl_fll1_init
#define tl_fll1_retune         aided_tl_fll1_retune
#define tl_fll1_update         aided_tl_fll1_update
#define tl_fll1_adjust         aided_tl_fll1_adjust
#define tl_fll1_get_dll_error  aided_tl_fll1_get_dll_error

/* FLL-assisted DLL. FLL and DLL are both second order */
#define tl_fll2_state_t        aided_tl_state_fll2_t
#define tl_fll2_init           aided_tl_fll2_init
#define tl_fll2_retune         aided_tl_fll2_retune
#define tl_fll2_update         aided_tl_fll2_update
#define tl_fll2_adjust         aided_tl_fll2_adjust
#define tl_fll2_get_dll_error  aided_tl_fll2_get_dll_error

typedef struct {
  union {
    tl_pll2_state_t   pll2_state;          /**< Tracking loop filter state. */
    tl_pll3_state_t   pll3_state;          /**< Tracking loop filter state. */
    tl_fll1_state_t   fll1_state;          /**< Tracking loop filter state. */
    tl_fll2_state_t   fll2_state;          /**< Tracking loop filter state. */
  };
  corr_t           cs[3];                  /**< EPL correlation results in
                                            *   correlation period. */
  track_cn0_est_e  cn0_est_type;           /**< C/N0 estimator type */
  track_cn0_state_t cn0_est;                /**< C/N0 estimator state. */
  alias_detect_t   alias_detect;           /**< Alias lock detector. */
  lock_detect_t    lock_detect;            /**< Phase-lock detector state. */
  lp1_filter_params_t fll_lock_params;
  lp1_filter_t     fll_lock_detect;
  u8               int_ms;                 /**< Current integration length. */
  u8               cycle_no: 5;            /**< Number of cycle inside current
                                            *   integration mode. */
  u8               use_alias_detection: 1; /**< Flag for alias detection control */
  u8               tracking_mode: 3;       /**< Tracking mode */
  u8               tracking_ctrl: 2;       /**< Tracking controller type */
  u8               has_next_params: 1;     /**< Flag if stage transition is in
                                            *   progress */
} gps_l1ca_tracker_data_t;

static tracker_t gps_l1ca_trackers[NUM_GPS_L1CA_TRACKERS];
static gps_l1ca_tracker_data_t gps_l1ca_tracker_data[NUM_GPS_L1CA_TRACKERS];

static void tracker_gps_l1ca_init(const tracker_channel_info_t *channel_info,
                                  tracker_common_data_t *common_data,
                                  tracker_data_t *tracker_data);
static void tracker_gps_l1ca_disable(const tracker_channel_info_t *channel_info,
                                     tracker_common_data_t *common_data,
                                     tracker_data_t *tracker_data);
static void tracker_gps_l1ca_update(const tracker_channel_info_t *channel_info,
                                    tracker_common_data_t *common_data,
                                    tracker_data_t *tracker_data);

static const tracker_interface_t tracker_interface_gps_l1ca = {
  .code =         CODE_GPS_L1CA,
  .init =         tracker_gps_l1ca_init,
  .disable =      tracker_gps_l1ca_disable,
  .update =       tracker_gps_l1ca_update,
  .trackers =     gps_l1ca_trackers,
  .num_trackers = NUM_GPS_L1CA_TRACKERS
};

static tracker_interface_list_element_t
tracker_interface_list_element_gps_l1ca = {
  .interface = &tracker_interface_gps_l1ca,
  .next = 0
};

void track_gps_l1ca_register(void)
{
  for (u32 i=0; i<NUM_GPS_L1CA_TRACKERS; i++) {
    gps_l1ca_trackers[i].active = false;
    gps_l1ca_trackers[i].data = &gps_l1ca_tracker_data[i];
  }

  tracker_interface_register(&tracker_interface_list_element_gps_l1ca);
}

/**
 * Get C/N0 estimator update period in ms.
 *
 * \param[in] data Tracker data.
 *
 * \return C/N0 estimator update period in ms.
 */
static u8 get_cn0_ms(const gps_l1ca_tracker_data_t *data)
{
  u8 cn0_ms = data->int_ms;
  switch (data->tracking_mode) {
  case TP_TM_INITIAL:
  case TP_TM_IMMEDIATE:
  case TP_TM_PIPELINING:
  case TP_TM_ONE_PLUS_N:
  case TP_TM_ONE_PLUS_N20:
    cn0_ms = data->int_ms;
    break;

  case TP_TM_ONE_PLUS_N5:
    cn0_ms = 5;
    break;

  case TP_TM_SPLIT:
    cn0_ms = 1;
    break;

  default:
    assert(false);
  }

  return cn0_ms;
}

static void tracker_gps_l1ca_update_parameters(
    const tracker_channel_info_t *channel_info,
    tracker_common_data_t *common_data,
    gps_l1ca_tracker_data_t *data,
    const tp_config_t *next_params,
    bool init)
{
  const tp_loop_params_t *l = &next_params->loop_params;
  const tp_lock_detect_params_t *ld = &next_params->lock_detect_params;

  const float old_loop_freq = 1000.f / data->int_ms;
  u8 old_cn0_ms = get_cn0_ms(data);

  if (data->tracking_mode == TP_TM_INITIAL && next_params->loop_params.mode != TP_TM_INITIAL) {
    init = true;
  }

  data->tracking_mode = next_params->loop_params.mode;
  tp_ctrl_e prev_ctrl = data->tracking_ctrl;
  data->tracking_ctrl = next_params->loop_params.ctrl;
  bool use_alias_detection = data->use_alias_detection;
  data->use_alias_detection = next_params->use_alias_detection;

  /*
   * Coherent integration is possible for intervals longer, than bit length,
   * assuming that accumulated bit value error is sufficiently small to allow
   * data wipe off.
   *
   * data->int_ms = MIN(l->coherent_ms,
   *                    tracker_bit_length_get(channel_info->context));
   */
  data->int_ms = next_params->loop_params.coherent_ms;
  data->has_next_params = false;

  /* Set the step number for mode switch. Current step is one step behind bit
   * edge */
  switch (l->mode) {
  case TP_TM_SPLIT:
    data->cycle_no = data->int_ms - 1;
    break;

  case TP_TM_ONE_PLUS_N:
    data->cycle_no = 1;
    break;

  case TP_TM_ONE_PLUS_N20:
    data->cycle_no = data->int_ms / 20;
    break;

  case TP_TM_ONE_PLUS_N5:
    data->cycle_no = data->int_ms / 5;
    break;

  case TP_TM_INITIAL:
  case TP_TM_PIPELINING:
  case TP_TM_IMMEDIATE:
    data->cycle_no = 0;
    break;

  default:
    assert(false);
  }

  float loop_freq = 1000.f / data->int_ms; /**< Tracking loop frequency */
  u8 cn0_ms = get_cn0_ms(data);   /**< C/N0 integration time */
  float ld_int_ms = data->int_ms; /**< Lock detector integration time */

  if (data->tracking_mode == TP_TM_SPLIT) {
    /* 1ms coherent interval split is used. */
    ld_int_ms = 1;
  } else if (data->tracking_mode == TP_TM_ONE_PLUS_N5) {
    ld_int_ms = 5;
  } else if (data->tracking_mode == TP_TM_ONE_PLUS_N20) {
    /* 20+ms coherent interval split is used. */
    // ld_int_ms = 20;
  }

  if (init || prev_ctrl != data->tracking_ctrl) {
    log_debug_sid(channel_info->sid, "Initializing TL");

    switch (data->tracking_ctrl) {
    case TP_CTRL_PLL2:
      tl_pll2_init(&data->pll2_state, loop_freq,
                   common_data->code_phase_rate - GPS_CA_CHIPPING_RATE,
                   l->code_bw, l->code_zeta, l->code_k,
                   l->carr_to_code,
                   common_data->carrier_freq,
                   l->carr_bw, l->carr_zeta, l->carr_k,
                   l->carr_fll_aid_gain);
      break;
    case TP_CTRL_PLL3:
      tl_pll3_init(&data->pll3_state, loop_freq,
                   common_data->code_phase_rate - GPS_CA_CHIPPING_RATE,
                   l->code_bw, l->code_zeta, l->code_k,
                   l->carr_to_code,
                   common_data->carrier_freq,
                   l->carr_bw, l->carr_zeta, l->carr_k,
                   l->carr_fll_aid_gain);
      break;
    case TP_CTRL_FLL1:
      tl_fll1_init(&data->fll1_state, loop_freq,
                   common_data->code_phase_rate - GPS_CA_CHIPPING_RATE,
                   l->code_bw, l->code_zeta, l->code_k,
                   l->carr_to_code,
                   common_data->carrier_freq,
                   l->carr_bw, l->carr_zeta, l->carr_k,
                   l->carr_fll_aid_gain);
      break;
    case TP_CTRL_FLL2:
      tl_fll2_init(&data->fll2_state, loop_freq,
                   common_data->code_phase_rate - GPS_CA_CHIPPING_RATE,
                   l->code_bw, l->code_zeta, l->code_k,
                   l->carr_to_code,
                   common_data->carrier_freq,
                   l->carr_bw, l->carr_zeta, l->carr_k,
                   l->carr_fll_aid_gain);
      break;
    }

    // log_info_sid(channel_info->sid, "LF=%f", common_data->carrier_freq);

    lock_detect_init(&data->lock_detect,
                     ld->k1 * ld_int_ms,
                     ld->k2,
                     ld->lp,
                     ld->lo);

  } else {
    log_debug_sid(channel_info->sid, "Re-tuning TL");

    /* Recalculate filter coefficients */
    switch (data->tracking_ctrl) {
    case TP_CTRL_PLL2:
      tl_pll2_retune(&data->pll2_state, loop_freq,
                     l->code_bw, l->code_zeta, l->code_k,
                     l->carr_to_code,
                     l->carr_bw, l->carr_zeta, l->carr_k,
                     l->carr_fll_aid_gain);
      break;
    case TP_CTRL_PLL3:
      tl_pll3_retune(&data->pll3_state, loop_freq,
                     l->code_bw, l->code_zeta, l->code_k,
                     l->carr_to_code,
                     l->carr_bw, l->carr_zeta, l->carr_k,
                     l->carr_fll_aid_gain);
      break;
    case TP_CTRL_FLL1:
      tl_fll1_retune(&data->fll1_state, loop_freq,
                     l->code_bw, l->code_zeta, l->code_k,
                     l->carr_to_code,
                     l->carr_bw, l->carr_zeta, l->carr_k,
                     l->carr_fll_aid_gain);
      break;
    case TP_CTRL_FLL2:
      tl_fll2_retune(&data->fll2_state, loop_freq,
                     l->code_bw, l->code_zeta, l->code_k,
                     l->carr_to_code,
                     l->carr_bw, l->carr_zeta, l->carr_k,
                     l->carr_fll_aid_gain);
      break;
    default:
      assert(false);
    }

    if (old_loop_freq != loop_freq) {
      /* When loop frequency changes, reset partially reset filter state. */
      // data->tl_state.carr_filt.prev_error = 0.f;
      // data->tl_state.code_filt.prev_error = 0.f;
    }

    lock_detect_reinit(&data->lock_detect,
                       ld->k1 * ld_int_ms,
                       ld->k2,
                       ld->lp,
                       ld->lo);
  }

  if (init || cn0_ms != old_cn0_ms) {
    tp_cn0_params_t cn0_params;
    tp_get_cn0_params(channel_info->sid, &cn0_params);

    data->cn0_est_type = cn0_params.est;

    /* Initialize C/N0 estimator and filter */
    track_cn0_init(cn0_ms,            /* C/N0 period in ms */
                   &data->cn0_est,    /* C/N0 estimator state */
                   common_data->cn0); /* Initial C/N0 value */
  }

  if (init || loop_freq != old_loop_freq) {
    lp1_filter_compute_params(&data->fll_lock_params, 0.1, loop_freq);
    lp1_filter_init(&data->fll_lock_detect, &data->fll_lock_params, 0);
  } else {
    lp1_filter_compute_params(&data->fll_lock_params, 0.1, loop_freq);
    lp1_filter_init(&data->fll_lock_detect, &data->fll_lock_params, data->fll_lock_detect.yn);
  }

  if (data->use_alias_detection) {
    u8 alias_detect_ms = data->int_ms;
    if (l->mode == TP_TM_ONE_PLUS_N ||
//        l->mode == TP_TM_ONE_PLUS_N5 ||
        l->mode == TP_TM_ONE_PLUS_N20 ||
        l->mode == TP_TM_SPLIT)
      alias_detect_ms--;

    if (use_alias_detection)
      alias_detect_reinit(&data->alias_detect, 500 / alias_detect_ms,
                          alias_detect_ms * 1e-3f);
    else
      alias_detect_init(&data->alias_detect, 500 / alias_detect_ms,
                        alias_detect_ms * 1e-3f);
  }

}

static void tracker_gps_l1ca_init(const tracker_channel_info_t *channel_info,
                                  tracker_common_data_t *common_data,
                                  tracker_data_t *tracker_data)
{
  (void)channel_info;
  gps_l1ca_tracker_data_t *data = tracker_data;
  tp_config_t init_profile;

  memset(data, 0, sizeof(gps_l1ca_tracker_data_t));
  tracker_ambiguity_unknown(channel_info->context);

  {
    /* Do tracking report to manager */
    tp_report_t report;
    report.bsync = false;
    report.carr_freq = common_data->carrier_freq;
    report.code_phase_rate = common_data->code_phase_rate;
    report.cn0 = report.cn0_raw = common_data->cn0;
    report.olock = false;
    report.plock = false;
    report.sample_count = common_data->sample_count;
    report.time_ms = 0;

    tp_tracking_start(channel_info->sid, &report, &init_profile);
  }
  tracker_gps_l1ca_update_parameters(channel_info,
                                     common_data,
                                     data,
                                     &init_profile,
                                     true);
}

static void tracker_gps_l1ca_disable(const tracker_channel_info_t *channel_info,
                                     tracker_common_data_t *common_data,
                                     tracker_data_t *tracker_data)
{
  (void)common_data;
  (void)tracker_data;
  tp_tracking_stop(channel_info->sid);
}

static u32 compute_rollover_count(const tracker_channel_info_t *channel_info,
                                  const gps_l1ca_tracker_data_t *data)
{
  u8 rollover_count;
  if (data->has_next_params) {
    tp_config_t next_params;
    tp_get_profile(channel_info->sid, &next_params, false);

    /* Mode switch: first integration interval of the next configuration */
    switch (next_params.loop_params.mode) {
    case TP_TM_SPLIT:
    case TP_TM_ONE_PLUS_N:
    case TP_TM_ONE_PLUS_N5:
    case TP_TM_ONE_PLUS_N20:
      rollover_count = 0;
      break;

    case TP_TM_IMMEDIATE:
    case TP_TM_PIPELINING:
    case TP_TM_INITIAL:
      rollover_count = next_params.loop_params.coherent_ms - 1;
      break;

    default:
      assert(false);
    }
  } else {
    /* Continuation of the current stage */
    switch (data->tracking_mode) {
    case TP_TM_SPLIT:
      rollover_count = 0;
      break;

    case TP_TM_ONE_PLUS_N:
      rollover_count = data->cycle_no == 0 ?
                       0 :
                       data->int_ms - 2;
      break;

    case TP_TM_ONE_PLUS_N5:
      {
        u8 n_bits = data->int_ms / 5;
        if (data->cycle_no == n_bits - 1) {
          /* First interval is 1ms */
          rollover_count = 0;
        } else if (data->cycle_no == n_bits) {
          /* Second interval is 4ms */
          rollover_count = 3;
        } else {
          /* Other intervals are 5ms */
          rollover_count = 4;
        }
      }
      break;

    case TP_TM_ONE_PLUS_N20:
      {
        u8 n_bits = data->int_ms / 20;
        if (data->cycle_no == n_bits - 1) {
          /* First interval is 1ms */
          rollover_count = 0;
        } else if (data->cycle_no == n_bits) {
          /* Second interval is 19ms */
          rollover_count = 18;
        } else {
          /* Other intervals are 20ms */
          rollover_count = 19;
        }
      }
      break;

    case TP_TM_IMMEDIATE:
    case TP_TM_PIPELINING:
    case TP_TM_INITIAL:
      rollover_count = data->int_ms - 1;
      break;

    default:
      assert(false);
    }
  }

  return L1CA_TRACK_MS_TO_CHIPS(rollover_count + 1);
}

static void mode_change_init(const tracker_channel_info_t *channel_info,
                             tracker_common_data_t *common_data,
                             gps_l1ca_tracker_data_t *data)
{
  /* Unused parameters */
  (void)common_data;

  /* Compute time of the currently integrated period */
  u8 next_ms = 0;
  switch (data->tracking_mode)
  {
  case TP_TM_SPLIT:
    next_ms = data->cycle_no + 2 == data->int_ms ? data->int_ms : 0;
    break;

  case TP_TM_ONE_PLUS_N:
    next_ms = data->cycle_no == 0 ? data->int_ms: 0;
    break;

  case TP_TM_ONE_PLUS_N5:
    if (data->cycle_no == 1) {
      next_ms = 0;
    } else {
      next_ms = 5;
    }
    break;

  case TP_TM_ONE_PLUS_N20:
    if (data->cycle_no == 1) {
      next_ms = 0;
    } else {
      next_ms = 20;
    }
    break;

  case TP_TM_INITIAL:
  case TP_TM_PIPELINING:
  case TP_TM_IMMEDIATE:
    next_ms = data->int_ms;
    break;

  default:
    assert(false);
  }

  if (0 != next_ms &&
      tracker_next_bit_aligned(channel_info->context, next_ms)) {

    /* When the bit sync is available and the next integration interval is the
     * last one in the bit, check if the profile switch is required. */
    if (tp_has_new_profile(channel_info->sid)) {
      /* Initiate profile change */
      data->has_next_params = true;
    }
  }
}

/**
 * Finish profile switching operation.
 *
 * Method fetches new profile parameters and reconfigures as necessary.
 *
 * \param[in]     hannel_info Tracker channel data
 * \param[in,out] common_data Common data
 * \param[in,out] data        L1 C/A data
 *
 * \return None
 */
static void mode_change_complete(const tracker_channel_info_t *channel_info,
                                 tracker_common_data_t *common_data,
                                 gps_l1ca_tracker_data_t *data)
{
  if (data->has_next_params) {
    tp_config_t next_params;

    tp_get_profile(channel_info->sid, &next_params, true);

    /* If there is a stage transition in progress, update parameters for the
     * next iteration. */
    log_info_sid(channel_info->sid,
                  "Reconfiguring tracking profile: new mode=%d, ms=%d",
                  next_params.loop_params.mode,
                  (int)next_params.loop_params.coherent_ms);

    tracker_gps_l1ca_update_parameters(channel_info,
                                       common_data,
                                       data,
                                       &next_params,
                                       false);
    /* Indicate that a mode change has occurred. */
    common_data->mode_change_count = common_data->update_count;
  }
}

/**
 * Controls TL operation steps.
 *
 * The method updates the TL step according to tracking mode.
 *
 * \param[in,out] data Tracker data
 *
 * \return None
 */
static void update_cycle_counter(gps_l1ca_tracker_data_t *data)
{
  u8 cycle_cnt; /**< Number of cycles in the current tracking mode */

  switch (data->tracking_mode) {
  case TP_TM_SPLIT:
    /* Special integration mode: each coherent integration is split into a
     * number of 1ms integrations. */
    cycle_cnt = data->int_ms;
    break;

  case TP_TM_ONE_PLUS_N:
    /* One plus N integrations.
     * Each cycle has two integrations: short (1ms) and long */
    cycle_cnt = 2;
    break;

  case TP_TM_ONE_PLUS_N5:
    /* One plus N 5ms integrations.
     * Each cycle has two integrations in the first bit, and one extra
     * integration per additional bit. */
    cycle_cnt = data->int_ms / 5 + 1;
    break;

  case TP_TM_ONE_PLUS_N20:
    /* One plus N long integrations.
     * Each cycle has two integrations in the first bit, and one extra
     * integration per additional bit. */
    cycle_cnt = data->int_ms / 20 + 1;
    break;

  case TP_TM_INITIAL:
  case TP_TM_IMMEDIATE:
  case TP_TM_PIPELINING:
    /* One cycle only. */
    cycle_cnt = 0;
    break;

  default:
    assert(false);
  }

  if (cycle_cnt > 0 && ++data->cycle_no == cycle_cnt)
    data->cycle_no = 0;
}

enum
{
  SUM_UP_NONE,
  SUM_UP_ONE,
  SUM_UP_ONE_FLIP,
  SUM_UP_FLIP
};

static void tracker_gps_l1ca_update(const tracker_channel_info_t *channel_info,
                                    tracker_common_data_t *common_data,
                                    tracker_data_t *tracker_data)
{
  gps_l1ca_tracker_data_t *data = tracker_data;

  u8 int_ms; /* Integration time for the currently reported value */
  u8 update_count_ms; /* Update counter. */
  u8 use_controller = true;
  u8 use_cn0 = true;
  u8 sum_up = SUM_UP_NONE;
  float ld_int_ms = data->int_ms;

  /* Determine, if the tracking channel EPL data shall be added or not. */
  switch (data->tracking_mode)
  {
  case TP_TM_SPLIT:
    int_ms = 1;
    sum_up = data->cycle_no > 1 ? SUM_UP_ONE : SUM_UP_NONE;
    ld_int_ms = 1;
    update_count_ms = data->int_ms;
    break;

  case TP_TM_ONE_PLUS_N:
    int_ms = data->cycle_no == 0 ? 1 : data->int_ms - 1;
    sum_up = data->cycle_no != 0 ? SUM_UP_ONE : SUM_UP_NONE;
    update_count_ms = data->int_ms;
    break;

  case TP_TM_ONE_PLUS_N5:
    use_controller = false;

    if (data->cycle_no == 0) {
      int_ms = 1;
      sum_up = SUM_UP_NONE;
    } else if (data->cycle_no == 1) {
      int_ms = 4;
      sum_up = SUM_UP_ONE;
    } else {
      int_ms = 5;
      sum_up = SUM_UP_ONE;
    }
    if (data->cycle_no == data->int_ms / 5)
      use_controller = true;
    else
      use_cn0 = false;
    update_count_ms = 5;
    break;

  case TP_TM_ONE_PLUS_N20:
    use_controller = false;
    if (data->cycle_no == 0) {
      int_ms = 1;
      sum_up = SUM_UP_NONE;
    } else if (data->cycle_no == 1) {
      int_ms = 19;
      sum_up = SUM_UP_ONE_FLIP;
    } else {
      int_ms = 20;
      sum_up = SUM_UP_FLIP;
    }
    if (data->cycle_no == data->int_ms / 20)
      use_controller = true;
    else
      use_cn0 = false;
    update_count_ms = 20;
    break;

  case TP_TM_INITIAL:
  case TP_TM_IMMEDIATE:
  case TP_TM_PIPELINING:
    int_ms = data->int_ms;
    update_count_ms = data->int_ms;
    break;

  default:
    assert(false);
  }
  /* Prompt correlations for C/N0 estimator */
  corr_t cs_now[3]; /**< Correlations from FPGA */
  corr_t cs_bit[3]; /**< Sub-bit sum */

  /* Read early ([0]), prompt ([1]) and late ([2]) correlations. */
  tracker_correlations_read(channel_info->context, cs_now,
                            &common_data->sample_count,
                            &common_data->code_phase_early,
                            &common_data->carrier_phase);

  switch (sum_up) {
  case SUM_UP_NONE:
    for (int i = 0; i < 3; ++i)
      cs_bit[i] = data->cs[i] = cs_now[i];
    break;

  case SUM_UP_ONE:
    for (int i = 0; i < 3; ++i) {
      cs_bit[i].I = data->cs[i].I += cs_now[i].I;
      cs_bit[i].Q = data->cs[i].Q += cs_now[i].Q;
    }
    break;

  case SUM_UP_ONE_FLIP:
    for (int i = 0; i < 3; ++i) {
      cs_bit[i].I = data->cs[i].I += cs_now[i].I;
      cs_bit[i].Q = data->cs[i].Q += cs_now[i].Q;
    }
    /* When using multi-bit coherent integration, invert values if needed. */
    if (data->cs[1].I < 0)
      for (int i = 0; i < 3; ++i) {
        data->cs[i].I = -data->cs[i].I;
        data->cs[i].Q = -data->cs[i].Q;
      }
    break;

  case SUM_UP_FLIP:
    /* When using multi-bit coherent integration, invert new values if needed. */
    if (cs_now[1].I > 0)
      for (int i = 0; i < 3; ++i) {
        data->cs[i].I += cs_bit[i].I = cs_now[i].I;
        data->cs[i].Q += cs_bit[i].Q = cs_now[i].Q;
      }
    else
      for (int i = 0; i < 3; ++i) {
        data->cs[i].I -= cs_bit[i].I = cs_now[i].I;
        data->cs[i].Q -= cs_bit[i].Q = cs_now[i].Q;
      }
    break;

  default:
    assert(false);
  }

  if (data->tracking_mode == TP_TM_ONE_PLUS_N20 ) {
//    log_info_sid(channel_info->sid, "Scan: %d: n=%d/%d b=%d/%d s=%d/%d",
//                 data->cycle_cnt,
//                 cs_now[1].I, cs_now[1].Q,
//                 cs_bit[1].I, cs_bit[1].Q,
//                 data->cs[1].I, data->cs[1].Q);
  }

  common_data->TOW_ms = tracker_tow_update(channel_info->context,
                                           common_data->TOW_ms,
                                           int_ms);

  if ((data->tracking_mode == TP_TM_ONE_PLUS_N && data->cycle_no == 0) ||
      (data->tracking_mode == TP_TM_ONE_PLUS_N5 && data->cycle_no == 0) ||
      (data->tracking_mode == TP_TM_ONE_PLUS_N20 && data->cycle_no == 0) ||
      (data->tracking_mode == TP_TM_SPLIT && data->cycle_no < data->int_ms - 1)) {
    /* If we're doing long integrations, alternate between short and long
     * cycles.  This is because of FPGA pipelining and latency.  The
     * loop parameters can only be updated at the end of the second
     * integration interval and waiting a whole 20ms is too long.
     */

    if (data->tracking_mode == TP_TM_SPLIT) {
      common_data->cn0 = track_cn0_update(data->cn0_est_type, 1,
                                          &data->cn0_est,
                                          cs_now[1].I, cs_now[1].Q);

      lock_detect_update(&data->lock_detect, cs_now[1].I, cs_now[1].Q, 1);
    }

    if (data->use_alias_detection)
      alias_detect_first(&data->alias_detect, cs_now[1].I, cs_now[1].Q);

    /* We may change the integration time here, but only if the next long
     * integration period reaches bit boundary */
    mode_change_complete(channel_info, common_data, data);
    mode_change_init(channel_info, common_data, data);

    tracker_retune(channel_info->context, common_data->carrier_freq,
                   common_data->code_phase_rate,
                   compute_rollover_count(channel_info, data));

    update_cycle_counter(data);
    return;
  }

  /* Continue TL: below this line we may have both partial bit or multi-bit
   * integrations.
   *
   */

  common_data->update_count += update_count_ms;

  tracker_bit_sync_update(channel_info->context, update_count_ms, cs_bit[1].I);

  /* Output I/Q correlations using SBP if enabled for this channel */
  if (data->tracking_mode != TP_TM_INITIAL) {
    tracker_correlations_send(channel_info->context, cs_bit);
  }

  /* Correlations should already be in chan->cs thanks to
   * tracking_channel_get_corrs. */
  const corr_t* cs = data->cs;
  if (use_cn0) {
    tp_cn0_params_t cn0_params;
    tp_get_cn0_params(channel_info->sid, &cn0_params);

    if (cn0_params.est != data->cn0_est_type) {
      float cn0_ms = data->int_ms; /**< C/N0 input period in ms */

      data->cn0_est_type = cn0_params.est;

      /* Initialize C/N0 estimator and filter */
      track_cn0_init(cn0_ms,            /* C/N0 period in ms */
                     &data->cn0_est,    /* C/N0 estimator state */
                     common_data->cn0); /* Initial C/N0 value */

    }

    /* Update C/N0 estimate */
    common_data->cn0 = track_cn0_update(data->cn0_est_type,
                                        data->int_ms,
                                        &data->cn0_est,
                                        cs_now[1].I, cs_now[1].Q);

    if (common_data->cn0 > cn0_params.track_cn0_drop_thres ||
        (data->tracking_ctrl == TP_CTRL_PLL2 && data->lock_detect.outp) ||
        (data->tracking_ctrl == TP_CTRL_PLL3 && data->lock_detect.outp)
        ) {
      /* When C/N0 is above a drop threshold or there is a pessimistic lock,
       * tracking shall continue.
       */
      common_data->cn0_above_drop_thres_count = common_data->update_count;
    }

    if (common_data->cn0 < cn0_params.track_cn0_use_thres) {
      /* SNR has dropped below threshold, indicate that the carrier phase
       * ambiguity is now unknown as cycle slips are likely. */
      tracker_ambiguity_unknown(channel_info->context);
      /* Update the latest time we were below the threshold. */
      common_data->cn0_below_use_thres_count = common_data->update_count;
    }
  }

  if (use_controller) {
    /* Run the loop filters. */

    /* Update PLL/FLL lock detector */
    bool last_outp = data->lock_detect.outp;
    bool outo = false, outp = false;
    if (data->tracking_ctrl == TP_CTRL_PLL2 ||
        data->tracking_ctrl == TP_CTRL_PLL3) {
      lock_detect_update(&data->lock_detect, cs_now[1].I, cs_now[1].Q, ld_int_ms);
      outo = data->lock_detect.outo;
      outp = data->lock_detect.outp;
      if (data->fll_lock_detect.yn >= 0.10) {
        outo = outp = false;
      }
    } else if (data->tracking_ctrl == TP_CTRL_FLL1 ||
               data->tracking_ctrl == TP_CTRL_FLL2) {
      /* In FLL mode, there is no phase lock. Check if FLL/DLL error is small */
      outp = false;
      outo = data->fll_lock_detect.yn < 0.1;
    }

    if (outo)
      common_data->ld_opti_locked_count = common_data->update_count;
    if (!outp)
      common_data->ld_pess_unlocked_count = common_data->update_count;
    /* Reset carrier phase ambiguity if there's doubt as to our phase lock */
    if (last_outp && !outp) {
      log_info_sid(channel_info->sid, "PLL stress");
      tracker_ambiguity_unknown(channel_info->context);
    } else if (last_outp != outp) {
      log_info_sid(channel_info->sid, "PLL pessimistic lock");
    }

    /* TODO: Make this more elegant. */
    correlation_t cs2[3];
    for (u32 i = 0; i < 3; i++) {
      cs2[i].I = cs[2-i].I;
      cs2[i].Q = cs[2-i].Q;
    }

  if (data->lock_detect.outp &&
      tracker_bit_aligned(channel_info->context))
    do_l1ca_to_l2cm_handover(common_data->sample_count,
                             channel_info->sid.sat,
                             common_data->code_phase_early,
                             common_data->carrier_freq,
                             common_data->cn0);

    if (data->has_next_params) {
      /* Transitional state: when the next interval has a different integration
       * period, the controller will give wrong correction. Due to that the
       * input parameters are scaled to stabilize tracker.
       */
      const tp_loop_params_t *lp = tp_get_next_loop_params(channel_info->sid);
      if (data->int_ms != lp->coherent_ms) {
        /* TODO utilize noise bandwidth and damping ratio */
        float k1 = (float)data->int_ms / lp->coherent_ms;
        float k2 = k1 * k1;
        for (u32 i = 0; i < 3; i++) {
          cs2[i].I *= k1;
          cs2[i].Q *= k2;
        }
      }
    }


    float dll_err = 0;
    switch (data->tracking_ctrl) {
    case TP_CTRL_PLL2:
      tl_pll2_update(&data->pll2_state, cs2);
      common_data->carrier_freq = data->pll2_state.carr_freq;
      common_data->code_phase_rate = data->pll2_state.code_freq + GPS_CA_CHIPPING_RATE;
      dll_err = tl_pll2_get_dll_error(&data->pll2_state);
      break;
    case TP_CTRL_PLL3:
      tl_pll3_update(&data->pll3_state, cs2);
      common_data->carrier_freq = data->pll3_state.carr_freq;
      common_data->code_phase_rate = data->pll3_state.code_freq + GPS_CA_CHIPPING_RATE;
      dll_err = tl_pll3_get_dll_error(&data->pll3_state);
      break;
    case TP_CTRL_FLL1:
      tl_fll1_update(&data->fll1_state, cs2);
      common_data->carrier_freq = data->fll1_state.carr_freq;
      common_data->code_phase_rate = data->fll1_state.code_freq + GPS_CA_CHIPPING_RATE;
      dll_err = tl_fll1_get_dll_error(&data->fll1_state);
      break;
    case TP_CTRL_FLL2:
      tl_fll2_update(&data->fll2_state, cs2);
      common_data->carrier_freq = data->fll2_state.carr_freq;
      common_data->code_phase_rate = data->fll2_state.code_freq + GPS_CA_CHIPPING_RATE;
      dll_err = tl_fll1_get_dll_error(&data->fll1_state);
      break;
    default:
      assert(false);
    }

    dll_err = lp1_filter_update(&data->fll_lock_detect, &data->fll_lock_params, dll_err);
//      if (fabsf(dll_err) > 0.12f && !data->lock_detect.outp) {
//        log_info_sid(channel_info->sid, "Adjusting code error=%f", dll_err * 1540);
//        switch (data->tracking_ctrl) {
//        case TP_CTRL_PLL:
//          tl_pll_state_adjust(&data->pll_state, dll_err * 1540);
//          break;
//        case TP_CTRL_FLL:
//          tl_fll_state_adjust(&data->fll_state, dll_err * 1540);
//          break;
//        }
//      }
//      log_info_sid(channel_info->sid, "carr=%f %f code=%f",
//                   common_data->carrier_freq, data->tl_state_ref.carr_freq,
//                   // data->tl_state.freq_prev,

    /* Attempt alias detection if we have pessimistic phase lock detect, OR
       (optimistic phase lock detect AND are in second-stage tracking) */
    if (data->use_alias_detection &&
        (data->tracking_mode != TP_TM_INITIAL && data->lock_detect.outo)) {

      /* Last period integration time */
      u8 alias_ms = update_count_ms;
      bool alias_first = false;
      bool alias_second = false;

      switch (data->tracking_mode)
      {
      case TP_TM_ONE_PLUS_N:
        if (data->int_ms == 10 || data->int_ms == 5) {
          alias_first = true;
          alias_second = true;
          if (data->cycle_no == 1)
            alias_ms--;
        }
        break;

      case TP_TM_ONE_PLUS_N5:
        alias_first = true;
        alias_second = true;
        if (data->cycle_no == 1)
          alias_ms--;

        break;

      default:
        break;
      }

      if (alias_second) {
        float I = (cs_now[1].I - data->alias_detect.first_I) / alias_ms;
        float Q = (cs_now[1].Q - data->alias_detect.first_Q) / alias_ms;
        float err = alias_detect_second(&data->alias_detect, I, Q);
        if (fabs(err) > (250 / alias_ms)) {
          if (data->lock_detect.outp) {
            log_warn_sid(channel_info->sid, "False phase lock detected: %f, %d", err, alias_ms);
          } else {
            log_warn_sid(channel_info->sid, "False optimistic lock detected: %f, %d", err, alias_ms);
          }

          tracker_ambiguity_unknown(channel_info->context);
          /* Indicate that a mode change has occurred. */
          common_data->mode_change_count = common_data->update_count;

          switch (data->tracking_ctrl) {
          case TP_CTRL_PLL2:
            tl_pll2_adjust(&data->pll2_state, err);
            break;
          case TP_CTRL_PLL3:
            tl_pll3_adjust(&data->pll3_state, err);
            break;
          case TP_CTRL_FLL1:
            tl_fll1_adjust(&data->fll1_state, err);
            break;
          case TP_CTRL_FLL2:
            tl_fll2_adjust(&data->fll2_state, err);
            break;
          default:
            assert(false);
          }
        }

        if (alias_first)
          alias_detect_first(&data->alias_detect, cs_now[1].I, cs_now[1].Q);
      }
    }


    {
      /* Do tracking report to manager */
      tp_report_t report;
      report.bsync = tracker_has_bit_sync(channel_info->context);
      report.carr_freq = common_data->carrier_freq;
      report.code_phase_rate = common_data->code_phase_rate;
      report.cn0_raw = common_data->cn0;
      report.cn0 = common_data->cn0;
      report.olock = data->lock_detect.outo;
      report.plock = data->lock_detect.outp;
      report.lock_i = data->lock_detect.lpfi.y;
      report.lock_q = data->lock_detect.lpfq.y;
      report.lock_f = dll_err;
      report.sample_count = common_data->sample_count;
      report.time_ms = data->int_ms;

      tp_report_data(channel_info->sid, &report);
    }
  }

  mode_change_complete(channel_info, common_data, data);
  mode_change_init(channel_info, common_data, data);

  tracker_retune(channel_info->context, common_data->carrier_freq,
                 common_data->code_phase_rate,
                 compute_rollover_count(channel_info, data));

  update_cycle_counter(data);
}

