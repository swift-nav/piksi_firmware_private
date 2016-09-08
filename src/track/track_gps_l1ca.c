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
#include <stdlib.h>
#include <assert.h>

#include "chconf.h"
#include "settings.h"
#include "signal.h"
#include "board.h"
#include "platform_signal.h"
#include "platform_track.h"
#include "chconf_board.h"

#include "track_profiles.h"
#include "track_profile_utils.h"

/* DLL error-based lock detector: when FLL mode is used, DLL-based lock
 * detector is the only reliable (but slow) way to identify the tracking is
 * lost. However on V2 there is no enough memory to use it.
 */
/* #define USE_DLL_ERROR 1 */

/* Convert milliseconds to L1C/A chips */
#define L1CA_TRACK_MS_TO_CHIPS(ms) ((ms) * GPS_L1CA_CHIPS_NUM)

#define L1CA_FLOCK_INTERVAL_MS   (5000)
#define L1CA_FLOCK_THRESHOLD_HZ  (20)
#define L1CA_FLOCK_EF_ALPHA      (0.03f)

#define L1CA_TRACK_SETTING_SECTION "l1ca_track"

/**
 * GPS L1 C/A tracker data
 */
typedef struct {
  tp_tl_state_t     tl_state;               /**< Tracking loop filter state. */
  tp_corr_state_t   corrs;                  /**< Correlations */
  track_cn0_state_t cn0_est;                /**< C/N0 estimator state. */
  alias_detect_t    alias_detect;           /**< Alias lock detector. */
  lock_detect_t     lock_detect;            /**< Phase-lock detector state. */
#if USE_DLL_ERROR
  float             fll_lock_detect;        /**< FLL lock detector */
  u16               fll_lock_counter;       /**< False lock state duration counter */
#endif /* USE_DLL_ERROR */
  u8                int_ms: 5;              /**< Current integration length. */
  u8                tracking_mode: 3;       /**< Tracking mode */
  u8                cycle_no: 5;            /**< Number of cycle inside current
                                             *   integration mode. */
  u8                use_alias_detection: 1; /**< Flag for alias detection control */
  u8                has_next_params: 1;     /**< Flag if stage transition is in
                                             *   progress */
  u8                confirmed: 1;           /**< Flag if the tracking is confirmed */
} gps_l1ca_tracker_data_t;

static bool show_unconfirmed_trackers = false;
static tracker_t gps_l1ca_trackers[NUM_GPS_L1CA_TRACKERS]
                                   PLATFORM_TRACK_DATA_TRACKER;
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
  SETTING(L1CA_TRACK_SETTING_SECTION, "show_unconfirmed",
          show_unconfirmed_trackers, TYPE_BOOL);

  for (u32 i=0; i<NUM_GPS_L1CA_TRACKERS; i++) {
    gps_l1ca_trackers[i].active = false;
    gps_l1ca_trackers[i].data = &gps_l1ca_tracker_data[i];
  }

  tracker_interface_register(&tracker_interface_list_element_gps_l1ca);
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

  u8 prev_cn0_ms = 0;
  bool prev_use_alias_detection = 0;
#if USE_DLL_ERROR
  float prev_loop_freq = 0;
#endif

  if (!init) {
    prev_cn0_ms = tp_get_cn0_ms(data->tracking_mode, data->int_ms);
    prev_use_alias_detection = data->use_alias_detection;
#if USE_DLL_ERROR
    prev_loop_freq = 1000.f / tp_get_dll_ms(data->tracking_mode, data->int_ms);
#endif
  }

  data->tracking_mode = next_params->loop_params.mode;
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
  u8 cycle_cnt = tp_get_cycle_count(l->mode, l->coherent_ms);
  data->cycle_no = cycle_cnt - 1;

  float loop_freq = 1000.f / tp_get_dll_ms(data->tracking_mode, data->int_ms);
  /**< Tracking loop frequency */
  u8 cn0_ms = tp_get_cn0_ms(data->tracking_mode, data->int_ms);
  /**< C/N0 integration time */
  u8 ld_int_ms = tp_get_ld_ms(data->tracking_mode, data->int_ms);
  /**< Lock detector integration time */
  float fll_loop_freq = 1000.f / tp_get_fll_ms(data->tracking_mode, data->int_ms);

  if (init) {
    log_debug_sid(channel_info->sid, "Initializing TL");

    tp_tl_init(&data->tl_state,
               next_params->loop_params.ctrl,
               loop_freq,
               common_data->code_phase_rate - GPS_CA_CHIPPING_RATE,
               l->code_bw, l->code_zeta, l->code_k,
               l->carr_to_code,
               common_data->carrier_freq,
               l->carr_bw, l->carr_zeta, l->carr_k,
               l->carr_fll_aid_gain, fll_loop_freq);

    lock_detect_init(&data->lock_detect,
                     ld->k1 * ld_int_ms,
                     ld->k2,
                     ld->lp,
                     ld->lo);

  } else {
    log_debug_sid(channel_info->sid, "Re-tuning TL");

    /* Recalculate filter coefficients */
    tp_tl_retune(&data->tl_state,
                 next_params->loop_params.ctrl,
                 loop_freq,
                 l->code_bw, l->code_zeta, l->code_k,
                 l->carr_to_code,
                 l->carr_bw, l->carr_zeta, l->carr_k,
                 l->carr_fll_aid_gain, fll_loop_freq);

    lock_detect_reinit(&data->lock_detect,
                       ld->k1 * ld_int_ms,
                       ld->k2,
                       ld->lp,
                       ld->lo);
  }

  if (init || cn0_ms != prev_cn0_ms) {
    tp_cn0_params_t cn0_params;
    tp_get_cn0_params(channel_info->sid, &cn0_params);

    float cn0_0;
    if (data->confirmed)
      cn0_0 = common_data->cn0;
    else
      cn0_0 = cn0_params.track_cn0_drop_thres - 2;

    /* Initialize C/N0 estimator and filter */
    track_cn0_init(channel_info->sid, /* Signal for logging */
                   cn0_ms,            /* C/N0 period in ms */
                   &data->cn0_est,    /* C/N0 estimator state */
                   cn0_0,             /* Initial C/N0 value */
                   0);                /* Flags */

    if (!data->confirmed) {
      data->cn0_est.cn0_0 = common_data->cn0;
      common_data->cn0 = -1;
    }
  }

#if USE_DLL_ERROR
  data->fll_lock_counter = 0;
  if (init || loop_freq != prev_loop_freq) {
    data->fll_lock_detect = 0;
  } else {
    data->fll_lock_detect = 0;
  }
#endif /* USE_DLL_ERROR */

  if (data->use_alias_detection) {
    u8 alias_detect_ms = tp_get_alias_ms(data->tracking_mode, data->int_ms);

    if (prev_use_alias_detection)
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

  if (show_unconfirmed_trackers)
    data->confirmed = 1;

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
  u32 result_ms = 0;
  if (data->has_next_params) {
    tp_config_t next_params;
    tp_get_profile(channel_info->sid, &next_params, false);
    result_ms = tp_get_current_cycle_duration(
        next_params.loop_params.mode, next_params.loop_params.coherent_ms, 0);
  } else {
    result_ms = tp_get_rollover_cycle_duration(data->tracking_mode,
                                               data->int_ms, data->cycle_no);
  }
  return L1CA_TRACK_MS_TO_CHIPS(result_ms);
}

static void mode_change_init(const tracker_channel_info_t *channel_info,
                             tracker_common_data_t *common_data,
                             gps_l1ca_tracker_data_t *data)
{
  /* Unused parameters */
  (void)common_data;

  if (data->has_next_params || !data->confirmed)
    /* If the mode switch has been initiated - do nothing */
    return;

  /* Compute time of the currently integrated period */
  u8 next_cycle = tp_next_cycle_counter(data->tracking_mode,
                                        data->int_ms,
                                        data->cycle_no);
  u32 cycle_flags = tp_get_cycle_flags(data->tracking_mode,
                                       data->int_ms,
                                       next_cycle);

  if (0 != (cycle_flags & TP_CFLAG_BSYNC_UPDATE)) {
    /* The switch is possible only when bit sync counter is updated: get the
     * bit update interval in ms. */
    u8 bit_ms = tp_get_bit_ms(data->tracking_mode, data->int_ms);

    if (tracker_next_bit_aligned(channel_info->context, bit_ms))
      /* When the bit sync is available and the next integration interval is the
       * last one in the bit, check if the profile switch is required. */
      if (tp_has_new_profile(channel_info->sid))
        /* Initiate profile change */
        data->has_next_params = true;
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
    log_debug_sid(channel_info->sid,
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
  data->cycle_no = tp_next_cycle_counter(data->tracking_mode,
                                         data->int_ms,
                                         data->cycle_no);
}

static void process_alias_error(const tracker_channel_info_t *channel_info,
                                tracker_common_data_t *common_data,
                                gps_l1ca_tracker_data_t *data)
{
  u8 alias_ms = tp_get_alias_ms(data->tracking_mode, data->int_ms);

  float I = data->corrs.corr_ad.I;
  float Q = data->corrs.corr_ad.Q;
  I -= data->alias_detect.first_I;
  Q -= data->alias_detect.first_Q;

  float err = alias_detect_second(&data->alias_detect, I, Q);

  if (fabs(err) > (125 / alias_ms)) {

    if (data->lock_detect.outp) {
      log_warn_sid(channel_info->sid, "False phase lock detected: %f", err);
    } else {
      log_warn_sid(channel_info->sid, "False optimistic lock detected: %f", err);
    }

    s32 c = (labs((s32)(err - 12)) / 50) * 50 + 25;
    err = err > 0 ? c : -c;

    tracker_ambiguity_unknown(channel_info->context);
    /* Indicate that a mode change has occurred. */
    common_data->mode_change_count = common_data->update_count;

    tp_tl_adjust(&data->tl_state, err);

  } else if (fabs(err) > 10) {
    log_info_sid(channel_info->sid, "Uncorrected false lock: %f, %d", err, alias_ms);
  }
}
#if USE_DLL_ERROR
static void process_dll_error(const tracker_channel_info_t *channel_info,
                              gps_l1ca_tracker_data_t *data,
                              float dll_err)
{
  /* Exponential filter for DLL/FLL error */
  data->fll_lock_detect += L1CA_FLOCK_EF_ALPHA * (dll_err - data->fll_lock_detect);

  if (tp_tl_is_pll(&data->tl_state) &&
      (data->int_ms == 5 || data->int_ms == 10 || data->int_ms == 20)) {
    s32 err_hz = (s32)(data->fll_lock_detect * 1540.f + 0.5f);
    s32 abs_err_hz = abs(err_hz);

    /* FLL-assisted DLL implementation: when the error between FLL and DLL
     * become too large, FLL frequency is adjusted. */
    if (abs_err_hz > L1CA_FLOCK_THRESHOLD_HZ)
      data->fll_lock_counter ++;
    else
      data->fll_lock_counter = 0;

    if (data->fll_lock_counter == L1CA_FLOCK_INTERVAL_MS / data->int_ms)
    {
      err_hz = ((abs_err_hz - 20) / 50) * 50 + 25;
      if (data->fll_lock_detect < 0)
        err_hz = -err_hz;

      /* Reset FLL/DLL error state */
      data->fll_lock_counter = 0;
      data->fll_lock_detect = 0;

      log_info_sid(channel_info->sid, "False lock error %" PRId32, err_hz);

      tp_tl_adjust(&data->tl_state, err_hz);
    }
  }
}
#endif /* USE_DLL_ERROR */

static void tracker_gps_l1ca_update(const tracker_channel_info_t *channel_info,
                                    tracker_common_data_t *common_data,
                                    tracker_data_t *tracker_data)
{
  gps_l1ca_tracker_data_t *data = tracker_data;
  tp_epl_corr_t cs_now; /**< Correlations from FPGA */

  /* Read correlations. */
  tracker_correlations_read(channel_info->context, cs_now.epl,
                            &common_data->sample_count,
                            &common_data->code_phase_early,
                            &common_data->carrier_phase);

  /*
   * State machine control: control is a combination of actions permitted by
   * the tracker state and flags specific for current cycle.
   */
  u32 cycle_flags = tp_get_cycle_flags(data->tracking_mode,
                                       data->int_ms,
                                       data->cycle_no);

  tp_update_correlators(cycle_flags, &cs_now, &data->corrs);

  u8 int_ms = tp_get_current_cycle_duration(data->tracking_mode,
                                            data->int_ms,
                                            data->cycle_no);
  common_data->TOW_ms = tracker_tow_update(channel_info->context,
                                           common_data->TOW_ms,
                                           int_ms);

  if (0 != (cycle_flags & TP_CFLAG_BSYNC_UPDATE)) {
    /* Update counter. */
    u8 update_count_ms = tp_get_bit_ms(data->tracking_mode, data->int_ms);
    common_data->update_count += update_count_ms;
    /* Bit sync advance / message decoding */
    tracker_bit_sync_update(channel_info->context, update_count_ms,
                            data->corrs.corr_bit);
  }

  if (0 != (cycle_flags & TP_CFLAG_CN0_USE)) {
    tp_cn0_params_t cn0_params;
    tp_get_cn0_params(channel_info->sid, &cn0_params);

    /* Update C/N0 estimate */
    float cn0 = track_cn0_update(channel_info->sid,
                                 cn0_params.est,
                                 &data->cn0_est,
                                 data->corrs.corr_cn0.I,
                                 data->corrs.corr_cn0.Q);

    if (cn0 > cn0_params.track_cn0_drop_thres ||
        (tp_tl_is_pll(&data->tl_state) && data->lock_detect.outp)
        ) {
      /* When C/N0 is above a drop threshold or there is a pessimistic lock,
       * tracking shall continue.
       */
      common_data->cn0_above_drop_thres_count = common_data->update_count;
    }

    if (cn0 > cn0_params.track_cn0_drop_thres && !data->confirmed &&
        data->lock_detect.outo && tracker_has_bit_sync(channel_info->context)) {
      data->confirmed = 1;
      cn0 = data->cn0_est.cn0_0;
      /* Re-initialize C/N0 estimator and filter */
      track_cn0_init(channel_info->sid,   /* SV signal */
                     data->cn0_est.cn0_ms,/* C/N0 period in ms */
                     &data->cn0_est,      /* C/N0 estimator state */
                     cn0,                 /* Initial C/N0 value */
                     0);                  /* Flags */
    }

    if (data->confirmed)
      common_data->cn0 = cn0;

    if (cn0 < cn0_params.track_cn0_use_thres) {
      /* SNR has dropped below threshold, indicate that the carrier phase
       * ambiguity is now unknown as cycle slips are likely. */
      tracker_ambiguity_unknown(channel_info->context);
      /* Update the latest time we were below the threshold. */
      common_data->cn0_below_use_thres_count = common_data->update_count;
    }
  }

  /* Run the loop filters. */

  /* Update PLL/FLL lock detector */
  bool last_outp = data->lock_detect.outp;
  bool outo = false, outp = false;
  if (tp_tl_is_pll(&data->tl_state)) {

    if (0 != (cycle_flags & TP_CFLAG_LD_USE))
      lock_detect_update(&data->lock_detect,
                         data->corrs.corr_ld.I,
                         data->corrs.corr_ld.Q,
                         tp_get_ld_ms(data->tracking_mode, data->int_ms));

    outo = data->lock_detect.outo;
    outp = data->lock_detect.outp;

  } else if (tp_tl_is_fll(&data->tl_state)) {
    /* In FLL mode, there is no phase lock. Check if FLL/DLL error is small */
    outp = false;
#if USE_DLL_ERROR
    outo = data->fll_lock_detect < 0.1;
#else /* USE_DLL_ERROR */
    outo = true;
#endif /* USE_DLL_ERROR */
  }

  if (outo)
    common_data->ld_opti_locked_count = common_data->update_count;
  if (!outp)
    common_data->ld_pess_unlocked_count = common_data->update_count;
  /* Reset carrier phase ambiguity if there's doubt as to our phase lock */
  if (last_outp && !outp) {
    if (data->tracking_mode != TP_TM_INITIAL)
      log_info_sid(channel_info->sid, "PLL stress");
    tracker_ambiguity_unknown(channel_info->context);
  }

  if (data->lock_detect.outp && data->confirmed &&
      0 != (cycle_flags & TP_CFLAG_BSYNC_UPDATE) &&
      tracker_bit_aligned(channel_info->context))
    do_l1ca_to_l2cm_handover(common_data->sample_count,
                             channel_info->sid.sat,
                             common_data->code_phase_early,
                             common_data->carrier_freq,
                             common_data->cn0);

  if (0 != (cycle_flags & TP_CFLAG_FLL_SECOND))
    tp_tl_fll_update_second(&data->tl_state, data->corrs.corr_fll);
  if (0 != (cycle_flags & TP_CFLAG_FLL_USE))
    tp_tl_fll_update(&data->tl_state);
  if (0 != (cycle_flags & TP_CFLAG_FLL_FIRST))
    tp_tl_fll_update_first(&data->tl_state, data->corrs.corr_fll);

  if (0 != (cycle_flags & TP_CFLAG_EPL_USE)) {

    /* Output I/Q correlations using SBP if enabled for this channel */
    if (data->tracking_mode != TP_TM_INITIAL) {
      tracker_correlations_send(channel_info->context, data->corrs.corr_epl.epl);
    }

    if (data->has_next_params) {
      /* Transitional state: when the next interval has a different integration
       * period, the controller will give wrong correction. Due to that the
       * input parameters are scaled to stabilize tracker.
       */
      const tp_loop_params_t *lp = tp_get_next_loop_params(channel_info->sid);
      if (data->int_ms != lp->coherent_ms) {
        /* TODO utilize noise bandwidth and damping ratio */
        float k2 = (float)data->int_ms / lp->coherent_ms;
        float k1 = sqrtf(k2);
        for (u32 i = 0; i < 3; i++) {
          data->corrs.corr_epl.epl[i].I *= k1;
          data->corrs.corr_epl.epl[i].Q *= k2;
        }
      }
    }

    float carr_freq = 0;
    float code_freq = 0;

    tp_tl_update(&data->tl_state, &data->corrs.corr_epl);
    tp_tl_get_rates(&data->tl_state, &carr_freq, &code_freq);

    common_data->carrier_freq = carr_freq;
    common_data->code_phase_rate = code_freq + GPS_CA_CHIPPING_RATE;

#if USE_DLL_ERROR
    float dll_err = tp_tl_get_dll_error(&data->tl_state);
    /* Check DLL errors if available */
    process_dll_error(channel_info, data, dll_err);
#endif /* USE_DLL_ERROR */

    {
      /* Do tracking report to manager */
      tp_report_t report;
      report.bsync = tracker_has_bit_sync(channel_info->context);
      report.carr_freq = common_data->carrier_freq;
      report.code_phase_rate = common_data->code_phase_rate;
      report.cn0_raw = common_data->cn0;
      report.cn0 = data->confirmed ? common_data->cn0 : data->cn0_est.cn0_0;
      report.olock = data->lock_detect.outo;
      report.plock = data->lock_detect.outp;
      report.lock_i = data->lock_detect.lpfi.y;
      report.lock_q = data->lock_detect.lpfq.y;
#if USE_DLL_ERROR
      report.lock_f = data->fll_lock_detect * 1540.f;
#else /* USE_DLL_ERROR */
      report.lock_f = -1;
#endif /* USE_DLL_ERROR */
      report.sample_count = common_data->sample_count;
      report.time_ms = tp_get_dll_ms(data->tracking_mode, data->int_ms);

      tp_report_data(channel_info->sid, &report);
    }
  }

  /* Attempt alias detection if we have pessimistic phase lock detect, OR
     (optimistic phase lock detect AND are in second-stage tracking) */
  if (data->use_alias_detection && data->lock_detect.outo &&
      0 != (cycle_flags & TP_CFLAG_ALIAS_SECOND))
      process_alias_error(channel_info, common_data, data);

  if (0 != (cycle_flags & TP_CFLAG_ALIAS_FIRST))
    alias_detect_first(&data->alias_detect,
                       data->corrs.corr_ad.I,
                       data->corrs.corr_ad.Q);

  mode_change_complete(channel_info, common_data, data);
  mode_change_init(channel_info, common_data, data);

  tracker_retune(channel_info->context, common_data->carrier_freq,
                 common_data->code_phase_rate,
                 compute_rollover_count(channel_info, data));

  update_cycle_counter(data);
}

