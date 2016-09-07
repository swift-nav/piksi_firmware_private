/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Adel Mamin <adel.mamin@exafore.com>
 * Contact: Valeri Atamaniouk <valeri.atamaniouk@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <libswiftnav/track.h>
#include <libswiftnav/constants.h>

#include <track.h>

#include "track_profiles.h"
#include "track_profile_utils.h"

#include <math.h>
#include <string.h>
#include <assert.h>

/** Exponential filter coefficient for FLL/DLL false lock detector. */
#define TP_TRACKER_FLL_LOCK_ALPHA      (0.03f)
/** False lock detector filter interval in ms. */
#define TP_TRACKER_ALIAS_DURATION_MS   (1000)
/** Initial C/N0 for confirmation [dB/Hz] */
#define TP_TRACKER_CN0_CONFIRM_DELTA   (2.f)

/**
 * Computes number of chips in the integration interval
 *
 * In most cases the number of chips is the result of multiplication of a
 * chip rate to interval duration.
 *
 * \param[in] sid         GNSS signal identifier.
 * \param[in] ms          Interval duration in ms.
 *
 * \return Computed number of chips.
 */
static u32 tp_convert_ms_to_chips(gnss_signal_t sid, u8 ms)
{
  u32 chip_rate = 0;

  /* First, select the appropriate chip rate in chips/ms, integer equivalent of
   * the expression:
   * chip_rate = (u32)code_to_chip_rate(sid.code) / 1000;
   */
  switch (sid_to_constellation(sid)) {
  case CONSTELLATION_GPS:
  case CONSTELLATION_SBAS:
    chip_rate = GPS_L1CA_CHIPS_NUM;
    break;

  case CONSTELLATION_GLO:
    chip_rate = GLO_CA_CHIPS_NUM;
    break;

  default:
    assert(!"Unsupported constellation");
  }

  return ms * chip_rate;
}

/**
 * Update tracker parameters when initializing or changing tracking mode.
 *
 * \param[in]     channel_info Tracking channel information.
 * \param[in,out] common_data  Common tracking channel data.
 * \param[in,out] data         Generic tracker data.
 * \param[in]     next_params  Tracking configuration.
 * \param[in]     init         Flag to indicate if the call to initialize or
 *                             to update.
 *
 * \return None
 */
void tp_tracker_update_parameters(const tracker_channel_info_t *channel_info,
                                  tracker_common_data_t *common_data,
                                  tp_tracker_data_t *data,
                                  const tp_config_t *next_params,
                                  bool init)
{
  const tp_loop_params_t *l = &next_params->loop_params;
  const tp_lock_detect_params_t *ld = &next_params->lock_detect_params;

  u8 prev_cn0_ms = 0;
  bool prev_use_alias_detection = 0;

  if (!init) {
    prev_cn0_ms = tp_get_cn0_ms(data->tracking_mode, data->mode_ms);
    prev_use_alias_detection = data->use_alias_detection;
  }

  data->tracking_mode = next_params->loop_params.mode;
  data->use_alias_detection = next_params->use_alias_detection;

  data->mode_ms = next_params->loop_params.mode_ms;
  data->has_next_params = false;

  /* Set the step number for mode switch. Current step is one step behind bit
   * edge */
  u8 cycle_cnt = tp_get_cycle_count(l->mode, l->mode_ms);
  data->cycle_no = cycle_cnt - 1;
  float carr_to_code = code_to_carr_to_code(channel_info->sid.code);

  float loop_freq = 1000.f / tp_get_dll_ms(data->tracking_mode, data->mode_ms);
  /**< Tracking loop frequency */
  u8 cn0_ms = tp_get_cn0_ms(data->tracking_mode, data->mode_ms);
  /**< C/N0 integration time */
  u8 ld_int_ms = tp_get_ld_ms(data->tracking_mode, data->mode_ms);
  /**< Lock detector integration time */
  float fll_loop_freq = 1000.f / tp_get_fll_ms(data->tracking_mode, data->mode_ms);

  if (init) {
    log_debug_sid(channel_info->sid, "Initializing TL");


    tp_tl_init(&data->tl_state,
               next_params->loop_params.ctrl,
               loop_freq,
               common_data->code_phase_rate - GPS_CA_CHIPPING_RATE,
               l->code_bw, l->code_zeta, l->code_k,
               carr_to_code,
               common_data->carrier_freq,
               0, /* acceleration*/
               l->carr_bw, l->carr_zeta, l->carr_k,
               l->fll_bw, fll_loop_freq);

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
                 carr_to_code,
                 l->carr_bw, l->carr_zeta, l->carr_k,
                 l->fll_bw, fll_loop_freq);

    lock_detect_reinit(&data->lock_detect,
                       ld->k1 * ld_int_ms,
                       ld->k2,
                       ld->lp,
                       ld->lo);
  }

  if (init || cn0_ms != prev_cn0_ms) {
    tp_cn0_params_t cn0_params;
    tp_get_cn0_params(channel_info->sid, &cn0_params);

    float cn0_t;
    float cn0_0;

    if (data->confirmed) {
      cn0_t = cn0_0 = common_data->cn0;
    } else {
      /* When confirmation is required, set C/N0 below drop threshold and
       * check that is actually grows to correct range */
      cn0_0 = cn0_params.track_cn0_drop_thres - TP_TRACKER_CN0_CONFIRM_DELTA;
      cn0_t = init ? common_data->cn0 : data->cn0_est.cn0_0;
    }

    /* Initialize C/N0 estimator and filter */
    track_cn0_init(channel_info->sid, /* Signal for logging */
                   cn0_ms,            /* C/N0 period in ms */
                   &data->cn0_est,    /* C/N0 estimator state */
                   cn0_0,             /* Initial C/N0 value */
                   0);                /* Flags */

    if (!data->confirmed) {
      data->cn0_est.cn0_0 = cn0_t;
      common_data->cn0 = -1;
    }
    log_debug_sid(channel_info->sid, "CN0 update: CD=%f EST=%f CN0_0=%f",
                  common_data->cn0,
                  cn0_0,
                  cn0_t);
  }

  data->fll_lock_counter = 0;
  data->fll_lock_detect = 0;

  if (data->use_alias_detection) {
    u8 alias_detect_ms = tp_get_alias_ms(data->tracking_mode, data->mode_ms);

    if (prev_use_alias_detection) {
      alias_detect_reinit(&data->alias_detect,
                          TP_TRACKER_ALIAS_DURATION_MS / alias_detect_ms,
                          alias_detect_ms * 1e-3f);
    } else {
      alias_detect_init(&data->alias_detect,
                        TP_TRACKER_ALIAS_DURATION_MS / alias_detect_ms,
                        alias_detect_ms * 1e-3f);
    }
  }
}

/**
 * Initializes tracker data.
 *
 * The method initializes tracker parameter and updates tracker context.
 *
 * \param[in]     channel_info Tracking channel information.
 * \param[in,out] common_data  Common tracking channel data.
 * \param[out]    data         Generic tracker data to initialize.
 * \param[in]     config       Configuration parameters.
 *
 * \return None
 */
void tp_tracker_init(const tracker_channel_info_t *channel_info,
                     tracker_common_data_t *common_data,
                     tp_tracker_data_t *data,
                     const tp_tracker_config_t *config)
{
  tp_config_t init_profile;

  memset(data, 0, sizeof(*data));
  tracker_ambiguity_unknown(channel_info->context);

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

  if (config->show_unconfirmed_trackers) {
    data->confirmed = 1;
  }

  tp_tracker_update_parameters(channel_info,
                               common_data,
                               data,
                               &init_profile,
                               true);
}

/**
 * Releases tracker data.
 *
 * The method releases tracker state.
 *
 * \param[in]     channel_info Tracking channel information.
 *
 * \return None
 */
void tp_tracker_disable(const tracker_channel_info_t *channel_info)
{
  tp_tracking_stop(channel_info->sid);
}

/**
 * Computes number of chips for the coming integration period.
 *
 * The method computes forthcoming interval duration and flags and uses them
 * to compute the number of chips in the interval.
 *
 * The method checks if the tracking mode is persisted. In this case the current
 * tracking mode parameters are used. Otherwise, the new tracking parameters
 * are obtained and used for computation.
 *
 * \param[in]     channel_info Tracking channel information.
 * \param[in]     data         Generic tracker data.
 *
 * \return Computed number of chips.
 */
u32 tp_tracker_compute_rollover_count(const tracker_channel_info_t *channel_info,
                                      const tp_tracker_data_t *data)
{
  u32 result_ms = 0;
  if (data->has_next_params) {
    tp_config_t next_params;
    tp_get_profile(channel_info->sid, &next_params, false);
    result_ms = tp_get_current_cycle_duration(next_params.loop_params.mode,
                                              next_params.loop_params.mode_ms,
                                              0);
  } else {
    result_ms = tp_get_rollover_cycle_duration(data->tracking_mode,
                                               data->mode_ms,
                                               data->cycle_no);
  }
  return tp_convert_ms_to_chips(channel_info->sid, result_ms);
}

/**
 * Initialize profile switching operation.
 *
 * Method checks if the profile switching is possible and required. The profile
 * switching is possible only when the bit boundary is reached, thus the check
 * is performed when the interval processed by FPGA is the last bit interval,
 * or, in other words, the next cycle has a bit sync flag and closes the bit.
 *
 * \param[in]     channel_info Tracking channel information.
 * \param[in,out] data         Generic tracker data.
 *
 * \return None
 */
static void mode_change_init(const tracker_channel_info_t *channel_info,
                             tp_tracker_data_t *data)
{
  if (data->has_next_params || !data->confirmed) {
    /* If the mode switch has been initiated - do nothing */
    return;
  }

  /* Compute time of the currently integrated period */
  u8 next_cycle = tp_next_cycle_counter(data->tracking_mode,
                                        data->mode_ms,
                                        data->cycle_no);
  u32 next_cycle_flags = tp_get_cycle_flags(data->tracking_mode,
                                            data->mode_ms,
                                            next_cycle);

  if (0 != (next_cycle_flags & TP_CFLAG_BSYNC_UPDATE)) {
    /* The switch is possible only when bit sync counter is updated: get the
     * bit update interval in ms. */
    u8 bit_ms = tp_get_bit_ms(data->tracking_mode, data->mode_ms);

    if (tracker_next_bit_aligned(channel_info->context, bit_ms)) {
      /* When the bit sync is available and the next integration interval is the
       * last one in the bit, check if the profile switch is required. */
      if (tp_has_new_profile(channel_info->sid)) {
        /* Initiate profile change */
        data->has_next_params = true;
      }
    }
  }
}

/**
 * Finish profile switching operation.
 *
 * Method fetches new profile parameters and reconfigures as necessary.
 *
 * \param[in]     channel_info Tracking channel information.
 * \param[in,out] common_data  Common tracking channel data.
 * \param[in,out] data         Generic tracker data.
 *
 * \return None
 */
static void mode_change_complete(const tracker_channel_info_t *channel_info,
                                 tracker_common_data_t *common_data,
                                 tp_tracker_data_t *data)
{
  if (data->has_next_params) {
    tp_config_t next_params;

    tp_get_profile(channel_info->sid, &next_params, true);

    /* If there is a stage transition in progress, update parameters for the
     * next iteration. */
    log_debug_sid(channel_info->sid,
                  "Reconfiguring tracking profile: new mode=%d, ms=%d",
                  next_params.loop_params.mode,
                  (int)next_params.loop_params.mode_ms);

    tp_tracker_update_parameters(channel_info,
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
void tp_tracker_update_cycle_counter(tp_tracker_data_t *data)
{
  data->cycle_no = tp_next_cycle_counter(data->tracking_mode,
                                         data->mode_ms,
                                         data->cycle_no);
}

/**
 * Runs alias detection logic
 *
 * \param[in,out] alias_detect Alias detector's state
 * \param[in]     I            the value of in-phase arm of the correlator
 * \param[in]     Q            the value of quadrature arm of the correlator
 *
 * \return The frequency error of PLL [Hz]
 */
static s32 tp_tl_detect_alias(alias_detect_t *alias_detect, float I, float Q)
{
  float err = alias_detect_second(alias_detect, I, Q);
  s32 abs_err = (s32)(fabsf(err) + .5f);
  s32 correction = 0;

  /* The expected frequency errors are +-(25 + N * 50) Hz
     For more details, see:
     https://swiftnav.hackpad.com/Alias-PLL-lock-detector-in-L2C-4fWUJWUNnOE */
  if (abs_err > 12) {
    correction = 50 * (abs_err / 50) + 25;
  }

  return err >= 0 ? correction : -correction;
}

/**
 * Second stage of false lock detection.
 *
 * Detect frequency error and update tracker state as appropriate.
 *
 * \param[in]     channel_info Tracking channel information.
 * \param[in,out] common_data  Common tracking channel data.
 * \param[in,out] data         Generic tracker data.
 *
 * \return None
 */
static void process_alias_error(const tracker_channel_info_t *channel_info,
                                tracker_common_data_t *common_data,
                                tp_tracker_data_t *data)
{
  float I = data->corrs.corr_ad.I - data->alias_detect.first_I;
  float Q = data->corrs.corr_ad.Q - data->alias_detect.first_Q;

  s32 err = tp_tl_detect_alias(&data->alias_detect, I, Q);

  if (0 != err) {

    if (data->lock_detect.outp) {
      log_warn_sid(channel_info->sid, "False phase lock detected: %f", err);
    } else {
      log_debug_sid(channel_info->sid, "False optimistic lock detected: %f", err);
    }

    tracker_ambiguity_unknown(channel_info->context);
    /* Indicate that a mode change has occurred. */
    common_data->mode_change_count = common_data->update_count;

    tp_tl_adjust(&data->tl_state, err);

  }
}

/**
 * Updates tracker correlators.
 *
 * Updates tracker correlators with data read from FPGA according to current
 * cycle flags.
 *
 * \param[in]     channel_info Tracking channel information.
 * \param[in,out] common_data  Common tracking channel data.
 * \param[in,out] data         Generic tracker data.
 * \param[in]     cycle_flags  Current cycle flags.
 */
void tp_tracker_update_correlators(const tracker_channel_info_t *channel_info,
                                   tracker_common_data_t *common_data,
                                   tp_tracker_data_t *data,
                                   u32 cycle_flags)
{
  tp_epl_corr_t cs_now;    /**< Correlations from FPGA */
  u32    sample_count;     /**< Sample count from FPGA */
  double code_phase_early; /**< Code phase from FPGA */
  double carrier_phase;    /**< Carrier phase from FPGA */
  u8     int_ms = 0;       /**< Current cycle duration in ms */

  /* Read correlations. */
  tracker_correlations_read(channel_info->context, cs_now.epl,
                            &sample_count,
                            &code_phase_early,
                            &carrier_phase);

  tp_update_correlators(cycle_flags, &cs_now, &data->corrs);

  /* Current cycle duration */
  int_ms = tp_get_current_cycle_duration(data->tracking_mode,
                                         data->mode_ms,
                                         data->cycle_no);

  common_data->sample_count = sample_count;
  common_data->code_phase_early = code_phase_early;
  common_data->carrier_phase = carrier_phase;

  /* ToW update:
   * ToW along with carrier and code phases and sample number shall be updated
   * in sync.
   *
   * While the tracker can run in fractions of milliseconds, the current
   * design doesn't permit ToW updates with sub-millisecond values.
   */

  /* Channel run time. */
  common_data->update_count += int_ms;
  /* ToW update counter. */
  common_data->TOW_ms = tracker_tow_update(channel_info->context,
                                           common_data->TOW_ms,
                                           int_ms);
}

/**
 * Updates bit synchronization and decodes message
 *
 * \param[in]     channel_info Tracking channel information.
 * \param[in,out] data         Generic tracker data.
 * \param[in]     cycle_flags  Current cycle flags.
 *
 * \return None
 */
void tp_tracker_update_bsync(const tracker_channel_info_t *channel_info,
                             tp_tracker_data_t *data,
                             u32 cycle_flags)
{
  if (0 != (cycle_flags & TP_CFLAG_BSYNC_UPDATE)) {
    /* Bit sync / data decoding update counter. */
    u8 update_count_ms = tp_get_bit_ms(data->tracking_mode, data->mode_ms);
    /* Bit sync advance / message decoding */
    tracker_bit_sync_update(channel_info->context, update_count_ms,
                            data->corrs.corr_bit);
  }
}

/**
 * Updates C/N0 estimators.
 *
 * \param[in]     channel_info Tracking channel information.
 * \param[in,out] common_data  Common tracking channel data.
 * \param[in,out] data         Generic tracker data.
 * \param[in]     cycle_flags  Current cycle flags.
 *
 * \return None
 */
void tp_tracker_update_cn0(const tracker_channel_info_t *channel_info,
                           tracker_common_data_t *common_data,
                           tp_tracker_data_t *data,
                           u32 cycle_flags)
{
  float cn0 = data->cn0_est.filter.yn;
  tp_cn0_params_t cn0_params;
  tp_get_cn0_params(channel_info->sid, &cn0_params);

  if (0 != (cycle_flags & TP_CFLAG_CN0_USE)) {
    /* Update C/N0 estimate */
    cn0 = track_cn0_update(channel_info->sid,
                           cn0_params.est,
                           &data->cn0_est,
                           data->corrs.corr_cn0.I,
                           data->corrs.corr_cn0.Q);
  }

  if (cn0 > cn0_params.track_cn0_drop_thres ||
      (tp_tl_is_pll(&data->tl_state) && data->lock_detect.outp)) {
    /* When C/N0 is above a drop threshold or there is a pessimistic lock,
     * tracking shall continue.
     */
    common_data->cn0_above_drop_thres_count = common_data->update_count;
  }

  if (cn0 > cn0_params.track_cn0_drop_thres &&
      !data->confirmed &&
      data->lock_detect.outo && tracker_has_bit_sync(channel_info->context)) {
    data->confirmed = 1;
    log_debug_sid(channel_info->sid, "CONFIRMED from %f to %d",
                  cn0, data->cn0_est.cn0_0);

    cn0 = data->cn0_est.cn0_0;
    /* Re-initialize C/N0 estimator and filter */
    track_cn0_init(channel_info->sid,   /* SV signal */
                   data->cn0_est.cn0_ms,/* C/N0 period in ms */
                   &data->cn0_est,      /* C/N0 estimator state */
                   cn0,                 /* Initial C/N0 value */
                   0);                  /* Flags */
  }

  if (data->confirmed) {
    common_data->cn0 = cn0;
  }

  if (cn0 < cn0_params.track_cn0_use_thres) {
    /* SNR has dropped below threshold, indicate that the carrier phase
     * ambiguity is now unknown as cycle slips are likely. */
    tracker_ambiguity_unknown(channel_info->context);
    /* Update the latest time we were below the threshold. */
    common_data->cn0_below_use_thres_count = common_data->update_count;
  }
}

/**
 * Updates PLL and FLL lock registers.
 *
 * Updates PLL optimistic and pessimistic registers. In addition, checks if
 * FLL lock is present in FLL tracking mode.
 *
 * \param[in]     channel_info Tracking channel information.
 * \param[in,out] common_data  Common tracking channel data.
 * \param[in,out] data         Generic tracker data.
 * \param[in]     cycle_flags  Current cycle flags.
 *
 * \return None
 */
void tp_tracker_update_locks(const tracker_channel_info_t *channel_info,
                             tracker_common_data_t *common_data,
                             tp_tracker_data_t *data,
                             u32 cycle_flags)
{
  if (0 != (cycle_flags & TP_CFLAG_LD_USE)) {
    /* Update PLL/FLL lock detector */
    bool last_outp = data->lock_detect.outp;
    bool outo = false, outp = false;
    if (tp_tl_is_pll(&data->tl_state)) {
      lock_detect_update(&data->lock_detect,
                         data->corrs.corr_ld.I,
                         data->corrs.corr_ld.Q,
                         tp_get_ld_ms(data->tracking_mode, data->mode_ms));

      outo = data->lock_detect.outo;
      outp = data->lock_detect.outp;
      data->fll_lock_detect = 0;

    } else if (tp_tl_is_fll(&data->tl_state)) {
      /* In FLL mode, there is no phase lock. Check if FLL/DLL error is small */

      float dll_err = tp_tl_get_dll_error(&data->tl_state);
      /* Exponential filter for DLL/FLL error */
      data->fll_lock_detect += TP_TRACKER_FLL_LOCK_ALPHA *
                               (dll_err - data->fll_lock_detect);

      /* In FLL mode simulate optimistic lock only, no pessimistic lock. */
      data->lock_detect.outp = outp = false;
      data->lock_detect.outo = outo = data->fll_lock_detect < 0.1;
    }

    if (outo) {
      common_data->ld_opti_locked_count = common_data->update_count;
    }
    if (!outp) {
      common_data->ld_pess_unlocked_count = common_data->update_count;
    }

    if (last_outp && !outp && data->tracking_mode != TP_TM_INITIAL) {
      log_info_sid(channel_info->sid, "PLL stress");
    }
    /* Reset carrier phase ambiguity if there's doubt as to our phase lock */
    if (!outp) {
      tracker_ambiguity_unknown(channel_info->context);
    }
  }
}

/**
 * Handle FLL tracker operations
 *
 * The method performs FLL tracking or FLL tracking assistance for PLL.
 *
 * \param[in,out] data         Generic tracker data.
 * \param[in]     cycle_flags  Current cycle flags.
 *
 * \return None
 */
void tp_tracker_update_fll(tp_tracker_data_t *data, u32 cycle_flags)
{
  if (0 != (cycle_flags & TP_CFLAG_FLL_SECOND)) {
    tp_tl_fll_update_second(&data->tl_state, data->corrs.corr_fll);
  }

  if (0 != (cycle_flags & TP_CFLAG_FLL_USE)) {
    tp_tl_fll_update(&data->tl_state);
  }

  if (0 != (cycle_flags & TP_CFLAG_FLL_FIRST)) {
    tp_tl_fll_update_first(&data->tl_state, data->corrs.corr_fll);
  }
}

/**
 * Runs PLL and DLL controller updates.
 *
 * This method updates PLL and DLL loops and additionally checks for DLL errors
 * and report data to profile managements.
 *
 * \param[in]     channel_info Tracking channel information.
 * \param[in,out] common_data  Common tracking channel data.
 * \param[in,out] data         Generic tracker data.
 * \param[in]     cycle_flags  Current cycle flags.
 *
 * \return None
 */
void tp_tracker_update_pll_dll(const tracker_channel_info_t *channel_info,
                               tracker_common_data_t *common_data,
                               tp_tracker_data_t *data,
                               u32 cycle_flags)
{
  if (0 != (cycle_flags & TP_CFLAG_EPL_USE)) {

    /* Output I/Q correlations using SBP if enabled for this channel */
    if (data->tracking_mode != TP_TM_INITIAL)
      tracker_correlations_send(channel_info->context, data->corrs.corr_epl.epl);

    if (data->has_next_params) {
      /* Transitional state: when the next interval has a different integration
       * period, the controller will give wrong correction. Due to that the
       * input parameters are scaled to stabilize tracker.
       */
      u8 new_dll_ms = tp_get_next_loop_params_ms(channel_info->sid);
      u8 old_dll_ms = tp_get_dll_ms(data->tracking_mode, data->mode_ms);

      if (old_dll_ms != new_dll_ms) {
        /* TODO utilize noise bandwidth and damping ratio */
        float k2 = (float)old_dll_ms / new_dll_ms;
        float k1 = sqrtf(k2);
        for (u32 i = 0; i < 3; i++) {
          data->corrs.corr_epl.epl[i].I *= k1;
          data->corrs.corr_epl.epl[i].Q *= k2;
        }
      }
    }

    tl_rates_t rates = {0};

    tp_tl_update(&data->tl_state, &data->corrs.corr_epl);
    tp_tl_get_rates(&data->tl_state, &rates);

    common_data->carrier_freq = rates.carr_freq;
    common_data->code_phase_rate = rates.code_freq + GPS_CA_CHIPPING_RATE;

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
    float code_rate = code_to_carr_to_code(channel_info->sid.code);
    report.lock_f = data->fll_lock_detect * code_rate;
    report.sample_count = common_data->sample_count;
    report.time_ms = tp_get_dll_ms(data->tracking_mode, data->mode_ms);

    tp_report_data(channel_info->sid, &report);
  }
}

/**
 * Runs false lock detection logic for PLL.
 *
 * \param[in]     channel_info Tracking channel information.
 * \param[in,out] common_data  Common tracking channel data.
 * \param[in,out] data         Generic tracker data.
 * \param[in]     cycle_flags  Current cycle flags.
 *
 * \return None
 */
void tp_tracker_update_alias(const tracker_channel_info_t *channel_info,
                             tracker_common_data_t *common_data,
                             tp_tracker_data_t *data,
                             u32 cycle_flags)
{
  bool do_first = 0 != (cycle_flags & TP_CFLAG_ALIAS_FIRST);

  /* Attempt alias detection if we have pessimistic phase lock detect, OR
     (optimistic phase lock detect AND are in second-stage tracking) */
  if (0 != (cycle_flags & TP_CFLAG_ALIAS_SECOND)) {
    if (data->use_alias_detection && data->lock_detect.outo) {
      process_alias_error(channel_info, common_data, data);
    } else {
      /* If second stage is not enabled, make the first one */
      do_first = true;
    }
  }

  if (do_first) {
    alias_detect_first(&data->alias_detect,
                       data->corrs.corr_ad.I,
                       data->corrs.corr_ad.Q);
  }
}

/**
 * Mode switching control.
 *
 * \param[in]     channel_info Tracking channel information.
 * \param[in,out] common_data  Common tracking channel data.
 * \param[in,out] data         Generic tracker data.
 *
 * \return None
 */
void tp_tracker_update_mode(const tracker_channel_info_t *channel_info,
                            tracker_common_data_t *common_data,
                            tp_tracker_data_t *data)
{
  mode_change_complete(channel_info, common_data, data);
  mode_change_init(channel_info, data);
}

/**
 * Default tracking loop.
 *
 * \param[in]     channel_info Tracking channel information.
 * \param[in,out] common_data  Common tracking channel data.
 * \param[in,out] data         Generic tracker data.
 *
 * \return Flags, used in the current tracking cycle.
 */
u32 tp_tracker_update(const tracker_channel_info_t *channel_info,
                      tracker_common_data_t *common_data,
                      tp_tracker_data_t *data)
{
  /*
   * State machine control: control is a combination of actions permitted by
   * the tracker state and flags specific for current cycle.
   */
  u32 cflags = tp_get_cycle_flags(data->tracking_mode,
                                  data->mode_ms,
                                  data->cycle_no);

  tp_tracker_update_correlators(channel_info, common_data, data, cflags);
  tp_tracker_update_bsync(channel_info, data, cflags);
  tp_tracker_update_cn0(channel_info, common_data, data, cflags);
  tp_tracker_update_locks(channel_info, common_data, data, cflags);
  tp_tracker_update_fll(data, cflags);
  tp_tracker_update_pll_dll(channel_info, common_data, data, cflags);
  tp_tracker_update_alias(channel_info, common_data, data, cflags);
  tp_tracker_update_mode(channel_info, common_data, data);

  tracker_retune(channel_info->context, common_data->carrier_freq,
                 common_data->code_phase_rate,
                 tp_tracker_compute_rollover_count(channel_info, data));

  tp_tracker_update_cycle_counter(data);

  return cflags;
}
