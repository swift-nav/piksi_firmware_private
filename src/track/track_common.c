/*
 * Copyright (C) 2016 - 2017 Swift Navigation Inc.
 * Contact: Adel Mamin <adel.mamin@exafore.com>
 * Contact: Michele Bavaro <michele@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <libswiftnav/constants.h>
#include <libswiftnav/gnss_time.h>

#include "board/nap/track_channel.h"
#include "lock_detector/lock_detector.h"
#include "signal_db/signal_db.h"
#include "timing/timing.h"
#include "track_api.h"
#include "track_cfg.h"
#include "track_common.h"
#include "track_flags.h"
#include "track_sid_db.h"
#include "track_utils.h"

#include <assert.h>
#include <inttypes.h>
#include <math.h>
#include <string.h>

/**
 * Computes number of chips in the integration interval
 *
 * In most cases the number of chips is the result of multiplication of a
 * chip rate to interval duration.
 *
 * \param[in] mesid       ME signal identifier.
 * \param[in] ms          Interval duration in ms.
 * \param[in] code_phase  Current code phase in chips.
 * \param[in] plock       Flag indicating pessimistic lock.
 *
 * \return Computed number of chips.
 */
static u32 tp_convert_ms_to_chips(me_gnss_signal_t mesid,
                                  u8 ms,
                                  double code_phase,
                                  bool plock) {
  /* First, select the appropriate chip rate in chips/ms. */
  u32 chip_rate = (u32)code_to_chip_rate(mesid.code) / 1000;

  /* Round the current code_phase towards nearest integer. */
  u32 current_chip = round(code_phase);

  /* Take modulo of the code phase. Nominally this should be close to zero,
   * or close to chip_rate. */
  current_chip %= chip_rate;

  s32 offset = current_chip;
  /* If current_chip is close to chip_rate, the code hasn't rolled over yet,
   * and thus next integration period should be longer than nominally. */
  if (current_chip > chip_rate / 2) {
    offset = current_chip - chip_rate;
  }

  /* No adjustment for signals that have no lock.
   * These are mainly the unconfirmed signals. */
  if (!plock) {
    offset = 0;
  }

  /* Log warning if an offset is applied (and we have a pessimistic lock). */
  if (0 != offset) {
    log_warn_mesid(mesid, "Applying code phase offset: %" PRIi32 "", offset);
  }

  return ms * chip_rate - offset;
}

/**
 * (Re-)initialize lock detector parameters.
 * \param[in,out] tracker_channel Tracker channel data.
 * \param[in]     init        Flag to indicate if the call to initialize or
 *                            to update.
 * \return None
 */
void tp_tracker_update_lock_detect_parameters(tracker_t *tracker_channel,
                                              bool init) {
  tp_profile_t *profile = &tracker_channel->profile;
  const tp_lock_detect_params_t *ldp = &profile->ld_phase_params;
  const tp_lock_detect_params_t *ldf = &profile->ld_freq_params;

  if (init) {
    lock_detect_init(
        &tracker_channel->ld_phase, ldp->k1, ldp->k2, ldp->lp, /*lo=*/0);
    lock_detect_init(
        &tracker_channel->ld_freq, ldf->k1, ldf->k2, ldf->lp, /*lo=*/0);
  } else {
    lock_detect_reinit(
        &tracker_channel->ld_phase, ldp->k1, ldp->k2, ldp->lp, /*lo=*/0);
    lock_detect_reinit(
        &tracker_channel->ld_freq, ldf->k1, ldf->k2, ldf->lp, /*lo=*/0);
  }
}

/**
 * Update tracker parameters when initializing or changing tracking mode.
 *
 * \param[in,out]  tracker_channel Tracker channel data.
 * \param[in]     init         Flag to indicate if the call to initialize or
 *                             to update.
 *
 * \return None
 */
void tp_profile_apply_config(tracker_t *tracker_channel, bool init) {
  me_gnss_signal_t mesid = tracker_channel->mesid;
  tp_profile_t *profile = &tracker_channel->profile;

  const tp_loop_params_t *l = &profile->loop_params;
  u8 prev_cn0_ms = 0;
  bool prev_use_alias_detection = 0;

  if (!init) {
    prev_cn0_ms = tp_get_cn0_ms(tracker_channel->tracking_mode);
    prev_use_alias_detection = tracker_channel->use_alias_detection;
  }

  tracker_channel->tracking_mode = profile->loop_params.mode;
  tracker_channel->use_alias_detection = profile->use_alias_detection;

  tracker_channel->has_next_params = false;

  /* For mode switch the current step is one step behind bit edge.
     For start-up it is 0. */
  u8 cycle_cnt = tp_get_cycle_count(l->mode);
  tracker_channel->cycle_no = init ? 0 : (cycle_cnt - 1);

  /**< C/N0 integration time */
  u8 cn0_ms = tp_get_cn0_ms(tracker_channel->tracking_mode);
  /**< Set initial rates */
  tl_rates_t rates;
  rates.code_freq =
      tracker_channel->code_phase_rate - code_to_chip_rate(mesid.code);
  rates.carr_freq = tracker_channel->carrier_freq;
  rates.acceleration = 0.0f;
  /**< Set tracking loop configuration parameters */
  tl_config_t config;
  tp_tl_get_config(l, &config);
  config.dll_loop_freq = 1000.f / tp_get_dll_ms(tracker_channel->tracking_mode);
  config.fll_loop_freq =
      1000.f / tp_get_flll_ms(tracker_channel->tracking_mode);
  config.fll_discr_freq =
      1000.f / tp_get_flld_ms(tracker_channel->tracking_mode);

  /* DLL init could be done nicer by initing DLL only */
  if (init || profile->dll_init) {
    log_debug_mesid(mesid, "Initializing TL");

    tp_tl_init(
        &tracker_channel->tl_state, profile->loop_params.ctrl, &rates, &config);
  } else {
    log_debug_mesid(mesid, "Re-tuning TL");

    /* Recalculate filter coefficients */
    tp_tl_retune(
        &tracker_channel->tl_state, profile->loop_params.ctrl, &config);
  }
  tp_tracker_update_lock_detect_parameters(tracker_channel, init);

  tracker_channel->flags &= ~TRACKER_FLAG_PLL_USE;
  tracker_channel->flags &= ~TRACKER_FLAG_FLL_USE;
  if (profile->loop_params.carr_bw > 0) {
    tracker_channel->flags |= TRACKER_FLAG_PLL_USE;
  }
  if (profile->loop_params.fll_bw > 0) {
    tracker_channel->flags |= TRACKER_FLAG_FLL_USE;
  }

  if (init || cn0_ms != prev_cn0_ms) {
    tp_cn0_params_t cn0_params;
    tp_profile_get_cn0_params(&tracker_channel->profile, &cn0_params);

    float cn0_t;
    float cn0_0;

    if (tracker_channel->flags & TRACKER_FLAG_CONFIRMED) {
      cn0_t = cn0_0 = tracker_channel->cn0;
    } else {
      /* When confirmation is required, set C/N0 below drop threshold and
       * check that is actually grows to correct range */
      cn0_0 =
          cn0_params.track_cn0_drop_thres_dbhz - TP_TRACKER_CN0_CONFIRM_DELTA;
      cn0_t = init ? tracker_channel->cn0 : tracker_channel->cn0_est.cn0_0;
    }

    /* Initialize C/N0 estimator and filter */
    track_cn0_init(mesid,                     /* ME signal for logging */
                   cn0_ms,                    /* C/N0 period in ms */
                   &tracker_channel->cn0_est, /* C/N0 estimator state */
                   cn0_0,                     /* Initial C/N0 value */
                   0);                        /* Flags */

    if (0 == (tracker_channel->flags & TRACKER_FLAG_CONFIRMED)) {
      tracker_channel->cn0_est.cn0_0 = cn0_t;
      tracker_channel->cn0 = cn0_t;
    }
    log_debug_mesid(mesid,
                    "CN0 update: CD=%f EST=%f CN0_0=%f",
                    tracker_channel->cn0,
                    cn0_0,
                    cn0_t);
  }

  if (tracker_channel->use_alias_detection) {
    float alias_detect_ms = tp_get_alias_ms(tracker_channel->tracking_mode);

    if (prev_use_alias_detection) {
      alias_detect_reinit(&tracker_channel->alias_detect,
                          (u32)(TP_TRACKER_ALIAS_DURATION_MS / alias_detect_ms),
                          alias_detect_ms * 1e-3f);
    } else {
      alias_detect_init(&tracker_channel->alias_detect,
                        (u32)(TP_TRACKER_ALIAS_DURATION_MS / alias_detect_ms),
                        alias_detect_ms * 1e-3f);
    }
  }
}

/**
 * Initializes tracker data.
 *
 * The method initializes tracker parameters.
 *
 * \param[in]     tracker_channel Tracker channel data
 * \param[in]     config       Configuration parameters.
 *
 * \return None
 */
void tp_tracker_init(tracker_t *tracker_channel,
                     const tp_tracker_config_t *config) {
  me_gnss_signal_t mesid = tracker_channel->mesid;

  tracker_ambiguity_unknown(tracker_channel);

  log_debug_mesid(
      mesid, "[+%" PRIu32 "ms] Tracker start", tracker_channel->update_count);

  /* Do tracking report to manager */
  tp_report_t report;
  memset(&report, 0, sizeof(report));
  report.cn0 = tracker_channel->cn0;

  tp_profile_init(tracker_channel, &report);

  if (config->show_unconfirmed_trackers) {
    tracker_channel->flags |= TRACKER_FLAG_CONFIRMED;
  }

  tp_profile_apply_config(tracker_channel, /* init = */ true);

  tracker_channel->flags |= TRACKER_FLAG_ACTIVE;
}

void tracker_cleanup(tracker_t *tracker_channel) {
  size_t cleanup_region_size =
      sizeof(tracker_t) - offsetof(tracker_t, cleanup_region_start);

  chMtxLock(&tracker_channel->mutex_pub);
  memset(&tracker_channel->cleanup_region_start, 0, cleanup_region_size);
  chMtxUnlock(&tracker_channel->mutex_pub);
}

/**
 * Releases tracker data.
 *
 * The method releases tracker state.
 *
 * \param[in] tracker_channel Tracker channel data
 */
void tp_tracker_disable(tracker_t *tracker_channel) {
  log_debug_mesid(tracker_channel->mesid,
                  "[+%" PRIu32 "ms] Tracker stop TOW=%" PRId32 "ms",
                  tracker_channel->update_count,
                  tracker_channel->TOW_ms);

  tracker_cleanup(tracker_channel);
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
 * \param[in] tracker_channel Tracker channel data
 *
 * \return Computed number of chips.
 */
u32 tp_tracker_compute_rollover_count(tracker_t *tracker_channel) {
  double code_phase_chips = tracker_channel->code_phase_prompt;

  bool plock = ((0 != (tracker_channel->flags & TRACKER_FLAG_HAS_PLOCK)) ||
                (0 != (tracker_channel->flags & TRACKER_FLAG_HAS_FLOCK)));
  u32 result_ms = 0;
  if (tracker_channel->has_next_params) {
    tp_profile_update_config(tracker_channel);
    tp_profile_t *profile = &tracker_channel->profile;
    result_ms = tp_get_current_cycle_duration(profile->loop_params.mode, 0);
  } else {
    result_ms = tp_get_rollover_cycle_duration(tracker_channel->tracking_mode,
                                               tracker_channel->cycle_no);
  }
  if (0 == result_ms) {
    log_error_mesid(tracker_channel->mesid,
                    "tracking_mode %d cycle_no %d result_ms 0",
                    tracker_channel->tracking_mode,
                    tracker_channel->cycle_no);
  }
  return tp_convert_ms_to_chips(
      tracker_channel->mesid, result_ms, code_phase_chips, plock);
}

/**
 * Initialize profile switching operation.
 *
 * Method checks if the profile switching is possible and required. The profile
 * switching is possible only when the bit boundary is reached, thus the check
 * is performed when the interval processed by FPGA is the last bit interval,
 * or, in other words, the next cycle has a bit sync flag and closes the bit.
 *
 * \param tracker_channel Tracker channel data
 *
 * \return None
 */
static void mode_change_init(tracker_t *tracker_channel) {
  bool confirmed = (0 != (tracker_channel->flags & TRACKER_FLAG_CONFIRMED));
  if (tracker_channel->has_next_params || !confirmed) {
    /* If the mode switch has been initiated - do nothing */
    return;
  }

  /* Compute time of the currently integrated period */
  u8 next_cycle = tp_next_cycle_counter(tracker_channel->tracking_mode,
                                        tracker_channel->cycle_no);
  u32 next_cycle_flags = tp_get_cycle_flags(tracker_channel, next_cycle);

  if (0 == (next_cycle_flags & TPF_BSYNC_UPD)) {
    return;
  }

  /* The switch is possible only when bit sync counter is updated: get the
   * bit update interval in ms. */
  u8 bit_ms = tp_get_bit_ms(tracker_channel->tracking_mode);

  if (tracker_next_bit_aligned(tracker_channel, bit_ms)) {
    /* When the bit sync is available and the next integration interval is the
     * last one in the bit, check if the profile switch is required. */
    if (tp_profile_has_new_profile(tracker_channel)) {
      /* Initiate profile change */
      tracker_channel->has_next_params = true;
    }
  }
}

/**
 * Finish profile switching operation.
 *
 * Method fetches new profile parameters and reconfigures as necessary.
 *
 * \param[in,out] tracker_channel Tracker channel data
 *
 * \return None
 */
static void mode_change_complete(tracker_t *tracker_channel) {
  if (tracker_channel->has_next_params) {
    tp_profile_switch(tracker_channel);
    tp_profile_update_config(tracker_channel);
    tp_profile_apply_config(tracker_channel, /* init = */ false);
  }
}

/**
 * Controls TL operation steps.
 *
 * The method updates the TL step according to tracking mode.
 *
 * \param tracker_channel Tracker channel data
 *
 * \return None
 */
void tp_tracker_update_cycle_counter(tracker_t *tracker_channel) {
  tracker_channel->cycle_no = tp_next_cycle_counter(
      tracker_channel->tracking_mode, tracker_channel->cycle_no);
}

/**
 * Second stage of false lock detection.
 *
 * Detect frequency error and update tracker state as appropriate.
 *
 * \param[in] tracker_channel Tracker channel data
 * \param[in] I in-phase signal component
 * \param[in] Q quadrature signal component
 *
 * \return None
 */
static void process_alias_error(tracker_t *tracker_channel, float I, float Q) {
  float err_hz = alias_detect_second(&tracker_channel->alias_detect, I, Q);

  if (fabsf(err_hz) < TRACK_ALIAS_THRESHOLD_HZ) {
    return;
  }

  /* The expected frequency errors depend on the modulated data rate.
     For more details on GPS, see:
     https://swiftnav.hackpad.com/Alias-PLL-lock-detector-in-L2C-4fWUJWUNnOE
     However in practice we see alias frequencies also in between the expected
     ones. So let's just correct the error we actually observe. */

  bool plock = (0 != (tracker_channel->flags & TRACKER_FLAG_HAS_PLOCK));
  bool flock = (0 != (tracker_channel->flags & TRACKER_FLAG_HAS_FLOCK));
  log_warn_mesid(tracker_channel->mesid,
                 "False freq detected: %.1f Hz (plock=%d,flock=%d)",
                 err_hz,
                 (int)plock,
                 (int)flock);

  tracker_ambiguity_unknown(tracker_channel);
  /* alias_detect_second() returns the freq correction value directly.
     So we can use it as is for the adjustment. */
  tp_tl_adjust(&tracker_channel->tl_state, err_hz);
}

/**
 * Updates tracker correlators.
 *
 * Updates tracker correlators with data read from FPGA according to current
 * cycle flags.
 *
 * \param[in]     tracker_channel Tracker channel data
 * \param[in]     cycle_flags  Current cycle flags.
 */
void tp_tracker_update_correlators(tracker_t *tracker_channel,
                                   u32 cycle_flags) {
  me_gnss_signal_t mesid = tracker_channel->mesid;
  tp_epl_corr_t cs_now;     /**< Correlations from FPGA */
  u32 sample_count;         /**< Sample count from FPGA */
  double code_phase_prompt; /**< Code phase from FPGA */
  double carrier_phase;     /**< Carrier phase from FPGA */
  u8 int_ms = 0;            /**< Current cycle duration in ms */

  /* Read NAP CORR register */
  nap_track_read_results(tracker_channel->nap_channel,
                         &sample_count,
                         cs_now.epl,
                         &code_phase_prompt,
                         &carrier_phase);

  tp_update_correlators(cycle_flags, &cs_now, &tracker_channel->corrs);

  /* Current cycle duration */
  int_ms = tp_get_current_cycle_duration(tracker_channel->tracking_mode,
                                         tracker_channel->cycle_no);

  u64 now = timing_getms();
  if (tracker_channel->updated_once) {
    u64 time_diff_ms = now - tracker_channel->update_timestamp_ms;
    if (time_diff_ms > NAP_CORR_LENGTH_MAX_MS) {
      log_warn_mesid(mesid,
                     "Unexpected tracking channel update rate: %" PRIu64 " ms",
                     time_diff_ms);
    }
  }
  tracker_channel->updated_once = true;
  tracker_channel->update_timestamp_ms = now;

  tracker_channel->sample_count = sample_count;
  tracker_channel->code_phase_prompt = code_phase_prompt;
  tracker_channel->carrier_phase_prev = tracker_channel->carrier_phase;
  tracker_channel->carrier_phase = carrier_phase;

  /* ToW update:
   * ToW along with carrier and code phases and sample number and health
   * shall be updated in sync.
   */
  bool decoded_tow;
  bool decoded_health;
  tracker_channel->TOW_ms_prev = tracker_channel->TOW_ms;
  tracker_channel->TOW_ms =
      tracker_tow_update(tracker_channel,
                         tracker_channel->TOW_ms,
                         int_ms,
                         &tracker_channel->TOW_residual_ns,
                         &decoded_tow,
                         &decoded_health);

  if (!tp_tow_is_sane(tracker_channel->TOW_ms)) {
    log_error_mesid(mesid,
                    "[+%" PRIu32 "ms] Error TOW from decoder %" PRId32 "(%s)",
                    tracker_channel->update_count,
                    tracker_channel->TOW_ms,
                    decoded_tow ? "decoded" : "propagated");

    tracker_channel->TOW_ms = TOW_UNKNOWN;
    tracker_channel->flags &= ~TRACKER_FLAG_TOW_DECODED;
    /* Still trust TRACKER_FLAG_GLO_HEALTH_DECODED, if it is set */
  }

  if (decoded_tow) {
    log_debug_mesid(mesid,
                    "[+%" PRIu32 "ms] Decoded TOW %" PRId32,
                    tracker_channel->update_count,
                    tracker_channel->TOW_ms);

    if (tracker_channel->TOW_ms != TOW_UNKNOWN) {
      tracker_channel->flags |= TRACKER_FLAG_TOW_DECODED;
    }
  }

  if (decoded_health) {
    /* GLO health data is also decoded along with TOW */
    if (IS_GLO(mesid)) {
      tracker_channel->flags |= TRACKER_FLAG_GLO_HEALTH_DECODED;
      tracker_channel->health = tracker_glo_sv_health_get(tracker_channel);
      log_debug_mesid(mesid,
                      "[+%" PRIu32 "ms] Decoded Health info %" PRIu8,
                      tracker_channel->update_count,
                      tracker_channel->health);
    }
  }

  if (TOW_UNKNOWN == tracker_channel->TOW_ms) {
    tracker_channel->flags &= ~TRACKER_FLAG_TOW_VALID;
  } else {
    tracker_channel->flags |= TRACKER_FLAG_TOW_VALID;
  }

  /* Channel run time. */
  tracker_channel->update_count += int_ms;
}

/**
 * Updates bit synchronization and decodes message
 *
 * \param[in]     tracker_channel Tracker channel data
 * \param[in]     cycle_flags  Current cycle flags.
 *
 * \return None
 */
void tp_tracker_update_bsync(tracker_t *tracker_channel, u32 cycle_flags) {
  if (0 != (cycle_flags & TPF_BSYNC_UPD)) {
    bool sensitivity_mode =
        (0 != (tracker_channel->flags & TRACKER_FLAG_SENSITIVITY_MODE));
    /* Bit sync / data decoding update counter. */
    u8 update_count_ms = tp_get_bit_ms(tracker_channel->tracking_mode);
    /* Bit sync advance / message decoding */
    tracker_bit_sync_update(tracker_channel,
                            update_count_ms,
                            tracker_channel->corrs.corr_bit.I,
                            tracker_channel->corrs.corr_bit.Q,
                            sensitivity_mode);

    /* TODO Update BS from ToW when appropriate. */
    /* TODO Add fast BS detection. */
    /* TODO Add bad BS recovery. */
  }
}

/**
 * Updates C/N0 estimators.
 *
 * \param[in]     tracker_channel Tracker channel data
 * \param[in]     cycle_flags  Current cycle flags.
 *
 * \return None
 */
void tp_tracker_update_cn0(tracker_t *tracker_channel, u32 cycle_flags) {
  float cn0 = tracker_channel->cn0_est.filter.yn;
  tp_cn0_params_t cn0_params;
  tp_profile_get_cn0_params(&tracker_channel->profile, &cn0_params);

  if (0 != (cycle_flags & TPF_CN0_USE)) {
    /* Workaround for
     * https://github.com/swift-nav/piksi_v3_bug_tracking/issues/475
     * don't update c/n0 if correlators data are 0 use
     * last post-filter sample instead */
    if (0 == tracker_channel->corrs.corr_cn0.prompt.I &&
        0 == tracker_channel->corrs.corr_cn0.prompt.Q) {
      log_warn_mesid(tracker_channel->mesid,
                     "Prompt I/Q: %" PRIi32 "/%" PRIi32,
                     tracker_channel->corrs.corr_cn0.prompt.I,
                     tracker_channel->corrs.corr_cn0.prompt.Q);
      log_warn_mesid(tracker_channel->mesid,
                     "Early I/Q: %" PRIi32 "/%" PRIi32,
                     tracker_channel->corrs.corr_cn0.early.I,
                     tracker_channel->corrs.corr_cn0.early.Q);
      log_warn_mesid(tracker_channel->mesid,
                     "Late I/Q: %" PRIi32 "/%" PRIi32,
                     tracker_channel->corrs.corr_cn0.late.I,
                     tracker_channel->corrs.corr_cn0.late.Q);
      log_warn_mesid(tracker_channel->mesid,
                     "Very Early I/Q: %" PRIi32 "/%" PRIi32,
                     tracker_channel->corrs.corr_cn0.very_early.I,
                     tracker_channel->corrs.corr_cn0.very_early.Q);
      log_warn_mesid(tracker_channel->mesid,
                     "Very Late I/Q: %" PRIi32 "/%" PRIi32,
                     tracker_channel->corrs.corr_cn0.very_late.I,
                     tracker_channel->corrs.corr_cn0.very_late.Q);
    } else {
      /* Update C/N0 estimate */
      cn0 = track_cn0_update(tracker_channel->mesid,
                             cn0_params.est,
                             &tracker_channel->cn0_est,
                             tracker_channel->corrs.corr_cn0.prompt.I,
                             tracker_channel->corrs.corr_cn0.prompt.Q,
                             tracker_channel->corrs.corr_cn0.very_early.I,
                             tracker_channel->corrs.corr_cn0.very_early.Q);
    }
  }

  if (cn0 > cn0_params.track_cn0_drop_thres_dbhz) {
    /* When C/N0 is above a drop threshold tracking shall continue. */
    tracker_channel->cn0_above_drop_thres_count = tracker_channel->update_count;
  }

  bool confirmed = (0 != (tracker_channel->flags & TRACKER_FLAG_CONFIRMED));
  bool inlock = ((0 != (tracker_channel->flags & TRACKER_FLAG_HAS_PLOCK)) ||
                 (0 != (tracker_channel->flags & TRACKER_FLAG_HAS_FLOCK)));
  if (cn0 > cn0_params.track_cn0_drop_thres_dbhz && !confirmed && inlock &&
      tracker_has_bit_sync(tracker_channel)) {
    tracker_channel->flags |= TRACKER_FLAG_CONFIRMED;
    log_debug_mesid(tracker_channel->mesid,
                    "CONFIRMED from %f to %d",
                    cn0,
                    tracker_channel->cn0_est.cn0_0);

    cn0 = tracker_channel->cn0_est.cn0_0;
    /* Re-initialize C/N0 estimator and filter */
    track_cn0_init(tracker_channel->mesid,          /* ME signal */
                   tracker_channel->cn0_est.cn0_ms, /* C/N0 period in ms */
                   &tracker_channel->cn0_est,       /* C/N0 estimator state */
                   cn0,                             /* Initial C/N0 value */
                   0);                              /* Flags */
  }

  if (0 != (tracker_channel->flags & TRACKER_FLAG_CONFIRMED)) {
    tracker_channel->cn0 = cn0;
  }

  if (cn0 < cn0_params.track_cn0_ambiguity_thres_dbhz) {
    /* C/N0 has dropped below threshold, indicate that the carrier phase
     * ambiguity is now unknown as cycle slips are likely. */
    tracker_ambiguity_unknown(tracker_channel);
  }

  if (cn0 < cn0_params.track_cn0_use_thres_dbhz) {
    /* Update the latest time we were below the threshold. */
    tracker_channel->cn0_below_use_thres_count = tracker_channel->update_count;
    /* Flag as low CN0 measurements. */
    tracker_channel->flags &= ~TRACKER_FLAG_CN0_SHORT;
  }

  if (cn0 >
      (cn0_params.track_cn0_use_thres_dbhz + TRACK_CN0_HYSTERESIS_THRES_DBHZ)) {
    /* Flag as high CN0 measurements. */
    tracker_channel->flags |= TRACKER_FLAG_CN0_SHORT;
  }
}

static void update_ld_phase(tracker_t *tracker_channel) {
  bool last_outp = tracker_channel->ld_phase.outp;

  lock_detect_update(&tracker_channel->ld_phase,
                     tracker_channel->corrs.corr_ld.I,
                     tracker_channel->corrs.corr_ld.Q,
                     tp_get_ld_ms(tracker_channel->tracking_mode));

  bool outp = tracker_channel->ld_phase.outp;

  if (outp) {
    tracker_channel->flags |= TRACKER_FLAG_HAS_PLOCK;
    tracker_channel->flags |= TRACKER_FLAG_HAD_PLOCK;
  } else if (last_outp && (TP_TM_INITIAL != tracker_channel->tracking_mode)) {
    log_info_mesid(tracker_channel->mesid, "PLL stress");
  }
}

static void update_ld_freq(tracker_t *tracker_channel) {
  /* FLL lock detector is based on frequency error seen by FLL discriminator */
  float unfiltered_freq_error = tracker_channel->unfiltered_freq_error;

  /* Calculate low-pass filtered frequency error */
  freq_lock_detect_update(&tracker_channel->ld_freq, unfiltered_freq_error);

  bool outp = tracker_channel->ld_freq.outp;

  if (outp) {
    tracker_channel->flags |= TRACKER_FLAG_HAS_FLOCK;
    tracker_channel->flags |= TRACKER_FLAG_HAD_FLOCK;
  }
}

/**
 * Updates PLL and FLL lock detectors.
 *
 * \param[in]     tracker_channel Tracker channel data
 * \param[in]     cycle_flags  Current cycle flags.
 *
 * \return None
 */
void tp_tracker_update_locks(tracker_t *tracker_channel, u32 cycle_flags) {
  /* Phase lock and frequency lock detectors are updated asynchronously. */
  if (0 != (cycle_flags & TPF_PLD_USE) || 0 != (cycle_flags & TPF_FLL_USE)) {
    bool outp_prev =
        tracker_channel->ld_phase.outp || tracker_channel->ld_freq.outp;

    if (0 != (cycle_flags & TPF_PLD_USE)) {
      tracker_channel->flags &= ~TRACKER_FLAG_HAS_PLOCK;

      if (0 != (tracker_channel->flags & TRACKER_FLAG_PLL_USE)) {
        update_ld_phase(tracker_channel);
      } else {
        /* Reset internal variables while phase lock detector is not in use. */
        tp_profile_t *profile = &tracker_channel->profile;
        const tp_lock_detect_params_t *ldp = &profile->ld_phase_params;
        lock_detect_init(
            &tracker_channel->ld_phase, ldp->k1, ldp->k2, ldp->lp, /*lo=*/0);
      }
    }

    if (0 != (cycle_flags & TPF_FLL_USE)) {
      tracker_channel->flags &= ~TRACKER_FLAG_HAS_FLOCK;
      update_ld_freq(tracker_channel);
    }

    bool outp = tracker_channel->ld_phase.outp || tracker_channel->ld_freq.outp;

    if (!outp_prev && outp) {
      u32 unlocked_ms = update_count_diff(
          tracker_channel, &tracker_channel->ld_pess_change_count);
      log_debug_mesid(
          tracker_channel->mesid, "Lock after %" PRIu32 "ms", unlocked_ms);
    }

    if (outp != outp_prev) {
      tracker_channel->ld_pess_change_count = tracker_channel->update_count;
    }
    if (outp) {
      tracker_channel->carrier_freq_at_lock = tracker_channel->carrier_freq;
    }
  }
  /*
   * Reset carrier phase ambiguity if there's doubt as to our phase lock.
   * Continue phase ambiguity reset until pessimistic PLL lock is reached. This
   * is done always to prevent incorrect handling of partial integration
   * intervals.
   */
  if (0 == (tracker_channel->flags & TRACKER_FLAG_HAS_PLOCK)) {
    tracker_ambiguity_unknown(tracker_channel);
  }
}

/**
 * Handle FLL tracker operations
 *
 * The method performs FLL tracking or FLL tracking assistance for PLL.
 *
 * \param[in]     tracker_channel Tracker channel data
 * \param[in]     cycle_flags  Current cycle flags.
 *
 * \return None
 */
void tp_tracker_update_fll(tracker_t *tracker_channel, u32 cycle_flags) {
  bool halfq = (0 != (cycle_flags & TPF_FLL_HALFQ));

  if (0 != (cycle_flags & TPF_FLL_USE)) {
    tp_tl_fll_update_second(
        &tracker_channel->tl_state, tracker_channel->corrs.corr_fll, halfq);
    tracker_channel->unfiltered_freq_error =
        tp_tl_get_fll_error(&tracker_channel->tl_state);
  }
}

/**
 * Runs PLL and DLL controller updates.
 *
 * This method updates PLL and DLL loops and additionally checks for DLL errors
 * and report data to profile managements.
 *
 * \param[in]     tracker_channel Tracker channel data
 * \param[in]     cycle_flags  Current cycle flags.
 *
 * \return None
 */
void tp_tracker_update_pll_dll(tracker_t *tracker_channel, u32 cycle_flags) {
  if (0 != (cycle_flags & TPF_EPL_USE)) {
    /* Output I/Q correlations using SBP if enabled for this channel */
    if (tracker_channel->tracking_mode != TP_TM_INITIAL) {
      tracker_correlations_send(tracker_channel,
                                tracker_channel->corrs.corr_epl.epl);
    }

    if (tracker_channel->has_next_params) {
      /* Transitional state: when the next interval has a different integration
       * period, the controller will give wrong correction. Due to that the
       * input parameters are scaled to stabilize tracker.
       */
      u8 new_dll_ms = tp_profile_get_next_loop_params_ms(
          tracker_channel->mesid, &tracker_channel->profile);
      u8 old_dll_ms = tp_get_dll_ms(tracker_channel->tracking_mode);

      if (old_dll_ms != new_dll_ms) {
        /* TODO utilize noise bandwidth and damping ratio */
        float k2 = (float)old_dll_ms / new_dll_ms;
        float k1 = sqrtf(k2);
        for (u32 i = 0; i < 3; i++) {
          tracker_channel->corrs.corr_epl.epl[i].I *= k1;
          tracker_channel->corrs.corr_epl.epl[i].Q *= k2;
        }
      }
    }

    tl_rates_t rates = {0};

    bool costas = true;
    tp_epl_corr_t corr_epl = tracker_channel->corrs.corr_epl;
    if (CODE_GPS_L2CM == tracker_channel->mesid.code) {
      corr_epl.prompt = corr_epl.very_late;
      costas = false;
    }
    tp_tl_update(&tracker_channel->tl_state, &corr_epl, costas);
    tp_tl_get_rates(&tracker_channel->tl_state, &rates);

    tracker_channel->carrier_freq = rates.carr_freq;
    tracker_channel->code_phase_rate =
        rates.code_freq + code_to_chip_rate(tracker_channel->mesid.code);
    tracker_channel->acceleration = rates.acceleration;

    /* Do tracking report to manager */
    tp_report_t report;
    memset(&report, 0, sizeof(report));
    if (0 == (tracker_channel->flags & TRACKER_FLAG_CONFIRMED)) {
      report.cn0 = tracker_channel->cn0_est.cn0_0;
    } else {
      report.cn0 = tracker_channel->cn0;
    }
    report.time_ms = tp_get_dll_ms(tracker_channel->tracking_mode);

    tp_profile_report_data(&tracker_channel->profile, &report);
  }
}

/**
 * Drops channels with measurement outliers.
 *
 * Check if an unexpected measurement is done and if so, flags the
 * channel for disposal
 *
 * \param[in,out] tracker Tracker channel data
 *
 * \return None
 */
static void tp_tracker_flag_outliers(tracker_t *tracker) {
  const float fMaxDoppler = code_to_sv_doppler_max(tracker->mesid.code) +
                            code_to_tcxo_doppler_max(tracker->mesid.code);

  /* remove channels with a large positive Doppler outlier */
  if (fabsf(tracker->carrier_freq) > fMaxDoppler) {
    log_debug_mesid(
        tracker->mesid, "Doppler %.2f too high", tracker->carrier_freq);
    (tracker->flags) |= TRACKER_FLAG_OUTLIER;
  }

  /* Check the maximum carrier frequency rate.
     Assume that the maximum expected acceleration is 7g and the carrier
     wavelength is of GPS L1CA (0.19m), which is a safe generalization for this
     purpose. */
  static const double max_freq_rate_hz_per_s =
      7. * STD_GRAVITY_ACCELERATION / 0.19;
  /* The carrier freq diff threshold we do not want to exceed */
  static const double max_freq_diff_hz = 70;
  /* work out the time difference needed to check the actual freq rate
     against max_freq_diff_hz threshold */
  static const u32 diff_interval_ms =
      (u32)(SECS_MS * max_freq_diff_hz / max_freq_rate_hz_per_s);

  u32 elapsed_ms = tracker->update_count - tracker->carrier_freq_timestamp_ms;
  if (elapsed_ms >= diff_interval_ms) {
    if (!tracker->carrier_freq_prev_valid) {
      tracker->carrier_freq_prev = tracker->carrier_freq;
      tracker->carrier_freq_prev_valid = true;
    }

    double diff_hz = tracker->carrier_freq - tracker->carrier_freq_prev;
    double elapsed_s = (double)elapsed_ms / SECS_MS;
    /* the elapsed time could be slightly larger than diff_interval_ms.
       So let's account for it in max_diff_hz */
    double max_diff_hz = max_freq_rate_hz_per_s * elapsed_s;
    if ((fabs(diff_hz) > max_diff_hz)) {
      log_debug_mesid(
          tracker->mesid, "Doppler difference %.2f is too high", diff_hz);
      tracker->flags |= TRACKER_FLAG_OUTLIER;
    }

    tracker->carrier_freq_prev = tracker->carrier_freq;
    tracker->carrier_freq_timestamp_ms = tracker->update_count;
  }
}

/**
 * Runs false lock detection logic.
 *
 * \param[in,out] tracker_channel Tracker channel data
 * \param[in]     cycle_flags  Current cycle flags.
 *
 * \return None
 */
void tp_tracker_update_alias(tracker_t *tracker_channel, u32 cycle_flags) {
  if (!tracker_channel->use_alias_detection) {
    return;
  }

  bool do_first = (0 != (cycle_flags & TPF_ALIAS_1ST));
  bool do_second = (0 != (cycle_flags & TPF_ALIAS_2ND));

  if (!do_first && !do_second) {
    return;
  }

  float I = tracker_channel->corrs.corr_ad.I;
  float Q = tracker_channel->corrs.corr_ad.Q;

  if (do_second) {
    bool inlock = ((0 != (tracker_channel->flags & TRACKER_FLAG_HAS_PLOCK)) ||
                   (0 != (tracker_channel->flags & TRACKER_FLAG_HAS_FLOCK)));
    if (inlock) {
      process_alias_error(tracker_channel, I, Q);
    }
    do_first = true;
  }

  if (do_first) {
    alias_detect_first(&tracker_channel->alias_detect, I, Q);
  }
}

/**
 * Computes filtered SV doppler for cross-correlation and other uses.
 *
 * The method updates SV doppler value on bit edges. The doppler is not taken
 * from
 * controller, but is a LP product of controller output.
 *
 * \param[in,out  tracker_channel Tracker channel data
 * \param[in]     cycle_flags  Current cycle flags.
 * \param[in]     config       Generic tracker configuration.
 *
 * \return None
 */
void tp_tracker_filter_doppler(tracker_t *tracker_channel,
                               u32 cycle_flags,
                               const tp_tracker_config_t *config) {
  if (0 != (cycle_flags & TPF_BSYNC_UPD) &&
      tracker_bit_aligned(tracker_channel)) {
    float xcorr_freq = tracker_channel->carrier_freq;

    if (tracker_channel->flags & TRACKER_FLAG_XCORR_FILTER_ACTIVE) {
      xcorr_freq = lp1_filter_update(
          &tracker_channel->xcorr_filter, &config->xcorr_f_params, xcorr_freq);
    } else {
      lp1_filter_init(
          &tracker_channel->xcorr_filter, &config->xcorr_f_params, xcorr_freq);
      tracker_channel->flags |= TRACKER_FLAG_XCORR_FILTER_ACTIVE;
    }

    tracker_channel->xcorr_freq = xcorr_freq;
  }
}

/**
 * Mode switching control.
 *
 * \param tracker_channel Tracker channel data
 *
 * \return None
 */
void tp_tracker_update_mode(tracker_t *tracker_channel) {
  mode_change_complete(tracker_channel);
  mode_change_init(tracker_channel);
}

/**
 * Default tracking loop.
 *
 * \param tracker_channel Tracking channel data.
 * \param config       Generic tracker configuration.
 *
 * \return Flags, used in the current tracking cycle.
 */
u32 tp_tracker_update(tracker_t *tracker_channel,
                      const tp_tracker_config_t *config) {
  /*
   * State machine control: control is a combination of actions permitted by
   * the tracker state and flags specific for current cycle.
   */
  u32 cflags = tp_get_cycle_flags(tracker_channel, tracker_channel->cycle_no);

  tp_tracker_update_correlators(tracker_channel, cflags);
  tp_tracker_update_bsync(tracker_channel, cflags);
  tp_tracker_update_cn0(tracker_channel, cflags);
  tp_tracker_update_fll(tracker_channel, cflags);
  tp_tracker_update_pll_dll(tracker_channel, cflags);
  tp_tracker_update_locks(tracker_channel, cflags);
  tp_tracker_flag_outliers(tracker_channel);
  tp_tracker_update_alias(tracker_channel, cflags);
  tp_tracker_filter_doppler(tracker_channel, cflags, config);
  tp_tracker_update_mode(tracker_channel);

  u32 chips_to_correlate = tp_tracker_compute_rollover_count(tracker_channel);
  /* Write NAP UPDATE register. */
  nap_track_update(tracker_channel->nap_channel,
                   tracker_channel->carrier_freq,
                   tracker_channel->code_phase_rate,
                   chips_to_correlate);

  tp_tracker_update_cycle_counter(tracker_channel);

  return cflags;
}

static bool tow_is_bit_aligned(tracker_t *tracker_channel) {
  me_gnss_signal_t mesid = tracker_channel->mesid;
  u8 bit_length = tracker_bit_length_get(tracker_channel);

  if (tracker_channel->TOW_ms == TOW_UNKNOWN) {
    return false;
  }

  /*
   * Verify ToW bit alignment
   * Current block assumes the bit sync has been reached and current
   * interval has closed a bit interval. ToW shall be aligned by bit
   * duration, which is:
   * 20ms for GPS and QZSS L1/L2/L5, plus Beidou B1/B2 with D1 data
   * 10ms for GLO L1/L2
   * 2 ms for SBAS and Beidou B1/B2 with D2 data
   */
  u8 tail = tracker_channel->TOW_ms % bit_length;
  if (0 != tail) {
    /* If this correction is needed, then there is something wrong
       either with the TOW cache update or with the bit sync */
    s8 error_ms = tail < (bit_length >> 1) ? -tail : bit_length - tail;

    log_error_mesid(mesid,
                    "[+%" PRIu32
                    "ms] TOW error detected: "
                    "error=%" PRId8 "ms old_tow=%" PRId32,
                    tracker_channel->update_count,
                    error_ms,
                    tracker_channel->TOW_ms);

    /* This is rude, but safe. Do not expect it to happen normally. */
    tracker_channel->flags |= TRACKER_FLAG_OUTLIER;
    return false;
  }
  return true;
}

static bool should_update_tow_cache(const tracker_t *tracker_channel) {
  me_gnss_signal_t mesid = tracker_channel->mesid;

  bool confirmed = (0 != (tracker_channel->flags & TRACKER_FLAG_CONFIRMED));
  bool cn0_ok = (tracker_channel->cn0 >= CN0_TOW_CACHE_THRESHOLD);
  bool tow_is_known = (TOW_UNKNOWN != tracker_channel->TOW_ms);
  bool responsible_for_update = false;

  if (CODE_GPS_L1CA == mesid.code || CODE_GLO_L1OF == mesid.code ||
      CODE_SBAS_L1CA == mesid.code || CODE_QZS_L1CA == mesid.code ||
      CODE_BDS2_B11 == mesid.code) {
    responsible_for_update = true;
  } else {
    me_gnss_signal_t mesid_L1;
    if (CODE_GPS_L2CM == mesid.code) {
      mesid_L1 = construct_mesid(CODE_GPS_L1CA, mesid.sat);
    } else if (CODE_GLO_L2OF == mesid.code) {
      mesid_L1 = construct_mesid(CODE_GLO_L1OF, mesid.sat);
    } else if (CODE_QZS_L2CM == mesid.code) {
      mesid_L1 = construct_mesid(CODE_QZS_L1CA, mesid.sat);
    } else if (CODE_BDS2_B2 == mesid.code) {
      mesid_L1 = construct_mesid(CODE_BDS2_B11, mesid.sat);
    } else {
      assert(!"Unsupported TOW cache code");
    }
    tracker_t *trk_ch = tracker_channel_get_by_mesid(mesid_L1);
    responsible_for_update =
        ((NULL == trk_ch) || (TOW_UNKNOWN == trk_ch->TOW_ms));
  }

  /* Update TOW cache if:
   * - CN0 is OK
   * - Tracker is confirmed
   * - Tracker TOW is known
   * - Tracker is responsible for TOW cache updates
   */
  return (cn0_ok && confirmed && tow_is_known && responsible_for_update);
}

/**
 * Performs ToW caching and propagation.
 *
 * GPS L1CA / L2CM and GLO L1OF / L2OF both use shared structure for ToW
 * caching.
 * When L1 tracker is running, it is responsible for cache updates. Otherwise L2
 * tracker updates the cache. The time difference between signals is ignored
 * as small.
 *
 * Tracker performs ToW update/propagation only on bit edge. This makes
 * it more robust to propagation errors.
 *
 * \param[in,out] tracker_channel Tracker channel data
 *
 * \return None
 */
void tracker_tow_cache(tracker_t *tracker_channel) {
  /* If TOW is unknown, check if a valid cached TOW is available. */
  if (TOW_UNKNOWN == tracker_channel->TOW_ms) {
    propagate_tow_from_sid_db(tracker_channel);
  }

  /* Check that tracker has valid TOW. */
  if (!tow_is_bit_aligned(tracker_channel)) {
    return;
  }

  /* Check if a tracker should update TOW cache. */
  if (should_update_tow_cache(tracker_channel)) {
    update_tow_in_sid_db(tracker_channel);
  }
}
