/*
 * Copyright (C) 2016 - 2017 Swift Navigation Inc.
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

#include <libswiftnav/constants.h>
#include <libswiftnav/time.h>
#include <libswiftnav/track.h>

#include <track.h>

#include "signal.h"
#include "timing.h"
#include "track_sbp.h"
#include "track_sid_db.h"

#include <assert.h>
#include <inttypes.h>
#include <math.h>
#include <string.h>

/** False lock detector filter interval in ms. */
#define TP_TRACKER_ALIAS_DURATION_MS (1000)
/** Initial C/N0 for confirmation [dB/Hz] */
#define TP_TRACKER_CN0_CONFIRM_DELTA (2.f)

/** DLL error threshold. Used to assess FLL frequency lock. In [Hz]. */
#define TP_FLL_DLL_ERR_THRESHOLD_HZ 0.1

/** C/N0 threshold long interval [ms] */
#define TRACK_CN0_THRES_COUNT_LONG 2000

/** C/N0 threshold short interval [ms] */
#define TRACK_CN0_THRES_COUNT_SHORT 100

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

  /* L2CL code phase has been adjusted by 1,
   * due to L2CM code chip occupying first slot. */
  if (CODE_GPS_L2CL == mesid.code) {
    current_chip += 1;
  }

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
 * \param[in]     next_params Tracking configuration.
 * \param[in]     init        Flag to indicate if the call to initialize or
 *                            to update.
 * \return None
 */
void tp_tracker_update_lock_detect_parameters(
    tracker_channel_t *tracker_channel,
    const tp_config_t *next_params,
    bool init) {
  const tp_lock_detect_params_t *ld = &next_params->lock_detect_params;
  /* Lock detector integration time */
  bool mode_switch = tp_is_fll_ctrl(tracker_channel->tl_state.ctrl) !=
                     tp_is_fll_ctrl(next_params->loop_params.ctrl);

  if (init || mode_switch) {
    lock_detect_init(
        &tracker_channel->lock_detect, ld->k1, ld->k2, ld->lp, ld->lo);
  } else {
    lock_detect_reinit(
        &tracker_channel->lock_detect, ld->k1, ld->k2, ld->lp, ld->lo);
  }
}

/**
 * Update tracker parameters when initializing or changing tracking mode.
 *
 * \param[in,out]  tracker_channel Tracker channel data.
 * \param[in]     next_params  Tracking configuration.
 * \param[in]     init         Flag to indicate if the call to initialize or
 *                             to update.
 *
 * \return None
 */
void tp_tracker_update_parameters(tracker_channel_t *tracker_channel,
                                  const tp_config_t *next_params,
                                  bool init) {
  me_gnss_signal_t mesid = tracker_channel->mesid;

  const tp_loop_params_t *l = &next_params->loop_params;
  u8 prev_cn0_ms = 0;
  bool prev_use_alias_detection = 0;

  if (!init) {
    prev_cn0_ms = tp_get_cn0_ms(tracker_channel->tracking_mode);
    prev_use_alias_detection = tracker_channel->use_alias_detection;
  }

  tracker_channel->tracking_mode = next_params->loop_params.mode;
  tracker_channel->use_alias_detection = next_params->use_alias_detection;

  tracker_channel->has_next_params = false;

  /* Set the step number for mode switch. Current step is one step behind bit
   * edge */
  u8 cycle_cnt = tp_get_cycle_count(l->mode);
  tracker_channel->cycle_no = cycle_cnt - 1;

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
  config.carr_to_code = mesid_to_carr_to_code(mesid);

  if (init) {
    log_debug_mesid(mesid, "Initializing TL");

    tp_tl_init(&tracker_channel->tl_state,
               next_params->loop_params.ctrl,
               &rates,
               &config);
  } else {
    log_debug_mesid(mesid, "Re-tuning TL");

    /* Recalculate filter coefficients */
    tp_tl_retune(
        &tracker_channel->tl_state, next_params->loop_params.ctrl, &config);
  }
  tp_tracker_update_lock_detect_parameters(tracker_channel, next_params, init);

  if (tp_tl_is_fll(&tracker_channel->tl_state)) {
    tracker_channel->flags |= TRACKER_FLAG_FLL_USE;
    tracker_channel->flags &= ~TRACKER_FLAG_PLL_USE;
  } else if (tp_tl_is_pll(&tracker_channel->tl_state)) {
    tracker_channel->flags |= TRACKER_FLAG_PLL_USE;
    tracker_channel->flags &= ~TRACKER_FLAG_FLL_USE;
    if (next_params->loop_params.fll_bw > 0.f) {
      tracker_channel->flags |= TRACKER_FLAG_FLL_USE;
    }
  } else {
    tracker_channel->flags &= ~TRACKER_FLAG_PLL_USE;
    tracker_channel->flags &= ~TRACKER_FLAG_FLL_USE;
    assert(!"Unexpected control mode");
  }

  /* Export loop controller parameters */
  tracker_channel->ctrl_params.int_ms =
      tp_get_dll_ms(tracker_channel->tracking_mode);
  tracker_channel->ctrl_params.dll_bw = next_params->loop_params.code_bw;

  if (tracker_channel->flags & TRACKER_FLAG_PLL_USE) {
    tracker_channel->ctrl_params.pll_bw = next_params->loop_params.carr_bw;
  } else {
    tracker_channel->ctrl_params.pll_bw = 0.f;
  }

  if (tracker_channel->flags & TRACKER_FLAG_FLL_USE) {
    tracker_channel->ctrl_params.fll_bw = next_params->loop_params.fll_bw;
  } else {
    tracker_channel->ctrl_params.fll_bw = 0.f;
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
    u8 alias_detect_ms = tp_get_alias_ms(tracker_channel->tracking_mode);

    if (prev_use_alias_detection) {
      alias_detect_reinit(&tracker_channel->alias_detect,
                          TP_TRACKER_ALIAS_DURATION_MS / alias_detect_ms,
                          alias_detect_ms * 1e-3f);
    } else {
      alias_detect_init(&tracker_channel->alias_detect,
                        TP_TRACKER_ALIAS_DURATION_MS / alias_detect_ms,
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
void tp_tracker_init(tracker_channel_t *tracker_channel,
                     const tp_tracker_config_t *config) {
  tp_config_t init_profile;
  me_gnss_signal_t mesid = tracker_channel->mesid;

  tracker_ambiguity_unknown(tracker_channel);

  log_debug_mesid(
      mesid, "[+%" PRIu32 "ms] Tracker start", tracker_channel->update_count);

  /* Do tracking report to manager */
  tp_report_t report;
  report.bsync = false;
  report.carr_freq = tracker_channel->carrier_freq;
  report.code_phase_rate = tracker_channel->code_phase_rate;
  report.cn0 = report.cn0_raw = tracker_channel->cn0;
  report.olock = false;
  report.plock = false;
  report.sample_count = tracker_channel->sample_count;
  report.time_ms = 0;

  if (TP_RESULT_SUCCESS !=
      tp_profile_init(
          mesid, &tracker_channel->profile, &report, &init_profile)) {
    log_error_mesid(mesid, "Tracker start error");
  }

  if (config->show_unconfirmed_trackers) {
    tracker_channel->flags |= TRACKER_FLAG_CONFIRMED;
  }

  tp_tracker_update_parameters(tracker_channel,
                               &init_profile,
                               /* init = */ true);

  tracker_channel->flags |= TRACKER_FLAG_ACTIVE;
}

void tracker_cleanup(tracker_channel_t *tracker_channel) {
  size_t cleanup_region_size =
      sizeof(tracker_channel_t) -
      offsetof(tracker_channel_t, cleanup_region_start);

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
void tp_tracker_disable(tracker_channel_t *tracker_channel) {
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
u32 tp_tracker_compute_rollover_count(tracker_channel_t *tracker_channel) {
  double code_phase_chips = tracker_channel->code_phase_prompt;

  bool plock = tracker_channel->lock_detect.outp;
  u32 result_ms = 0;
  if (tracker_channel->has_next_params) {
    tp_config_t next_params;
    tp_profile_get_config(
        tracker_channel->mesid, &tracker_channel->profile, &next_params, false);
    result_ms = tp_get_current_cycle_duration(next_params.loop_params.mode, 0);
  } else {
    result_ms = tp_get_rollover_cycle_duration(tracker_channel->tracking_mode,
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
static void mode_change_init(tracker_channel_t *tracker_channel) {
  bool confirmed = (0 != (tracker_channel->flags & TRACKER_FLAG_CONFIRMED));
  if (tracker_channel->has_next_params || !confirmed) {
    /* If the mode switch has been initiated - do nothing */
    return;
  }

  /* Compute time of the currently integrated period */
  u8 next_cycle = tp_next_cycle_counter(tracker_channel->tracking_mode,
                                        tracker_channel->cycle_no);
  u32 next_cycle_flags = tp_get_cycle_flags(tracker_channel, next_cycle);

  if (0 != (next_cycle_flags & TP_CFLAG_BSYNC_UPDATE)) {
    /* The switch is possible only when bit sync counter is updated: get the
     * bit update interval in ms. */
    u8 bit_ms = tp_get_bit_ms(tracker_channel->tracking_mode);

    if (tracker_next_bit_aligned(tracker_channel, bit_ms)) {
      /* When the bit sync is available and the next integration interval is the
       * last one in the bit, check if the profile switch is required. */
      if (tp_profile_has_new_profile(tracker_channel->mesid,
                                     &tracker_channel->profile)) {
        /* Initiate profile change */
        tracker_channel->has_next_params = true;
      }
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
static void mode_change_complete(tracker_channel_t *tracker_channel) {
  if (tracker_channel->has_next_params) {
    tp_config_t next_params;
    tp_profile_get_config(
        tracker_channel->mesid, &tracker_channel->profile, &next_params, true);

    /* If there is a stage transition in progress, update parameters for the
     * next iteration. */
    log_debug_mesid(tracker_channel->mesid,
                    "Reconfiguring tracking profile: new mode=%s",
                    tp_get_mode_str(next_params.loop_params.mode));

    tp_tracker_update_parameters(tracker_channel,
                                 &next_params,
                                 /* init = */ false);
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
void tp_tracker_update_cycle_counter(tracker_channel_t *tracker_channel) {
  tracker_channel->cycle_no = tp_next_cycle_counter(
      tracker_channel->tracking_mode, tracker_channel->cycle_no);
}

/**
 * Update #TRACKER_FLAG_BIT_POLARITY_KNOWN and
 * #TRACKER_FLAG_BIT_INVERTED flags based on
 * tracker_channel_t::bit_polarity value.
 *
 * \param[in,out] tracker_channel Tracker channel data.
 */
void update_bit_polarity_flags(tracker_channel_t *tracker_channel) {
  if ((BIT_POLARITY_UNKNOWN != tracker_channel->bit_polarity) &&
      (tracker_channel->flags & TRACKER_FLAG_HAS_PLOCK)) {
    /* Nav bit polarity is known, i.e. half-cycles have been resolved.
     * bit polarity known flag is set only when phase lock to prevent the
     * situation when channel loses an SV, but decoder just finished TOW
     * decoding
     * which cause bit polarity know flag set */
    tracker_channel->flags |= TRACKER_FLAG_BIT_POLARITY_KNOWN;
  } else {
    tracker_channel->flags &= ~TRACKER_FLAG_BIT_POLARITY_KNOWN;
  }

  if (tracker_channel->flags & TRACKER_FLAG_BIT_POLARITY_KNOWN) {
    if (BIT_POLARITY_INVERTED == tracker_channel->bit_polarity) {
      tracker_channel->flags |= TRACKER_FLAG_BIT_INVERTED;
      log_info_mesid(tracker_channel->mesid, "detected inverted polarity");
    } else {
      tracker_channel->flags &= ~TRACKER_FLAG_BIT_INVERTED;
      log_info_mesid(tracker_channel->mesid, "detected good polarity");
    }
  }
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
static s32 tp_tl_detect_alias(alias_detect_t *alias_detect, float I, float Q) {
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
 * \param[in] tracker_channel Tracker channel data
 *
 * \return None
 */
static void process_alias_error(tracker_channel_t *tracker_channel) {
  float I =
      tracker_channel->corrs.corr_ad.I - tracker_channel->alias_detect.first_I;
  float Q =
      tracker_channel->corrs.corr_ad.Q - tracker_channel->alias_detect.first_Q;

  s32 err_hz = tp_tl_detect_alias(&tracker_channel->alias_detect, I, Q);

  if (0 != err_hz) {
    if (tracker_channel->lock_detect.outp) {
      log_warn_mesid(tracker_channel->mesid,
                     "False phase lock detected: %" PRId32 " Hz",
                     err_hz);
    } else {
      log_debug_mesid(tracker_channel->mesid,
                      "False optimistic lock detected: %" PRId32 " Hz",
                      err_hz);
    }

    tracker_ambiguity_unknown(tracker_channel);
    tp_tl_adjust(&tracker_channel->tl_state, err_hz);
  }
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
void tp_tracker_update_correlators(tracker_channel_t *tracker_channel,
                                   u32 cycle_flags) {
  me_gnss_signal_t mesid = tracker_channel->mesid;
  tp_epl_corr_t cs_now;     /**< Correlations from FPGA */
  u32 sample_count;         /**< Sample count from FPGA */
  double code_phase_prompt; /**< Code phase from FPGA */
  double carrier_phase;     /**< Carrier phase from FPGA */
  u8 int_ms = 0;            /**< Current cycle duration in ms */

  /* Read correlations. */
  tracker_correlations_read(tracker_channel->nap_channel,
                            cs_now.epl,
                            &sample_count,
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
  tracker_channel->TOW_ms_prev = tracker_channel->TOW_ms;
  tracker_channel->TOW_ms =
      tracker_tow_update(tracker_channel,
                         tracker_channel->TOW_ms,
                         int_ms,
                         &tracker_channel->TOW_residual_ns,
                         &decoded_tow);

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

    /* GLO health data is also decoded along with TOW */
    if (is_glo_sid(mesid)) {
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
void tp_tracker_update_bsync(tracker_channel_t *tracker_channel,
                             u32 cycle_flags) {
  if (0 != (cycle_flags & TP_CFLAG_BSYNC_UPDATE)) {
    bool sensitivity_mode = tp_tl_is_fll(&tracker_channel->tl_state);
    /* Bit sync / data decoding update counter. */
    u8 update_count_ms = tp_get_bit_ms(tracker_channel->tracking_mode);
    /* Bit sync advance / message decoding */
    tracker_bit_sync_update(tracker_channel,
                            update_count_ms,
                            tracker_channel->corrs.corr_epl.prompt.I,
                            tracker_channel->corrs.corr_epl.prompt.Q,
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
void tp_tracker_update_cn0(tracker_channel_t *tracker_channel,
                           u32 cycle_flags) {
  float cn0 = tracker_channel->cn0_est.filter.yn;
  tp_cn0_params_t cn0_params;
  tp_profile_get_cn0_params(&tracker_channel->profile, &cn0_params);

  if (0 != (cycle_flags & TP_CFLAG_CN0_USE)) {
    /* Update C/N0 estimate */
    cn0 = track_cn0_update(tracker_channel->mesid,
                           cn0_params.est,
                           &tracker_channel->cn0_est,
                           tracker_channel->corrs.corr_cn0.prompt.I,
                           tracker_channel->corrs.corr_cn0.prompt.Q,
                           tracker_channel->corrs.corr_cn0.very_early.I,
                           tracker_channel->corrs.corr_cn0.very_early.Q);
  }

  if (cn0 > cn0_params.track_cn0_drop_thres_dbhz ||
      (tp_tl_is_pll(&tracker_channel->tl_state) &&
       tracker_channel->lock_detect.outp)) {
    /* When C/N0 is above a drop threshold or there is a pessimistic lock,
     * tracking shall continue.
     */
    tracker_channel->cn0_above_drop_thres_count = tracker_channel->update_count;
  }

  bool confirmed = (0 != (tracker_channel->flags & TRACKER_FLAG_CONFIRMED));
  if (cn0 > cn0_params.track_cn0_drop_thres_dbhz && !confirmed &&
      tracker_channel->lock_detect.outp &&
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
  }

  /* Check C/N0 has been above threshold for a long time (RTK). */
  u32 cn0_threshold_count_ms = (tracker_channel->update_count -
                                tracker_channel->cn0_below_use_thres_count);
  if (cn0_threshold_count_ms > TRACK_CN0_THRES_COUNT_LONG) {
    tracker_channel->flags |= TRACKER_FLAG_CN0_LONG;
  }
  /* Check C/N0 has been above threshold for the minimum time (SPP). */
  if (cn0_threshold_count_ms > TRACK_CN0_THRES_COUNT_SHORT) {
    tracker_channel->flags |= TRACKER_FLAG_CN0_SHORT;
  }
}

/**
 * Updates PLL and FLL lock registers.
 *
 * Updates PLL optimistic and pessimistic registers. In addition, checks if
 * FLL lock is present in FLL tracking mode.
 *
 * \param[in]     tracker_channel Tracker channel data
 * \param[in]     cycle_flags  Current cycle flags.
 *
 * \return None
 */
void tp_tracker_update_locks(tracker_channel_t *tracker_channel,
                             u32 cycle_flags) {
  bool fll_loop = (0 != (tracker_channel->flags & TRACKER_FLAG_FLL_USE));
  bool pll_loop = (0 != (tracker_channel->flags & TRACKER_FLAG_PLL_USE));

  if (0 != (cycle_flags & TP_CFLAG_LD_USE)) {
    /* Update PLL/FLL lock detector */
    bool last_outp = tracker_channel->lock_detect.outp;
    bool outp = false;

    tracker_channel->flags &= ~TRACKER_FLAG_HAS_PLOCK;
    tracker_channel->flags &= ~TRACKER_FLAG_HAS_FLOCK;

    if (pll_loop) {
      lock_detect_update(&tracker_channel->lock_detect,
                         tracker_channel->corrs.corr_ld.I,
                         tracker_channel->corrs.corr_ld.Q,
                         tp_get_ld_ms(tracker_channel->tracking_mode));

      outp = tracker_channel->lock_detect.outp;

      if (outp) {
        tracker_channel->flags |= TRACKER_FLAG_HAS_PLOCK;
        tracker_channel->flags |= TRACKER_FLAG_HAD_PLOCK;
        tracker_channel->carrier_freq_at_lock = tracker_channel->carrier_freq;
      }
    } else if (fll_loop) {
      /* In FLL mode, there is no phase lock. Check if FLL/DLL error is small */

      float dll_err = tp_tl_get_dll_error(&tracker_channel->tl_state);

      lock_detect_update(&tracker_channel->lock_detect,
                         TP_FLL_DLL_ERR_THRESHOLD_HZ,
                         dll_err,
                         tp_get_ld_ms(tracker_channel->tracking_mode));

      outp = tracker_channel->lock_detect.outp;

      if (outp) {
        tracker_channel->flags |= TRACKER_FLAG_HAS_FLOCK;
        tracker_channel->flags |= TRACKER_FLAG_HAD_FLOCK;
      }
    }

    if (outp != last_outp) {
      tracker_channel->ld_pess_change_count = tracker_channel->update_count;
    }

    if (last_outp && !outp &&
        (tracker_channel->tracking_mode != TP_TM_INITIAL)) {
      if (fll_loop) {
        log_info_mesid(tracker_channel->mesid, "FLL stress");
      } else if (pll_loop) {
        log_info_mesid(tracker_channel->mesid, "PLL stress");
      }
    }
  }
  /*
   * Reset carrier phase ambiguity if there's doubt as to our phase lock.
   * Continue phase ambiguity reset until pessimistic PLL lock is reached. This
   * is done always to prevent incorrect handling of partial integration
   * intervals.
   */
  if (!pll_loop || !tracker_channel->lock_detect.outp) {
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
void tp_tracker_update_fll(tracker_channel_t *tracker_channel,
                           u32 cycle_flags) {
  if (0 != (cycle_flags & TP_CFLAG_FLL_SECOND)) {
    tp_tl_fll_update_second(&tracker_channel->tl_state,
                            tracker_channel->corrs.corr_fll);
  }

  if (0 != (cycle_flags & TP_CFLAG_FLL_USE)) {
    tp_tl_fll_update(&tracker_channel->tl_state);
  }

  if (0 != (cycle_flags & TP_CFLAG_FLL_FIRST)) {
    tp_tl_fll_update_first(&tracker_channel->tl_state,
                           tracker_channel->corrs.corr_fll);
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
void tp_tracker_update_pll_dll(tracker_channel_t *tracker_channel,
                               u32 cycle_flags) {
  if (0 != (cycle_flags & TP_CFLAG_EPL_USE)) {
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

    bool costas = (tracker_channel->mesid.code != CODE_GPS_L2CL);
    tp_tl_update(
        &tracker_channel->tl_state, &tracker_channel->corrs.corr_epl, costas);
    tp_tl_get_rates(&tracker_channel->tl_state, &rates);

    tracker_channel->carrier_freq = rates.carr_freq;
    tracker_channel->code_phase_rate =
        rates.code_freq + code_to_chip_rate(tracker_channel->mesid.code);
    tracker_channel->acceleration = rates.acceleration;

    /* Do tracking report to manager */
    tp_report_t report;
    report.bsync = tracker_has_bit_sync(tracker_channel);
    report.carr_freq = tracker_channel->carrier_freq;
    report.code_phase_rate = tracker_channel->code_phase_rate;
    report.cn0_raw = tracker_channel->cn0;
    if (0 == (tracker_channel->flags & TRACKER_FLAG_CONFIRMED)) {
      report.cn0 = tracker_channel->cn0_est.cn0_0;
    } else {
      report.cn0 = tracker_channel->cn0;
    }
    report.olock = tracker_channel->lock_detect.outo;
    report.plock = tracker_channel->lock_detect.outp;
    report.lock_i = tracker_channel->lock_detect.lpfi.y;
    report.lock_q = tracker_channel->lock_detect.lpfq.y;
    report.sample_count = tracker_channel->sample_count;
    report.time_ms = tp_get_dll_ms(tracker_channel->tracking_mode);
    report.acceleration = rates.acceleration;

    tp_profile_report_data(tracker_channel, &tracker_channel->profile, &report);
  }
}

/**
 * Drops channels with measurement outliers.
 *
 * Check if an unexpected measurement is done and if so, flags the
 * channel for disposal
 *
 * \param[in,out] tracker_channel Tracker channel data
 *
 * \return None
 */
static void tp_tracker_flag_outliers(tracker_channel_t *tracker_channel) {
  const float fMaxDoppler =
      code_to_sv_doppler_max(tracker_channel->mesid.code) +
      code_to_tcxo_doppler_max(tracker_channel->mesid.code);

  /* remove channels with a large positive Doppler outlier */
  if (fabsf(tracker_channel->carrier_freq) > fMaxDoppler) {
    (tracker_channel->flags) |= TRACKER_FLAG_OUTLIER;
    return;
  }
}

/**
 * Runs false lock detection logic for PLL.
 *
 * \param[in,out] tracker_channel Tracker channel data
 * \param[in]     cycle_flags  Current cycle flags.
 *
 * \return None
 */
void tp_tracker_update_alias(tracker_channel_t *tracker_channel,
                             u32 cycle_flags) {
  bool do_first = 0 != (cycle_flags & TP_CFLAG_ALIAS_FIRST);

  /* Attempt alias detection if we have pessimistic phase lock detect, OR
     (optimistic phase lock detect AND are in second-stage tracking) */
  if (0 != (cycle_flags & TP_CFLAG_ALIAS_SECOND)) {
    if (tracker_channel->use_alias_detection &&
        tracker_channel->lock_detect.outo) {
      process_alias_error(tracker_channel);
    } else {
      /* If second stage is not enabled, make the first one */
      do_first = true;
    }
  }

  if (do_first) {
    alias_detect_first(&tracker_channel->alias_detect,
                       tracker_channel->corrs.corr_ad.I,
                       tracker_channel->corrs.corr_ad.Q);
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
void tp_tracker_filter_doppler(tracker_channel_t *tracker_channel,
                               u32 cycle_flags,
                               const tp_tracker_config_t *config) {
  if (0 != (cycle_flags & TP_CFLAG_BSYNC_UPDATE) &&
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
void tp_tracker_update_mode(tracker_channel_t *tracker_channel) {
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
u32 tp_tracker_update(tracker_channel_t *tracker_channel,
                      const tp_tracker_config_t *config) {
  /*
   * State machine control: control is a combination of actions permitted by
   * the tracker state and flags specific for current cycle.
   */
  u32 cflags = tp_get_cycle_flags(tracker_channel, tracker_channel->cycle_no);

  tp_tracker_update_correlators(tracker_channel, cflags);
  tp_tracker_update_bsync(tracker_channel, cflags);
  tp_tracker_update_cn0(tracker_channel, cflags);
  tp_tracker_update_locks(tracker_channel, cflags);
  tp_tracker_update_fll(tracker_channel, cflags);
  tp_tracker_update_pll_dll(tracker_channel, cflags);
  tp_tracker_flag_outliers(tracker_channel);
  tp_tracker_update_alias(tracker_channel, cflags);
  tp_tracker_filter_doppler(tracker_channel, cflags, config);
  tp_tracker_update_mode(tracker_channel);

  u32 chips_to_correlate = tp_tracker_compute_rollover_count(tracker_channel);
  tracker_retune(tracker_channel, chips_to_correlate);

  tp_tracker_update_cycle_counter(tracker_channel);

  return cflags;
}
