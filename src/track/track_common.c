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

#include <inttypes.h>
#include <swiftnav/bits.h>
#include <swiftnav/constants.h>
#include <swiftnav/gnss_time.h>

#include "lock_detector/lock_detector.h"
#include "manage.h"
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
  u32 current_chip = rint(code_phase);

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
 * \param[in,out] tracker Tracker channel data.
 * \param[in]     init        Flag to indicate if the call to initialize or
 *                            to update.
 * \return None
 */
void tp_tracker_update_lock_detect_parameters(tracker_t *tracker, bool init) {
  tp_profile_t *profile = &tracker->profile;
  const tp_lock_detect_params_t *ldp = &profile->ld_phase_params;
  const tp_lock_detect_params_t *ldf = &profile->ld_freq_params;

  if (init) {
    pll_lock_detect_init(
        &tracker->ld_phase, ldp->k1, ldp->k2, ldp->lp, /*lo=*/0);
    fll_lock_detect_init(
        &tracker->ld_freq, ldf->k1, ldf->k2, ldf->lp, /*lo=*/0);
  } else {
    lock_detect_reinit(&tracker->ld_phase, ldp->k1, ldp->k2, ldp->lp, /*lo=*/0);
    lock_detect_reinit(&tracker->ld_freq, ldf->k1, ldf->k2, ldf->lp, /*lo=*/0);
  }
}

/**
 * Update tracker parameters when initializing or changing tracking mode.
 *
 * \param[in,out]  tracker Tracker channel data.
 * \param[in]     init         Flag to indicate if the call to initialize or
 *                             to update.
 *
 * \return None
 */
void tp_profile_apply_config(tracker_t *tracker, bool init) {
  me_gnss_signal_t mesid = tracker->mesid;
  tp_profile_t *profile = &tracker->profile;

  const tp_loop_params_t *l = &profile->loop_params;
  bool prev_use_alias_detection = 0;

  if (!init) {
    prev_use_alias_detection =
        (0 != (tracker->flags & TRACKER_FLAG_USE_ALIAS_DETECTION));
  }

  tracker->tracking_mode = profile->loop_params.mode;

  if (profile->use_alias_detection) {
    tracker->flags |= TRACKER_FLAG_USE_ALIAS_DETECTION;
  } else {
    tracker->flags &= ~TRACKER_FLAG_USE_ALIAS_DETECTION;
  }

  tracker->has_next_params = false;

  u16 cycle_no = 0;
  if (!init) {
    cycle_no = tp_calc_init_cycle_no(tracker, l->mode, /*switch_in_ms=*/0);
    /* compensate for tp_tracker_update_cycle_counter() call */
    cycle_no = cycle_no ? (cycle_no - 1) : (tp_get_cycle_count(l->mode) - 1);
  }
  tracker->cycle_no = cycle_no;

  /**< C/N0 integration time */
  u8 cn0_ms = tp_get_cn0_ms(tracker->tracking_mode);
  /**< Set initial rates */
  tl_rates_t rates;
  rates.code_freq = tracker->code_phase_rate - code_to_chip_rate(mesid.code);
  rates.carr_freq = tracker->carrier_freq;
  rates.acceleration = 0.0f;
  /**< Set tracking loop configuration parameters */
  tl_config_t config;
  tp_tl_get_config(l, &config);
  /* DLL discriminator period is same as code_loop_period_s */
  config.code_loop_period_s =
      tp_get_dll_ms(tracker->tracking_mode) / (float)SECS_MS;
  config.carr_loop_period_s =
      tp_get_fpll_ms(tracker->tracking_mode) / (float)SECS_MS;
  config.fll_discr_period_s =
      tp_get_flld_ms(tracker->tracking_mode) / (float)SECS_MS;

  /* DLL init could be done nicer by initing DLL only */
  if (init || profile->dll_init) {
    log_debug_mesid(mesid, "Initializing TL");

    tp_tl_init(&tracker->tl_state, profile->loop_params.ctrl, &rates, &config);
  } else {
    log_debug_mesid(mesid, "Re-tuning TL");

    /* Recalculate filter coefficients */
    tp_tl_retune(&tracker->tl_state, profile->loop_params.ctrl, &config);
  }
  tp_tracker_update_lock_detect_parameters(tracker, init);

  tracker->flags &= ~TRACKER_FLAG_PLL_USE;
  tracker->flags &= ~TRACKER_FLAG_FLL_USE;
  if (profile->loop_params.pll_bw > 0) {
    tracker->flags |= TRACKER_FLAG_PLL_USE;
  }
  if (profile->loop_params.fll_bw > 0) {
    tracker->flags |= TRACKER_FLAG_FLL_USE;
  }

  if (init) {
    /* Initialize C/N0 estimator and filter */
    track_cn0_init(&tracker->cn0_est, /* C/N0 estimator state */
                   cn0_ms,            /* C/N0 period in ms */
                   tracker->cn0);     /* Initial C/N0 value */

    log_debug_mesid(mesid, "CN0 init: %f", tracker->cn0);
  }

  bool use_alias_detection =
      (0 != (tracker->flags & TRACKER_FLAG_USE_ALIAS_DETECTION));
  if (use_alias_detection) {
    float alias_detect_ms = tp_get_alias_ms(tracker->tracking_mode);

    if (prev_use_alias_detection) {
      alias_detect_reinit(&tracker->alias_detect,
                          (u32)(TP_TRACKER_ALIAS_DURATION_MS / alias_detect_ms),
                          alias_detect_ms * 1e-3f);
    } else {
      alias_detect_init(&tracker->alias_detect,
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
 * \param[in]     tracker Tracker channel data
 * \param[in]     config       Configuration parameters.
 *
 * \return None
 */
void tp_tracker_init(tracker_t *tracker, const tp_tracker_config_t *config) {
  me_gnss_signal_t mesid = tracker->mesid;

  tracker_ambiguity_unknown(tracker);

  log_debug_mesid(
      mesid, "[+%" PRIu32 "ms] Tracker start", tracker->update_count);

  /* Do tracking report to manager */
  tp_report_t report;
  memset(&report, 0, sizeof(report));
  report.cn0 = tracker->cn0;

  tp_profile_init(tracker, &report);

  if (config->confirm_early) {
    tracker->flags |= TRACKER_FLAG_CONFIRMED;
  }

  tp_profile_apply_config(tracker, /* init = */ true);

  tracker->flags |= TRACKER_FLAG_ACTIVE;
}

void tracker_cleanup(tracker_t *tracker) {
  size_t cleanup_region_size =
      sizeof(tracker_t) - offsetof(tracker_t, cleanup_region_start);
  memset(&tracker->cleanup_region_start, 0, cleanup_region_size);
}

/**
 * Releases tracker data.
 *
 * The method releases tracker state.
 *
 * \param[in] tracker Tracker channel data
 */
void tp_tracker_disable(tracker_t *tracker) {
  log_debug_mesid(tracker->mesid,
                  "[+%" PRIu32 "ms] Tracker stop TOW=%" PRId32 "ms",
                  tracker->update_count,
                  tracker->TOW_ms);

  /* restore acq for this tracked SV */
  restore_acq(tracker);

  /* final cleanup */
  tracker_cleanup(tracker);
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
 * \param[in] tracker Tracker channel data
 *
 * \return Computed number of chips.
 */
static u32 tp_tracker_compute_rollover_count(tracker_t *tracker) {
  u8 result_ms;
  tp_tm_e mode_cur = tracker->tracking_mode;
  if (tracker->has_next_params) {
    tp_profile_t *profile = &tracker->profile;
    tp_tm_e mode_next = tp_profile_get_next_track_mode(profile, tracker->mesid);
    u16 cycle_no = tp_wrap_cycle(mode_cur, tracker->cycle_no + 1);
    u8 ms = tp_get_cycle_duration(mode_cur, cycle_no);
    cycle_no = tp_calc_init_cycle_no(tracker, mode_next, /*switch_in_ms=*/ms);
    result_ms = tp_get_cycle_duration(mode_next, cycle_no);
  } else {
    u16 cycle_no = tp_wrap_cycle(mode_cur, tracker->cycle_no + 2);
    result_ms = tp_get_cycle_duration(mode_cur, cycle_no);
  }
  assert(result_ms);

  double code_phase_chips = tracker->code_phase_prompt;
  bool plock = ((0 != (tracker->flags & TRACKER_FLAG_HAS_PLOCK)) ||
                (0 != (tracker->flags & TRACKER_FLAG_HAS_FLOCK)));

  return tp_convert_ms_to_chips(
      tracker->mesid, result_ms, code_phase_chips, plock);
}

/**
 * Initialize profile switching operation.
 *
 * Method checks if the profile switching is possible and required. The profile
 * switching is possible only when the bit boundary is reached, thus the check
 * is performed when the interval processed by FPGA is the last bit interval,
 * or, in other words, the next cycle has a bit sync flag and closes the bit.
 *
 * \param tracker Tracker channel data
 *
 * \return None
 */
static void mode_change_init(tracker_t *tracker) {
  bool confirmed = (0 != (tracker->flags & TRACKER_FLAG_CONFIRMED));
  if (!confirmed) {
    return;
  }

  /* Compute time of the currently integrated period */
  u16 next_cycle = tp_wrap_cycle(tracker->tracking_mode, tracker->cycle_no + 1);
  u32 next_cycle_flags = tp_get_cycle_flags(tracker, next_cycle);

  if (0 == (next_cycle_flags & TPF_BSYNC_UPD)) {
    return;
  }

  /* The switch is possible only when bit sync counter is updated: get the
   * bit update interval in ms. */
  u8 bit_ms = tp_get_bit_ms(tracker->tracking_mode);

  if (tracker_next_bit_aligned(tracker, bit_ms)) {
    /* When the bit sync is available and the next integration interval is the
     * last one in the bit, check if the profile switch is required. */
    tracker->has_next_params = tp_profile_has_new_profile(tracker);
  }
}

/**
 * Finish profile switching operation.
 *
 * Method fetches new profile parameters and reconfigures as necessary.
 *
 * \param[in,out] tracker Tracker channel data
 *
 * \return None
 */
static void mode_change_complete(tracker_t *tracker) {
  if (tracker->has_next_params) {
    tp_profile_switch(tracker);
    tp_profile_update_config(tracker);
    tp_profile_apply_config(tracker, /* init = */ false);
  }
}

/**
 * Controls TL operation steps.
 *
 * The method updates the TL step according to tracking mode.
 *
 * \param tracker Tracker channel data
 *
 * \return None
 */
static void tp_tracker_update_cycle_counter(tracker_t *tracker) {
  tracker->cycle_no =
      tp_wrap_cycle(tracker->tracking_mode, tracker->cycle_no + 1);
}

/**
 * Second stage of false lock detection.
 *
 * Detect frequency error and update tracker state as appropriate.
 *
 * \param[in] tracker Tracker channel data
 * \param[in] I in-phase signal component
 * \param[in] Q quadrature signal component
 *
 * \return None
 */
static void process_alias_error(tracker_t *tracker, float I, float Q) {
  float err_hz = alias_detect_second(&tracker->alias_detect, I, Q);

  if (fabsf(err_hz) < TRACK_ALIAS_THRESHOLD_HZ) {
    return;
  }

  /* The expected frequency errors depend on the modulated data rate.
     For more details on GPS, see:
     https://swiftnav.hackpad.com/Alias-PLL-lock-detector-in-L2C-4fWUJWUNnOE
     However in practice we see alias frequencies also in between the expected
     ones. So let's just correct the error we actually observe. */

  bool plock = (0 != (tracker->flags & TRACKER_FLAG_HAS_PLOCK));
  bool flock = (0 != (tracker->flags & TRACKER_FLAG_HAS_FLOCK));
  log_warn_mesid(tracker->mesid,
                 "False freq detected: %.1f Hz (plock=%d,flock=%d)",
                 err_hz,
                 (int)plock,
                 (int)flock);

  tracker_ambiguity_unknown(tracker);
  /* alias_detect_second() returns the freq correction value directly.
     So we can use it as is for the adjustment. */
  tp_tl_adjust(&tracker->tl_state, err_hz);
}

static void add_pilot_and_data_iq(tp_epl_corr_t *cs_now) {
  corr_t *all = cs_now->all;
  corr_t *data = &all[3];
  corr_t *pilot = &all[0];

  /* In base station mode we combine pilot and data ELP. In rover mode we
     only combine EL. In rover mode we do not want to "borrow" from the phase
     using the data bit polarity as it might be flaky. */

  /* early */
  /* pilot[0].I = ABS(data[0].I) + ABS(pilot[0].I); */
  /* pilot[0].Q = ABS(data[0].Q) + ABS(pilot[0].Q); */

  /* prompt */
  /* if (tp_is_base_station_mode()) { */
    /* non-normalized dot product using data and pilot prompt IQ data */
    if ((data[1].I * pilot[1].I + data[1].Q * pilot[1].Q) > 0) {
      pilot[1].I += data[1].I; /* wipe-off data bits */
      pilot[1].Q += data[1].Q;
    } else {
      /* bit flip */
      pilot[1].I -= data[1].I; /* wipe-off data bits */
      pilot[1].Q -= data[1].Q;
    }
  /* } */

  /* late */
  /* pilot[2].I = ABS(data[2].I) + ABS(pilot[2].I); */
  /* pilot[2].Q = ABS(data[2].Q) + ABS(pilot[2].Q); */
}

/**
 * Updates tracker correlators.
 *
 * Updates tracker correlators with data read from FPGA according to current
 * cycle flags.
 *
 * \param[in]     tracker Tracker channel data
 * \param[in]     cycle_flags  Current cycle flags.
 */
static void tp_tracker_update_correlators(tracker_t *tracker, u32 cycle_flags) {
  me_gnss_signal_t mesid = tracker->mesid;
  tp_epl_corr_t cs_now;     /**< Correlations from FPGA */
  u32 sample_count;         /**< Sample count from FPGA */
  double code_phase_prompt; /**< Code phase from FPGA */
  double carrier_phase;     /**< Carrier phase from FPGA */
  u8 int_ms = 0;            /**< Current cycle duration in ms */

  /* Read correlations. */
  tracker_correlations_read(tracker->nap_channel,
                            cs_now.all,
                            &sample_count,
                            &code_phase_prompt,
                            &carrier_phase);

  if ((CODE_GAL_E5I == mesid.code) || (CODE_GAL_E7I == mesid.code)) {
    /* for Galileo E5a and E5b all tracking happens on the pilot
     * and when sync is achieved on the SC100 (Prompt)
     * the data can be extracted on the 5th correlator.
     * This is taken care by the flag TPF_BIT_PILOT.
     * However, as they have the pilot in quadrature,
     * one needs to apply a 90 deg rotation
     * before setting/accumulating the navigation data bit */
    corr_t temp = cs_now.dp_prompt;
    cs_now.dp_prompt.I = -temp.Q;
    cs_now.dp_prompt.Q = temp.I;

    /* Do the rotation for early and late for DLL discriminator */
    temp = cs_now.dp_early;
    cs_now.dp_early.I = -temp.Q;
    cs_now.dp_early.Q = temp.I;

    temp = cs_now.dp_late;
    cs_now.dp_late.I = -temp.Q;
    cs_now.dp_late.Q = temp.I;
  }

  /* Signals with pilot codes have data on the 5th correlator */
  bool has_pilot_sync = nap_sc_wipeoff(tracker);
  if ((CODE_GPS_L2CM == mesid.code) || has_pilot_sync) {
    cycle_flags |= TPF_BIT_PILOT;
    add_pilot_and_data_iq(&cs_now);
  }

  tp_update_correlators(cycle_flags, &cs_now, &tracker->corrs);

  /* Current cycle duration */
  int_ms = tp_get_cycle_duration(tracker->tracking_mode, tracker->cycle_no);

  if (tracker->updated_once) {
    u64 delay_ms = tracker_timer_ms(&tracker->update_timer);
    if (delay_ms > NAP_CORR_LENGTH_MAX_MS) {
      log_warn_mesid(mesid,
                     "Unexpected tracking channel update rate: %" PRIu64 " ms",
                     delay_ms);
    }
  }
  tracker->updated_once = true;
  tracker_timer_arm(&tracker->update_timer, /*deadline_ms=*/-1); /*re-arm*/

  tracker->sample_count = sample_count;
  tracker->code_phase_prompt = code_phase_prompt;
  tracker->carrier_phase = carrier_phase;

  /* ToW update:
   * ToW along with carrier and code phases and sample number and health
   * shall be updated in sync.
   */
  bool decoded_tow;
  tracker->TOW_ms_prev = tracker->TOW_ms;
  tracker->TOW_ms = tracker_tow_update(tracker,
                                       tracker->TOW_ms,
                                       int_ms,
                                       &tracker->TOW_residual_ns,
                                       &decoded_tow);

  if (!tp_tow_is_sane(tracker->TOW_ms)) {
    log_error_mesid(mesid,
                    "[+%" PRIu32 "ms] Error TOW from decoder %" PRId32 "(%s)",
                    tracker->update_count,
                    tracker->TOW_ms,
                    decoded_tow ? "decoded" : "propagated");

    tracker->TOW_ms = TOW_UNKNOWN;
    tracker->flags &= ~TRACKER_FLAG_TOW_DECODED;
    /* Still trust TRACKER_FLAG_GLO_HEALTH_DECODED, if it is set */
  }

  if (decoded_tow) {
    log_debug_mesid(mesid,
                    "[+%" PRIu32 "ms] Decoded TOW %" PRId32,
                    tracker->update_count,
                    tracker->TOW_ms);

    if (tracker->TOW_ms != TOW_UNKNOWN) {
      tracker->flags |= TRACKER_FLAG_TOW_DECODED;
    }
  }

  if (TOW_UNKNOWN == tracker->TOW_ms) {
    tracker->flags &= ~TRACKER_FLAG_TOW_VALID;
  } else {
    tracker->flags |= TRACKER_FLAG_TOW_VALID;
  }

  /* Channel run time. */
  tracker->update_count += int_ms;
}

/**
 * Updates bit synchronization and decodes message
 *
 * \param[in]     tracker Tracker channel data
 * \param[in]     cycle_flags  Current cycle flags.
 *
 * \return None
 */
static void tp_tracker_update_bsync(tracker_t *tracker, u32 cycle_flags) {
  if (0 != (cycle_flags & TPF_BSYNC_UPD)) {
    bool sensitivity_mode =
        (0 != (tracker->flags & TRACKER_FLAG_SENSITIVITY_MODE));
    /* Bit sync / data decoding update counter. */
    u8 int_ms = tp_get_bit_ms(tracker->tracking_mode);
    /* Bit sync advance / message decoding */
    tracker_bit_sync_update(tracker, int_ms, sensitivity_mode);

    /* TODO Update BS from ToW when appropriate. */
    /* TODO Add fast BS detection. */
    /* TODO Add bad BS recovery. */
  }
}

/**
 * Updates C/N0 estimators.
 *
 * \param[in]     tracker Tracker channel data
 * \param[in]     cycle_flags  Current cycle flags.
 *
 * \return None
 */
static void tp_tracker_update_cn0(tracker_t *tracker, u32 cycle_flags) {
  float cn0 = tracker->cn0_est.filter.yn;
  float cn0_prev = cn0;
  tp_cn0_thres_t cn0_thres;
  tp_profile_get_cn0_thres(&tracker->profile, &cn0_thres);

  if (0 != (cycle_flags & TPF_CN0_USE)) {
    /* Workaround for
     * https://github.com/swift-nav/piksi_v3_bug_tracking/issues/475
     * don't update c/n0 if correlators data are 0 use
     * last post-filter sample instead */
    if (0 == tracker->corrs.corr_cn0.I && 0 == tracker->corrs.corr_cn0.Q) {
      log_info_mesid(tracker->mesid,
                     "I/Q: %" PRIi32 "/%" PRIi32 " %" PRIi32 "/%" PRIi32
                     " %" PRIi32 "/%" PRIi32,
                     tracker->corrs.corr_main.early.I,
                     tracker->corrs.corr_main.early.Q,
                     tracker->corrs.corr_main.prompt.I,
                     tracker->corrs.corr_main.prompt.Q,
                     tracker->corrs.corr_main.late.I,
                     tracker->corrs.corr_main.late.Q);

    } else {
      /* Update C/N0 estimate */
      u8 cn0_ms = tp_get_cn0_ms(tracker->tracking_mode);
      cn0 = track_cn0_update(&tracker->cn0_est,
                             cn0_ms,
                             tracker->corrs.corr_cn0.I,
                             tracker->corrs.corr_cn0.Q);
    }
  }

  if ((cn0 < cn0_thres.drop_dbhz) && (cn0_prev > cn0_thres.drop_dbhz)) {
    tracker_timer_arm(&tracker->cn0_below_drop_thres_timer, /*deadline_ms=*/-1);
  } else if (cn0 >= cn0_thres.drop_dbhz) {
    tracker_timer_init(&tracker->cn0_below_drop_thres_timer);
  }

  bool confirmed = (0 != (tracker->flags & TRACKER_FLAG_CONFIRMED));
  bool inlock = tracker_has_all_locks(tracker);

  if (cn0 > cn0_thres.drop_dbhz && !confirmed && inlock &&
      tracker_has_bit_sync(tracker)) {
    tracker->flags |= TRACKER_FLAG_CONFIRMED;
    log_debug_mesid(tracker->mesid, "CONFIRMED with CN0: %f", cn0);
  }

  tracker->cn0 = cn0;

  if (cn0 < cn0_thres.ambiguity_dbhz) {
    /* C/N0 has dropped below threshold, indicate that the carrier phase
     * ambiguity is now unknown as cycle slips are likely. */
    tracker_ambiguity_unknown(tracker);
  }

  if (cn0 < cn0_thres.use_dbhz) {
    /* Flag as low CN0 measurements. */
    tracker->flags &= ~TRACKER_FLAG_CN0_USABLE;
  }

  if (cn0 > (cn0_thres.use_dbhz + TRACK_CN0_HYSTERESIS_THRES_DBHZ)) {
    /* Flag as high CN0 measurements. */
    tracker->flags |= TRACKER_FLAG_CN0_USABLE;
  }
}

static void update_ld_phase(tracker_t *tracker, u32 cycle_flags) {
  bool pll_in_use = (0 != (tracker->flags & TRACKER_FLAG_PLL_USE));
  if (!pll_in_use) {
    const tp_lock_detect_params_t *ldp = &tracker->profile.ld_phase_params;
    pll_lock_detect_init(
        &tracker->ld_phase, ldp->k1, ldp->k2, ldp->lp, /*lo=*/0);
    tracker->flags &= ~TRACKER_FLAG_HAS_PLOCK;
    return;
  }

  bool plock_update_needed = (0 != (cycle_flags & TPF_PLD_USE));
  if (!plock_update_needed) {
    return;
  }

  tracker->flags &= ~TRACKER_FLAG_HAS_PLOCK;

  bool last_outp = tracker->ld_phase.outp;

  pll_lock_detect_update(&tracker->ld_phase,
                         tracker->corrs.corr_ld.I,
                         tracker->corrs.corr_ld.Q,
                         tp_get_ld_ms(tracker->tracking_mode));

  bool outp = tracker->ld_phase.outp;

  if (outp) {
    tracker->flags |= TRACKER_FLAG_HAS_PLOCK;
    tracker->flags |= TRACKER_FLAG_HAD_PLOCK;
  } else if (last_outp && (TP_TM_INITIAL != tracker->tracking_mode)) {
    log_info_mesid(tracker->mesid, "PLL stress");
  }
}

static void update_ld_freq(tracker_t *tracker, u32 cycle_flags) {
  bool fll_in_use = (0 != (tracker->flags & TRACKER_FLAG_FLL_USE));
  if (!fll_in_use) {
    const tp_lock_detect_params_t *ldf = &tracker->profile.ld_freq_params;
    fll_lock_detect_init(
        &tracker->ld_freq, ldf->k1, ldf->k2, ldf->lp, /*lo=*/0);
    tracker->flags &= ~TRACKER_FLAG_HAS_FLOCK;
    return;
  }

  bool flock_update_needed = (0 != (cycle_flags & TPF_FLL_USE));
  if (!flock_update_needed) {
    return;
  }

  tracker->flags &= ~TRACKER_FLAG_HAS_FLOCK;

  /* FLL lock detector is based on frequency error seen by FLL discriminator */
  float unfiltered_freq_error = tracker->unfiltered_freq_error;

  /* Calculate low-pass filtered frequency error */
  fll_lock_detect_update(&tracker->ld_freq, unfiltered_freq_error);

  bool outp = tracker->ld_freq.outp;

  if (outp) {
    tracker->flags |= TRACKER_FLAG_HAS_FLOCK;
    tracker->flags |= TRACKER_FLAG_HAD_FLOCK;
  }
}

/**
 * Updates PLL and FLL lock detectors.
 *
 * \param[in]     tracker Tracker channel data
 * \param[in]     cycle_flags  Current cycle flags.
 *
 * \return None
 */
static void tp_tracker_update_locks(tracker_t *tracker, u32 cycle_flags) {
  bool outp_prev = tracker->ld_phase.outp || tracker->ld_freq.outp;

  update_ld_phase(tracker, cycle_flags);
  update_ld_freq(tracker, cycle_flags);

  bool outp = tracker->ld_phase.outp || tracker->ld_freq.outp;

  bool confirmed = (0 != (tracker->flags & TRACKER_FLAG_CONFIRMED));
  if (!outp_prev && outp && confirmed) {
    u64 unlocked_ms = tracker_timer_ms(&tracker->unlocked_timer);
    log_debug_mesid(tracker->mesid, "Lock after %" PRIu64 "ms", unlocked_ms);
  }

  if (outp != outp_prev) {
    if (outp) {
      tracker_timer_init(&tracker->unlocked_timer);
      tracker_timer_arm(&tracker->locked_timer, /*deadline_ms=*/-1);
    } else {
      tracker_timer_init(&tracker->locked_timer);
      tracker_timer_arm(&tracker->unlocked_timer, /*deadline_ms=*/-1);
    }
  }
  if (outp) {
    tracker->carrier_freq_at_lock = tracker->carrier_freq;
  }
  /*
   * Reset carrier phase ambiguity if there's doubt as to our phase lock.
   * Continue phase ambiguity reset until pessimistic PLL lock is reached. This
   * is done always to prevent incorrect handling of partial integration
   * intervals.
   */
  if (0 == (tracker->flags & TRACKER_FLAG_HAS_PLOCK)) {
    tracker_ambiguity_unknown(tracker);
  }
}

/**
 * Predicate that checks if the given cycle is decimated
 * \param cycle_no cycle index to check (one based)
 * \param decim_factor decimation factor
 * \retval true the given cycle index is decimated
 * \retval false the given cycle index in not decimated (passed through)
 */
static bool cycle_decimated(u16 cycle_no, u8 decim_factor) {
  if ((0 == decim_factor) || (1 == decim_factor)) {
    return false;
  }
  return (0 != (cycle_no % decim_factor));
}

/**
 * Runs FLL, PLL and DLL controller updates.
 *
 * This method updates FLL, PLL and DLL loops and additionally checks for DLL
 * errors and report data to profile managements.
 *
 * \param[in]     tracker Tracker channel data
 * \param[in]     cycle_flags  Current cycle flags.
 *
 * \return None
 */
static void tp_tracker_update_loops(tracker_t *tracker, u32 cycle_flags) {
  if (0 != (cycle_flags & TPF_FLL_USE)) {
    bool halfq = (0 != (cycle_flags & TPF_FLL_HALFQ));
    tp_tl_update_fll_discr(&tracker->tl_state, tracker->corrs.corr_fll, halfq);
    tracker->unfiltered_freq_error = tp_tl_get_fll_error(&tracker->tl_state);
  }

  if (0 != (cycle_flags & TPF_EPL_USE)) {
    tp_epl_corr_t corr_main = tracker->corrs.corr_main;

    /* Output I/Q correlations using SBP if enabled for this channel */
    if (tracker->tracking_mode != TP_TM_INITIAL) {
      tracker_correlations_send(tracker, corr_main.all);
    }

    bool costas = true;
    bool has_pilot_sync = nap_sc_wipeoff(tracker);
    if ((CODE_GPS_L2CM == tracker->mesid.code) || has_pilot_sync) {
      /* The L2CM and L2CL codes are in phase,
       * copy the dp_prompt to prompt so that the PLL
       * runs on the pilot instead of the data */
      /* Once in bit-sync, Galileo pilots are completely free of transitions so
       * no need for a Costas loop*/
      costas = false;
    }

    if (CODE_GAL_E7I == tracker->mesid.code) {
      const corr_t *c = corr_main.all;

      DO_EVERY(2048,
      log_info("CORR: %d,%d,%d,%d,%d,%d, %d,%d,%d,%d,%d,%d",
               (int)c[0].I, (int)c[0].Q,
               (int)c[1].I, (int)c[1].Q,
               (int)c[2].I, (int)c[2].Q,
               (int)c[3].I, (int)c[3].Q,
               (int)c[4].I, (int)c[4].Q,
               (int)c[5].I, (int)c[5].Q););
    }

    tp_tl_update_dll_discr(&tracker->tl_state, &corr_main);
    tp_tl_update_dll(&tracker->tl_state);

    bool run_fpll = (0 != (cycle_flags & TPF_FPLL_RUN));
    if (run_fpll) {
      tracker->fpll_cycle++;
      u8 fpll_decim = tp_get_fpll_decim(tracker->tracking_mode);
      run_fpll = !cycle_decimated(tracker->fpll_cycle, fpll_decim);
      if (run_fpll) {
        tp_tl_update_fpll(&tracker->tl_state, &corr_main, costas);
        tracker->fpll_cycle = 0;
      }
    }

    tl_rates_t rates = {0};
    tp_tl_get_rates(&tracker->tl_state, &rates);

    tracker->carrier_freq = rates.carr_freq;
    tracker->code_phase_rate =
        rates.code_freq + code_to_chip_rate(tracker->mesid.code);

    /* Do tracking report to manager */
    tp_report_t report;
    memset(&report, 0, sizeof(report));
    report.cn0 = tracker->cn0;

    tp_profile_report_data(&tracker->profile, &report);
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
    tracker_flag_drop(tracker, CH_DROP_REASON_OUTLIER);
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

  u64 elapsed_ms = tracker_timer_ms(&tracker->carrier_freq_age_timer);
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
    /* If raw CN0 is high the outliers are likely due to genuine acceleration */
    bool low_cn0 = (tracker->cn0_est.cn0_raw_dbhz < TP_OUTLIERS_CN0_THRES_DBHZ);
    if (low_cn0 && (fabs(diff_hz) > max_diff_hz)) {
      log_debug_mesid(
          tracker->mesid, "Doppler difference %.2f is too high", diff_hz);
      tracker_flag_drop(tracker, CH_DROP_REASON_OUTLIER);
    }

    tracker->carrier_freq_prev = tracker->carrier_freq;
    tracker_timer_arm(&tracker->carrier_freq_age_timer, -1);
  }
}

/**
 * Runs false lock detection logic.
 *
 * \param[in,out] tracker Tracker channel data
 * \param[in]     cycle_flags  Current cycle flags.
 *
 * \return None
 */
static void tp_tracker_update_alias(tracker_t *tracker, u32 cycle_flags) {
  bool use_alias_detection =
      (0 != (tracker->flags & TRACKER_FLAG_USE_ALIAS_DETECTION));
  if (!use_alias_detection) {
    return;
  }

  bool do_first = (0 != (cycle_flags & TPF_ALIAS_1ST));
  bool do_second = (0 != (cycle_flags & TPF_ALIAS_2ND));

  if (!do_first && !do_second) {
    return;
  }

  float I = tracker->corrs.corr_ad.I;
  float Q = tracker->corrs.corr_ad.Q;

  if (do_second) {
    bool inlock = ((0 != (tracker->flags & TRACKER_FLAG_HAS_PLOCK)) ||
                   (0 != (tracker->flags & TRACKER_FLAG_HAS_FLOCK)));
    if (inlock) {
      process_alias_error(tracker, I, Q);
    }
    do_first = true;
  }

  if (do_first) {
    alias_detect_first(&tracker->alias_detect, I, Q);
  }
}

/**
 * Computes filtered SV doppler for cross-correlation and other uses.
 *
 * The method updates SV doppler value on bit edges. The doppler is not taken
 * from
 * controller, but is a LP product of controller output.
 *
 * \param[in,out  tracker Tracker channel data
 * \param[in]     cycle_flags  Current cycle flags.
 * \param[in]     config       Generic tracker configuration.
 *
 * \return None
 */
static void tp_tracker_filter_doppler(tracker_t *tracker,
                                      u32 cycle_flags,
                                      const tp_tracker_config_t *config) {
  if (0 != (cycle_flags & TPF_BSYNC_UPD) && tracker_bit_aligned(tracker)) {
    float xcorr_freq = tracker->carrier_freq;

    if (tracker->flags & TRACKER_FLAG_XCORR_FILTER_ACTIVE) {
      xcorr_freq = lp1_filter_update(
          &tracker->xcorr_filter, &config->xcorr_f_params, xcorr_freq);
    } else {
      lp1_filter_init(
          &tracker->xcorr_filter, &config->xcorr_f_params, xcorr_freq);
      tracker->flags |= TRACKER_FLAG_XCORR_FILTER_ACTIVE;
    }

    tracker->xcorr_freq = xcorr_freq;
  }
}

/**
 * Mode switching control.
 *
 * \param tracker Tracker channel data
 *
 * \return None
 */
static void tp_tracker_update_mode(tracker_t *tracker) {
  mode_change_complete(tracker);
  mode_change_init(tracker);
}

/**
 * Default tracking loop.
 *
 * \param tracker Tracking channel data.
 * \param config       Generic tracker configuration.
 *
 * \return Flags, used in the current tracking cycle.
 */
u32 tp_tracker_update(tracker_t *tracker, const tp_tracker_config_t *config) {
  /*
   * State machine control: control is a combination of actions permitted by
   * the tracker state and flags specific for current cycle.
   */
  u32 cflags = tp_get_cycle_flags(tracker, tracker->cycle_no);

  tp_tracker_update_correlators(tracker, cflags);
  tp_tracker_update_bsync(tracker, cflags);
  tp_tracker_update_cn0(tracker, cflags);
  tp_tracker_update_loops(tracker, cflags);
  tp_tracker_update_locks(tracker, cflags);
  tp_tracker_flag_outliers(tracker);
  tp_tracker_update_alias(tracker, cflags);
  tp_tracker_filter_doppler(tracker, cflags, config);
  tp_tracker_update_mode(tracker);

  u32 chips_to_correlate = tp_tracker_compute_rollover_count(tracker);
  tracker_retune(tracker, chips_to_correlate);

  tp_tracker_update_cycle_counter(tracker);

  return cflags;
}

static bool tow_is_bit_aligned(tracker_t *tracker) {
  me_gnss_signal_t mesid = tracker->mesid;
  u8 bit_length = tracker_bit_length_get(tracker);

  if (tracker->TOW_ms == TOW_UNKNOWN) {
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
  u8 tail = tracker->TOW_ms % bit_length;
  if (0 != tail) {
    /* If this correction is needed, then there is something wrong
       either with the TOW cache update or with the bit sync */
    s8 error_ms = tail < (bit_length >> 1) ? -tail : bit_length - tail;

    log_error_mesid(mesid,
                    "[+%" PRIu32
                    "ms] TOW error detected: "
                    "error=%" PRId8 "ms old_tow=%" PRId32,
                    tracker->update_count,
                    error_ms,
                    tracker->TOW_ms);

    /* This is rude, but safe. Do not expect it to happen normally. */
    tracker_flag_drop(tracker, CH_DROP_REASON_OUTLIER);
    return false;
  }
  return true;
}

static bool should_update_tow_cache(const tracker_t *tracker) {
  me_gnss_signal_t mesid = tracker->mesid;

  bool confirmed = (0 != (tracker->flags & TRACKER_FLAG_CONFIRMED));
  bool cn0_ok = (tracker->cn0 >= CN0_TOW_CACHE_THRESHOLD);
  bool tow_is_known = (TOW_UNKNOWN != tracker->TOW_ms);
  bool responsible_for_update = false;

  if ((CODE_GPS_L1CA == mesid.code) || (CODE_GLO_L1OF == mesid.code) ||
      (CODE_SBAS_L1CA == mesid.code) || (CODE_QZS_L1CA == mesid.code) ||
      (CODE_BDS2_B1 == mesid.code) || (CODE_GAL_E1B == mesid.code) ||
      (CODE_GAL_E7I == mesid.code)) {
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
      mesid_L1 = construct_mesid(CODE_BDS2_B1, mesid.sat);
    } else {
      assert(!"Unsupported TOW cache code");
    }
    tracker_t *trk_ch = tracker_get_by_mesid(mesid_L1);
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
 * \param[in,out] tracker Tracker channel data
 *
 * \return None
 */
void tracker_tow_cache(tracker_t *tracker) {
  /* If TOW is unknown, check if a valid cached TOW is available. */
  if (TOW_UNKNOWN == tracker->TOW_ms) {
    propagate_tow_from_sid_db(tracker);
  }

  /* Check that tracker has valid TOW. */
  if (!tow_is_bit_aligned(tracker)) {
    return;
  }

  /* Check if a tracker should update TOW cache. */
  if (should_update_tow_cache(tracker)) {
    update_tow_in_sid_db(tracker);
  }
}
