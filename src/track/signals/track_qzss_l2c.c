/*
 * Copyright (C) 2017, 2020 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

/* Local headers */
#include "track_qzss_l2c.h"

#include "signal_db/signal_db.h"
#include "track/track_api.h"
#include "track/track_common.h"
#include "track/track_interface.h"
#include "track/track_sid_db.h"
#include "track/track_utils.h"

/* Non-local headers */
#include <acq/manage.h>

/* Libraries */
#include <swiftnav/constants.h>
#include <swiftnav/logging.h>
#include <swiftnav/signal.h>

/* STD headers */
#include <assert.h>
#include <string.h>

/** QZSS L2C configuration section name */
#define QZSS_L2C_TRACK_SETTING_SECTION "qzss_l2c_track"
#define NUM_COH_L2C_20MS_SYMB 10

/** QZSS L2C configuration container */
static tp_tracker_config_t qzss_l2c_config = TP_TRACKER_DEFAULT_CONFIG;

/* Forward declarations of interface methods for QZSS L2C */
static tracker_interface_function_t tracker_qzss_l2c_init;
static tracker_interface_function_t tracker_qzss_l2c_update;

/** QZSS L2C tracker interface */
static const tracker_interface_t tracker_interface_qzss_l2c = {
    .code = CODE_QZS_L2CM,
    .init = tracker_qzss_l2c_init,
    .disable = tp_tracker_disable,
    .update = tracker_qzss_l2c_update,
};

/** Register L2 CM tracker into the the tracker interface & settings
 *  framework.
 */
void track_qzss_l2c_register(void) {
  lp1_filter_compute_params(&qzss_l2c_config.xcorr_f_params,
                            qzss_l2c_config.xcorr_cof,
                            SECS_MS / QZS_L2C_SYMBOL_LENGTH_MS);

  tracker_interface_register(&tracker_interface_qzss_l2c);
}

/** Do L1CA to L2C handover.
 *
 * The condition for the handover is that TOW must be known.
 *
 * \param[in] sample_count NAP sample count
 * \param[in] sat          Satellite ID
 * \param[in] code_phase   code phase [chips]
 * \param[in] doppler_hz   Doppler [Hz]
 * \param[in] cn0_init     CN0 estimate [dB-Hz]
 * \param[in] TOW_ms       Latest decoded TOW [ms]
 */
void qzss_l1ca_to_l2c_handover(u32 sample_count,
                               u16 sat,
                               double code_phase,
                               double doppler_hz,
                               float cn0_init,
                               s32 TOW_ms) {
  /* compose L2CM MESID: same SV, but code is L2CM */
  me_gnss_signal_t mesid = construct_mesid(CODE_QZS_L2CM, sat);

  if (!tracking_startup_ready(mesid)) {
    log_debug_mesid(mesid, "already in track");
    return; /* L2C signal from the SV is already in track */
  }

  if (!handover_valid(code_phase, QZS_L1CA_CHIPS_NUM)) {
    log_warn_mesid(
        mesid, "Unexpected L1CA to L2C hand-over code phase: %f", code_phase);
    return;
  }

  /* L2CL code starts every 1.5 seconds. Offset must be taken into account. */
  s32 offset_ms = (TOW_ms % QZS_L2CL_PRN_PERIOD_MS);
  u32 code_length = code_to_chip_count(CODE_QZS_L2CL);
  u32 chips_in_ms = code_length / QZS_L2CL_PRN_PERIOD_MS;

  if (code_phase > (QZS_L1CA_CHIPS_NUM - HANDOVER_CODE_PHASE_THRESHOLD)) {
    if (offset_ms == 0) {
      code_phase = QZS_L2CL_CHIPS_NUM - (QZS_L1CA_CHIPS_NUM - code_phase);
    } else {
      code_phase = offset_ms * chips_in_ms - (QZS_L1CA_CHIPS_NUM - code_phase);
    }
  } else {
    code_phase += offset_ms * chips_in_ms;
  }

  tracking_startup_params_t startup_params = {
      .mesid = mesid,
      .sample_count = sample_count,
      /* recalculate doppler freq for L2 from L1 */
      .doppler_hz = (float)(doppler_hz * QZS_L2_HZ / QZS_L1_HZ),
      .code_phase = code_phase,
      /* chips to correlate during first 1 ms of tracking */
      .chips_to_correlate = (u32)lrint(code_to_chip_rate(mesid.code) * 1e-3),
      /* get initial cn0 from parent L1CA channel */
      .cn0_init = cn0_init};

  switch (tracking_startup_request(&startup_params)) {
    case 0:
      log_debug_mesid(mesid, "L2C handover done");
      break;

    case 1:
      /* sat is already in fifo, no need to inform */
      break;

    case 2:
      log_warn_mesid(mesid, "Failed to start L2C tracking");
      break;

    default:
      assert(!"Unknown code returned");
      break;
  }
}

static void tracker_qzss_l2c_init(tracker_t *tracker) {
  tp_tracker_init(tracker, &qzss_l2c_config);

  /* L2C bit sync is known once we start tracking it since
     the L2C ranging code length matches the bit length (20ms).
     This is the end of 20ms integration period and the edge
     of a data bit. */
  tracker_bit_sync_set(tracker, /* bit_phase_ref = */ 0);
}

/**
 * Performs ToW caching and propagation.
 *
 * QZSS L1 C/A and L2 C use shared structure for ToW caching. When QZSS L1 C/A
 * tracker is running, it is responsible for cache updates. Otherwise QZSS L2 C
 * tracker updates the cache. The time difference between signals is ignored
 * as small.
 *
 * QZSS L2 C tracker performs ToW update/propagation only on bit edge. This
 * makes
 * it more robust to propagation errors.
 *
 * \param[in] tracker     Tracker channel data
 * \param[in] cycle_flags Current cycle flags.
 *
 * \return None
 */
static void update_tow_qzss_l2c(tracker_t *tracker, u32 cycle_flags) {
  tp_tow_entry_t tow_entry;
  me_gnss_signal_t mesid = tracker->mesid;
  gnss_signal_t sid = construct_sid(mesid.code, mesid.sat);
  track_sid_db_load_tow(sid, &tow_entry);

  u64 sample_time_tk = nap_sample_time_to_count(tracker->sample_count);

  if (0 != (cycle_flags & TPF_BSYNC_UPD) && tracker_bit_aligned(tracker)) {
    if (TOW_UNKNOWN != tracker->TOW_ms) {
      /*
       * Verify ToW alignment
       * Current block assumes the bit sync has been reached and current
       * interval has closed a bit interval. ToW shall be aligned by bit
       * duration, which is 20ms for QZSS L1 C/A / L2 C.
       */
      u8 tail = tracker->TOW_ms % QZS_L2C_SYMBOL_LENGTH_MS;
      if (0 != tail) {
        s8 error_ms = tail < (QZS_L2C_SYMBOL_LENGTH_MS >> 1)
                          ? -tail
                          : QZS_L2C_SYMBOL_LENGTH_MS - tail;

        log_error_mesid(mesid,
                        "[+%" PRIu64
                        "ms] TOW error detected: "
                        "error=%" PRId8 "ms old_tow=%" PRId32,
                        tracker_timer_ms(&tracker->age_timer),
                        error_ms,
                        tracker->TOW_ms);

        /* This is rude, but safe. Do not expect it to happen normally. */
        tracker_flag_drop(tracker, CH_DROP_REASON_OUTLIER);
      }
    }

    if (TOW_UNKNOWN == tracker->TOW_ms && TOW_UNKNOWN != tow_entry.TOW_ms) {
      /* ToW is not known, but there is a cached value */
      s32 ToW_ms = TOW_UNKNOWN;
      double error_ms = 0;
      u64 time_delta_tk = sample_time_tk - tow_entry.sample_time_tk;
      u8 bit_length = tracker_bit_length_get(tracker);
      ToW_ms = tp_tow_compute(
          tow_entry.TOW_ms, time_delta_tk, bit_length, &error_ms);

      if (TOW_UNKNOWN != ToW_ms) {
        log_debug_mesid(mesid,
                        "[+%" PRIu64
                        "ms]"
                        " Initializing TOW from cache [%" PRIu8
                        "ms] "
                        "delta=%.2lfms ToW=%" PRId32 "ms error=%lf",
                        tracker_timer_ms(&tracker->age_timer),
                        bit_length,
                        nap_count_to_ms(time_delta_tk),
                        ToW_ms,
                        error_ms);
        tracker->TOW_ms = ToW_ms;
        if (tp_tow_is_sane(tracker->TOW_ms)) {
          tracker->flags |= TRACKER_FLAG_TOW_VALID;
        } else {
          log_error_mesid(mesid,
                          "[+%" PRIu64 "ms] Error TOW propagation %" PRId32,
                          tracker_timer_ms(&tracker->age_timer),
                          tracker->TOW_ms);
          tracker->TOW_ms = TOW_UNKNOWN;
          tracker->flags &= ~TRACKER_FLAG_TOW_VALID;
        }
      }
    }

    bool confirmed = (0 != (tracker->flags & TRACKER_FLAG_CONFIRMED));
    if ((TOW_UNKNOWN != tracker->TOW_ms) &&
        (tracker->cn0 >= CN0_TOW_CACHE_THRESHOLD) && confirmed &&
        !mesid_is_tracked(construct_mesid(CODE_QZS_L1CA, mesid.sat))) {
      /* Update ToW cache:
       * - bit edge is reached
       * - CN0 is OK
       * - Tracker is confirmed
       * - There is no QZSS L1 C/A tracker for the same SV.
       */
      tow_entry.TOW_ms = tracker->TOW_ms;
      tow_entry.sample_time_tk = sample_time_tk;
      track_sid_db_update_tow(sid, &tow_entry);
    }
  }
}

static void tracker_qzss_l2c_update(tracker_t *tracker) {
  u32 cflags = tp_tracker_update(tracker, &qzss_l2c_config);

  /* QZSS L2C-specific ToW manipulation */
  update_tow_qzss_l2c(tracker, cflags);

  bool confirmed = (0 != (tracker->flags & TRACKER_FLAG_CONFIRMED));
  bool in_phase_lock = (0 != (tracker->flags & TRACKER_FLAG_HAS_PLOCK));

  if (in_phase_lock && confirmed && tracker_bit_aligned(tracker) &&
      (0 != (cflags & TPF_BSYNC_UPD))) {
    /* naturally synched as we track */
    s8 symb_sign = SIGN(tracker->corrs.corr_main.dp_prompt.I);
    s8 pol_sign = SIGN(tracker->cp_sync.polarity);
    log_debug("J%02d L2C %+2d %+3d",
              tracker->mesid.sat - QZS_FIRST_PRN,
              symb_sign,
              tracker->cp_sync.polarity);
    if (symb_sign != pol_sign) {
      tracker->cp_sync.polarity = symb_sign;
      tracker->cp_sync.synced = false;
    } else {
      tracker->cp_sync.polarity += symb_sign;
      if (NUM_COH_L2C_20MS_SYMB == ABS(tracker->cp_sync.polarity)) {
        tracker->cp_sync.synced = true;
        tracker->cp_sync.polarity -= symb_sign; /* saturate */
      } else {
        tracker->cp_sync.synced = false;
      }
    }
    if (tracker->cp_sync.synced) {
      tracker->bit_polarity = ((tracker->cp_sync.polarity) < 0)
                                  ? BIT_POLARITY_NORMAL
                                  : BIT_POLARITY_INVERTED;
    } else {
      tracker->bit_polarity = BIT_POLARITY_UNKNOWN;
    }
    tracker_update_bit_polarity_flags(tracker);
  }
}
