/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Michele Bavaro <michele@swift-nav.com>
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
#include <manage.h>
#include <platform_track.h>

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
 * \param[in] carrier_freq Doppler [Hz]
 * \param[in] cn0_init     CN0 estimate [dB-Hz]
 * \param[in] TOW_ms       Latest decoded TOW [ms]
 */
void qzss_l1ca_to_l2c_handover(u32 sample_count,
                               u16 sat,
                               double code_phase,
                               double carrier_freq,
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
      .carrier_freq = carrier_freq * QZS_L2_HZ / QZS_L1_HZ,
      .code_phase = code_phase,
      /* chips to correlate during first 1 ms of tracking */
      .chips_to_correlate = code_to_chip_rate(mesid.code) * 1e-3,
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

static void tracker_qzss_l2c_update(tracker_t *tracker) {
  u32 cflags = tp_tracker_update(tracker, &qzss_l2c_config);

  bool bit_aligned =
      ((0 != (cflags & TPF_BSYNC_UPD)) && tracker_bit_aligned(tracker));

  if (!bit_aligned) {
    return;
  }

  /* TOW manipulation on bit edge */
  tracker_tow_cache(tracker);

  bool confirmed = (0 != (tracker->flags & TRACKER_FLAG_CONFIRMED));
  bool in_phase_lock = (0 != (tracker->flags & TRACKER_FLAG_HAS_PLOCK));

  if (in_phase_lock && confirmed) {
    /* naturally synched as we track */
    tracker->bit_polarity = BIT_POLARITY_INVERTED;
    tracker_update_bit_polarity_flags(tracker);
  }
}
