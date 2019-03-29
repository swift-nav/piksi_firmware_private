/*
 * Copyright (C) 2018 Swift Navigation Inc.
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
#include "track_qzss_l5.h"

#include "ndb/ndb.h"
#include "signal_db/signal_db.h"
#include "track/track_api.h"
#include "track/track_common.h"
#include "track/track_interface.h"
#include "track/track_utils.h"

/* Non-local headers */
#include <acq/manage.h>
#include <platform_track.h>

/* Libraries */
#include <swiftnav/constants.h>
#include <swiftnav/logging.h>
#include <swiftnav/signal.h>

/* STD headers */
#include <assert.h>
#include <string.h>

/** QZSS L5 configuration container */
static tp_tracker_config_t qzss_l5_config = TP_TRACKER_DEFAULT_CONFIG;

/* Forward declarations of interface methods for QZSS L5 */
static tracker_interface_function_t tracker_qzss_l5_init;
static tracker_interface_function_t tracker_qzss_l5_update;

/** QZSS L5 tracker interface */
static const tracker_interface_t tracker_interface_qzss_l5 = {
    .code = CODE_QZS_L5I,
    .init = tracker_qzss_l5_init,
    .disable = tp_tracker_disable,
    .update = tracker_qzss_l5_update,
};

/** Register L5 tracker into the the tracker interface & settings
 *  framework.
 */
void track_qzss_l5_register(void) {
  tracker_interface_register(&tracker_interface_qzss_l5);
}

/** Do L1CA to L5 handover.
 *
 * The condition for the handover is that TOW must be known.
 *
 * \param[in] sample_count NAP sample count
 * \param[in] sat          Satellite ID
 * \param[in] code_phase   code phase [chips]
 * \param[in] carrier_freq Doppler [Hz]
 * \param[in] cn0          CN0 estimate [dB-Hz]
 * \param[in] TOW_ms       Latest decoded TOW [ms]
 */
void do_qzss_l1_to_l5_handover(u32 sample_count,
                               u16 sat,
                               double code_phase,
                               double carrier_freq,
                               float cn0_init) {
  /* compose L5 MESID: same SV, but code is L5 */
  me_gnss_signal_t mesid_L5 = construct_mesid(CODE_QZS_L5I, sat);

  if (!tracking_startup_ready(mesid_L5)) {
    return;
  }

  if (!handover_valid(code_phase, QZS_L1CA_CHIPS_NUM)) {
    log_warn_mesid(mesid_L5, "Unexpected hand-over code phase: %f", code_phase);
    return;
  }

  tracking_startup_params_t startup_params = {
      .mesid = mesid_L5,
      .sample_count = sample_count,
      /* recalculate L5 Doppler freq from L1 */
      .doppler_hz = carrier_freq * QZS_L5_HZ / QZS_L1_HZ,
      .code_phase = fmod(code_phase * code_to_chip_rate(CODE_QZS_L5I) /
                             code_to_chip_rate(CODE_QZS_L1CA),
                         code_to_chip_count(CODE_QZS_L5I)),
      /* chips to correlate during first 1 ms of tracking */
      .chips_to_correlate = code_to_chip_rate(CODE_QZS_L5I) * 1e-3,
      /* get initial cn0 from parent L1CA channel */
      .cn0_init = cn0_init};

  switch (tracking_startup_request(&startup_params)) {
    case 0:
      log_debug_mesid(mesid_L5, "handover done");
      break;

    case 1:
      /* sat is already in fifo, no need to inform */
      break;

    case 2:
      log_warn_mesid(mesid_L5, "failed to start tracking");
      break;

    default:
      assert(!"Unknown code returned");
      break;
  }
}

static void tracker_qzss_l5_init(tracker_t *tracker_channel) {
  tp_tracker_init(tracker_channel, &qzss_l5_config);

  /* L5 bit sync is known once we start tracking it since
     the L5 ranging code length matches the bit length (20ms).
     This is the end of 20ms integration period and the edge
     of a data bit. */
  tracker_bit_sync_set(tracker_channel, /* bit_phase_ref = */ 0);
}

static void tracker_qzss_l5_update(tracker_t *tracker) {
  u32 cflags = tp_tracker_update(tracker, &qzss_l5_config);

  bool bit_aligned =
      ((0 != (cflags & TPF_BSYNC_UPD)) && tracker_bit_aligned(tracker));

  if (!bit_aligned) {
    return;
  }

  /* TOW manipulation on bit edge */
  /* tracker_tow_cache(tracker); */
  bool settled = (0 == (tracker->flags & TRACKER_FLAG_RECOVERY_MODE));
  bool inlock = tracker_has_all_locks(tracker);

  if (inlock && settled && (TOW_UNKNOWN != (tracker->TOW_ms))) {
    /* naturally synched as we track */
    tracker->bit_polarity = BIT_POLARITY_NORMAL;
    tracker_update_bit_polarity_flags(tracker);
  }
}
