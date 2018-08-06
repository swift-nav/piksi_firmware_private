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
#include "track_bds2_b2.h"
#include "gnss_capabilities/gnss_capabilities.h"
#include "signal_db/signal_db.h"
#include "track/track_api.h"
#include "track/track_common.h"
#include "track/track_interface.h"
#include "track/track_utils.h"

/* Non-local headers */
#include <manage.h>
#include <platform_track.h>

/* Libraries */
#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/signal.h>

/* STD headers */
#include <assert.h>
#include <string.h>

/** BDS2 B2 configuration section name */
#define BDS2_B2_TRACK_SETTING_SECTION "bds2_b2_track"

/** BDS2 B2 configuration container */
static tp_tracker_config_t bds2_b2_config = TP_TRACKER_DEFAULT_CONFIG;

/* Forward declarations of interface methods for BDS2 B2 */
static tracker_interface_function_t tracker_bds2_b2_init;
static tracker_interface_function_t tracker_bds2_b2_update;

/** BDS2 B2 tracker interface */
static const tracker_interface_t tracker_interface_bds2_b2 = {
    .code = CODE_BDS2_B2,
    .init = tracker_bds2_b2_init,
    .disable = tp_tracker_disable,
    .update = tracker_bds2_b2_update,
};

static void tracker_bds2_b2_init(tracker_t *tracker_channel) {
  tp_tracker_init(tracker_channel, &bds2_b2_config);

  tracker_bit_sync_set(tracker_channel, /* bit_phase_ref = */ 0);
}

static void tracker_bds2_b2_update(tracker_t *tracker_channel) {
  u32 cflags = tp_tracker_update(tracker_channel, &bds2_b2_config);

  /* If BDS SV is marked unhealthy from B2, also drop B1 tracker */
  if (0 != (tracker_channel->flags & TRACKER_FLAG_UNHEALTHY)) {
    me_gnss_signal_t mesid_drop;
    mesid_drop = construct_mesid(CODE_BDS2_B1, tracker_channel->mesid.sat);
    tracker_drop_unhealthy(mesid_drop);
    return;
  }

  bool bit_aligned =
      ((0 != (cflags & TPF_BSYNC_UPD)) && tracker_bit_aligned(tracker_channel));

  if (!bit_aligned) {
    return;
  }

  /* TOW manipulation on bit edge */
  tracker_tow_cache(tracker_channel);
}

/** Register BDS2 B2 tracker into the the tracker interface & settings
 *  framework.
 */
void track_bds2_b2_register(void) {
  tracker_interface_register(&tracker_interface_bds2_b2);
}

/** Do B1 to B2 handover.
 *
 * The condition for the handover is that TOW must be known.
 *
 * \param[in] sample_count NAP sample count
 * \param[in] sat          Satellite ID
 * \param[in] code_phase   code phase [chips]
 * \param[in] carrier_freq Doppler [Hz]
 * \param[in] cn0          CN0 estimate [dB-Hz]
 */
void bds_b11_to_b2_handover(u32 sample_count,
                            u16 sat,
                            double code_phase,
                            double carrier_freq,
                            float cn0_init) {
  /* compose B2 MESID: same SV, but code is B2 */
  me_gnss_signal_t mesid_B2 = construct_mesid(CODE_BDS2_B2, sat);

  if (!bds_b2(mesid_B2)) {
    return; /* B2 signal not available */
  }

  if (!tracking_startup_ready(mesid_B2)) {
    log_debug_mesid(mesid_B2, "already in track");
    return; /* B2 signal from the SV is already in track */
  }

  if (!handover_valid(code_phase, BDS2_B11_CHIPS_NUM)) {
    log_warn_mesid(
        mesid_B2, "Unexpected B1 to B2 hand-over code phase: %f", code_phase);
    return;
  }

  tracking_startup_params_t startup_params = {
      .mesid = mesid_B2,
      .sample_count = sample_count,
      /* recalculate doppler freq for B2 from B1 */
      .carrier_freq = carrier_freq * BDS2_B2_HZ / BDS2_B11_HZ,
      .code_phase = code_phase,
      /* chips to correlate during first 1 ms of tracking */
      .chips_to_correlate = code_to_chip_rate(mesid_B2.code) * 1e-3,
      /* get initial cn0 from parent B1 channel */
      .cn0_init = cn0_init};

  switch (tracking_startup_request(&startup_params)) {
    case 0:
      log_debug_mesid(mesid_B2, "B2 handover done");
      break;

    case 1:
      /* sat is already in fifo, no need to inform */
      break;

    case 2:
      log_warn_mesid(mesid_B2, "Failed to start B2 tracking");
      break;

    default:
      ASSERT(!"Unknown code returned");
      break;
  }
}
