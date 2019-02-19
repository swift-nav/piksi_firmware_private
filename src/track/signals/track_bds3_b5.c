/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Michele Bavaro <michele@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

/* Local headers */
#include "track_bds3_b5.h"
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
#include <swiftnav/constants.h>
#include <swiftnav/logging.h>
#include <swiftnav/signal.h>

/* STD headers */
#include <assert.h>
#include <string.h>

/** BDS3 B2a configuration section name */
#define BDS3_B5_TRACK_SETTING_SECTION "bds3_b5_track"

/** BDS3 B2a configuration container */
static tp_tracker_config_t bds3_b5_config = TP_TRACKER_DEFAULT_CONFIG;

/* Forward declarations of interface methods for BDS3 B2a */
static tracker_interface_function_t tracker_bds3_b5_init;
static tracker_interface_function_t tracker_bds3_b5_update;

/** BDS3 B2a tracker interface */
static const tracker_interface_t tracker_interface_bds3_b5 = {
    .code = CODE_BDS3_B5I,
    .init = tracker_bds3_b5_init,
    .disable = tp_tracker_disable,
    .update = tracker_bds3_b5_update,
};

static void tracker_bds3_b5_init(tracker_t *tracker) {
  tp_tracker_init(tracker, &bds3_b5_config);

  //  tracker_bit_sync_set(tracker, /* bit_phase_ref = */ 0);
}

static void tracker_bds3_b5_update(tracker_t *tracker) {
  u32 cflags = tp_tracker_update(tracker, &bds3_b5_config);

  bool bit_aligned =
      ((0 != (cflags & TPF_BSYNC_UPD)) && tracker_bit_aligned(tracker));

  if (!bit_aligned) {
    return;
  }

  /* TOW manipulation on bit edge */
  tracker_tow_cache(tracker);

  bool confirmed = (0 != (tracker->flags & TRACKER_FLAG_CONFIRMED));
  bool in_phase_lock = (0 != (tracker->flags & TRACKER_FLAG_HAS_PLOCK)) &&
                       (0 != (tracker->flags & TRACKER_FLAG_HAS_FLOCK));

  if (in_phase_lock && confirmed) {
    /* naturally synched as we track */
    tracker->bit_polarity = BIT_POLARITY_NORMAL;
    tracker_update_bit_polarity_flags(tracker);
  }
}

/** Register BDS3 B2a tracker into the the tracker interface & settings
 *  framework.
 */
void track_bds3_b5_register(void) {
  tracker_interface_register(&tracker_interface_bds3_b5);
}

/** Do B1 to B2a handover.
 *
 * The condition for the handover is that TOW must be known.
 *
 * \param[in] sample_count NAP sample count
 * \param[in] sat          Satellite ID
 * \param[in] code_phase   code phase [chips]
 * \param[in] carrier_freq Doppler [Hz]
 * \param[in] cn0          CN0 estimate [dB-Hz]
 */
void bds_b1_to_b5_handover(u32 sample_count,
                           u16 sat,
                           double code_phase,
                           double carrier_freq,
                           float cn0_init) {
  /* same SV, but code is B5 */
  me_gnss_signal_t mesid_B5 = construct_mesid(CODE_BDS3_B5I, sat);

  if (!bds_b2a(mesid_B5)) {
    return; /* B2a signal not available */
  }

  if (!tracking_startup_ready(mesid_B5)) {
    log_debug_mesid(mesid_B5, "already in track");
    return; /* B2a signal from the SV is already in track */
  }

  if (!handover_valid(code_phase, BDS2_B11_CHIPS_NUM)) {
    log_warn_mesid(mesid_B5, "Unexpected hand-over code phase: %f", code_phase);
    return;
  }

  tracking_startup_params_t startup_params = {
      .mesid = mesid_B5,
      .sample_count = sample_count,
      /* recalculate doppler freq for B2a from B1I */
      .doppler_hz = carrier_freq * GPS_L5_HZ / BDS2_B11_HZ,
      .code_phase = fmod(code_phase * code_to_chip_rate(CODE_GPS_L5I) /
                             code_to_chip_rate(CODE_BDS2_B1),
                         code_to_chip_count(CODE_GPS_L5I)),
      /* chips to correlate during first 1 ms of tracking */
      .chips_to_correlate = code_to_chip_rate(mesid_B5.code) * 1e-3,
      /* get initial cn0 from parent B1 channel */
      .cn0_init = cn0_init};

  switch (tracking_startup_request(&startup_params)) {
    case 0:
      log_debug_mesid(mesid_B5, "handover done");
      break;

    case 1:
      /* sat is already in fifo, no need to inform */
      break;

    case 2:
      log_warn_mesid(mesid_B5, "failed to start tracking");
      break;

    default:
      assert(!"Unknown code returned");
      break;
  }
}
