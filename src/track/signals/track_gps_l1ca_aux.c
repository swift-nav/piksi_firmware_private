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
#include "track_gps_l1ca_aux.h"
#include "ndb/ndb.h"
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

/** GPS auxiliary antenna configuration section name */
#define GPS_AUX_TRACK_SETTING_SECTION "gps_aux_track"

/** GPS auxiliary antenna configuration container */
static tp_tracker_config_t gps_aux_config = TP_TRACKER_DEFAULT_CONFIG;

/* Forward declarations of interface methods for GPS AUX */
static tracker_interface_function_t tracker_gps_aux_init;
static tracker_interface_function_t tracker_gps_aux_update;

/** GPS auxiliary antenna tracker interface */
static const tracker_interface_t tracker_interface_gps_aux = {
    .code = CODE_AUX_GPS,
    .init = tracker_gps_aux_init,
    .update = tracker_gps_aux_update,
    .disable = tp_tracker_disable,
};

/** Register tracker into the the tracker interface & settings
 *  framework.
 */
void track_gps_aux_register(void) {
  tracker_interface_register(&tracker_interface_gps_aux);
}

/** Do MAIN to AUX handover.
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
void do_l1ca_to_aux_handover(u32 sample_count,
                             u16 sat,
                             double code_phase,
                             double carrier_freq,
                             float cn0_init) {
  me_gnss_signal_t mesid = construct_mesid(CODE_AUX_GPS, sat);

  if (!tracking_startup_ready(mesid)) {
    return;
  }

  if (!handover_valid(code_phase, GPS_L1CA_CHIPS_NUM)) {
    log_warn_mesid(mesid, "unexpected hand-over code phase: %f", code_phase);
    return;
  }

  tracking_startup_params_t startup_params = {
      .mesid = mesid,
      .sample_count = sample_count,
      .carrier_freq = carrier_freq,
      .code_phase = code_phase,
      /* chips to correlate during first 1 ms of tracking */
      .chips_to_correlate = code_to_chip_rate(mesid.code) * 1e-3,
      /* get initial cn0 from parent L1CA channel */
      .cn0_init = cn0_init};

  switch (tracking_startup_request(&startup_params)) {
    case 0:
      log_debug_mesid(mesid, "handover done");
      break;

    case 1:
      /* sat is already in fifo, no need to inform */
      break;

    case 2:
      log_warn_mesid(mesid, "failed to start tracking");
      break;

    default:
      assert(!"Unknown code returned");
      break;
  }
}

static void tracker_gps_aux_init(tracker_t *tracker_channel) {
  tp_tracker_init(tracker_channel, &gps_aux_config);

  //  tracker_bit_sync_set(tracker_channel, /* bit_phase_ref = */ 0);
}

static void tracker_gps_aux_update(tracker_t *tracker) {
  u32 cflags = tp_tracker_update(tracker, &gps_aux_config);

  bool bit_aligned =
      ((0 != (cflags & TPF_BSYNC_UPD)) && tracker_bit_aligned(tracker));

  if (!bit_aligned) {
    return;
  }

  /* TOW manipulation on bit edge */
  tracker_tow_cache(tracker);

  bool confirmed = (0 != (tracker->flags & TRACKER_FLAG_CONFIRMED));
  bool inlock = ((0 != (tracker->flags & TRACKER_FLAG_HAS_PLOCK)) &&
                 (0 != (tracker->flags & TRACKER_FLAG_HAS_FLOCK)));

  if (inlock && confirmed) {
    tracker->bit_polarity = BIT_POLARITY_NORMAL;
    tracker_update_bit_polarity_flags(tracker);
  }
}
