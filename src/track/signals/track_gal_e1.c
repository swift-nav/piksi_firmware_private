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
#include "track_gal_e1.h"
#include "signal_db/signal_db.h"
#include "track/track_api.h"
#include "track/track_common.h"
#include "track/track_interface.h"
#include "track/track_utils.h"
#include "track_gal_e7.h"

/* Non-local headers */
#include <manage.h>
#include <platform_track.h>

/* Libraries */
#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/signal.h>

/* STD headers */
#include <string.h>

/** GAL E1 parameter section name */
#define GAL_E1_TRACK_SETTING_SECTION "gal_e1_track"

/** GAL E1 configuration container */
static tp_tracker_config_t gal_e1_config = TP_TRACKER_DEFAULT_CONFIG;

/* Forward declarations of interface methods for Galileo E5b */
static tracker_interface_function_t tracker_gal_e1_init;
static tracker_interface_function_t tracker_gal_e1_update;

/** GAL E1 tracker interface */
static const tracker_interface_t tracker_interface_gal_e1 = {
    .code = CODE_GAL_E1B,
    .init = tracker_gal_e1_init,
    .update = tracker_gal_e1_update,
    .disable = tp_tracker_disable,
};

static void tracker_gal_e1_init(tracker_t *tracker) {
  tp_tracker_init(tracker, &gal_e1_config);

  /* tracker_bit_sync_set(tracker, 0); */
}

static void tracker_gal_e1_update(tracker_t *tracker) {
  u32 cflags = tp_tracker_update(tracker, &gal_e1_config);

  bool bit_aligned =
      ((0 != (cflags & TPF_BSYNC_UPD)) && tracker_bit_aligned(tracker));

  if (!bit_aligned) {
    return;
  }

  bool confirmed = (0 != (tracker->flags & TRACKER_FLAG_CONFIRMED));
  bool inlock = ((0 != (tracker->flags & TRACKER_FLAG_HAS_PLOCK)) &&
                 (0 != (tracker->flags & TRACKER_FLAG_HAS_FLOCK)));

  if (inlock && confirmed) {
    tracker->bit_polarity = BIT_POLARITY_NORMAL;
    tracker_update_bit_polarity_flags(tracker);

    gal_e1_to_e7_handover(tracker->sample_count,
                          tracker->mesid.sat,
                          tracker->code_phase_prompt,
                          tracker->carrier_freq,
                          tracker->cn0);
  }
}

/** Register GAL E1 tracker into the the interface & settings framework.
 */
void track_gal_e1_register(void) {
  tracker_interface_register(&tracker_interface_gal_e1);
}

/** Do E7X to E1X handover.
 *
 * The condition for the handover is that TOW must be known and secondary code
 * matched.
 *
 * \param[in] sample_count NAP sample count
 * \param[in] sat          Satellite ID
 * \param[in] code_phase   code phase [chips]
 * \param[in] carrier_freq Doppler [Hz]
 * \param[in] cn0          CN0 estimate [dB-Hz]
 */
void gal_e7_to_e1_handover(u32 sample_count,
                           u16 sat,
                           double code_phase,
                           double carrier_freq,
                           float cn0_init) {
  /* compose E1 MESID: same SV, but code is E1 */
  me_gnss_signal_t mesid_e1 = construct_mesid(CODE_GAL_E1B, sat);

  if (!tracking_startup_ready(mesid_e1)) {
    log_debug_mesid(mesid_e1, "already in track");
    return; /* E7 signal from the SV is already in track */
  }

  if (!handover_valid(code_phase, GAL_E7_CHIPS_NUM)) {
    log_warn_mesid(mesid_e1, "E7-E1 handover code phase %f", code_phase);
    return;
  }

  tracking_startup_params_t startup_params = {
      .mesid = mesid_e1,
      .sample_count = sample_count,
      /* recalculate doppler freq for E7 from E1 */
      .carrier_freq = carrier_freq * GAL_E1_HZ / GAL_E7_HZ,
      .code_phase = fmod(code_phase / 10.0, code_to_chip_count(CODE_GAL_E1B)),
      /* chips to correlate during first 1 ms of tracking */
      .chips_to_correlate = code_to_chip_rate(mesid_e1.code) * 1e-3,
      /* get initial cn0 from parent E1 channel */
      .cn0_init = cn0_init};

  switch (tracking_startup_request(&startup_params)) {
    case 0:
      break;

    case 1:
      /* sat is already in FIFO, no need to inform */
      break;

    case 2:
      log_warn_mesid(mesid_e1, "failed to start tracking");
      break;

    default:
      assert(!"Unknown code returned");
      break;
  }
}
