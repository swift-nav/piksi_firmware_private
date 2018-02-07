/*
 * Copyright (C) 2016 - 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

/* Local headers */
#include "track_gps_l1ca.h"
#include "filters/filter_common.h"
#include "signal_db/signal_db.h"
#include "track/track_api.h"
#include "track/track_common.h"
#include "track/track_interface.h"
#include "track/track_utils.h"
#include "track_gps_l2c.h" /* for L1C/A to L2C tracking handover */
#include "xcorr/xcorr.h"

/* Non-local headers */
#include <manage.h>
#include <platform_track.h>

/* Libraries */
#include <libswiftnav/ch_meas.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/signal.h>

/* STD headers */
#include <string.h>

/** GPS L1 C/A parameter section name */
#define L1CA_TRACK_SETTING_SECTION "l1ca_track"

/** GPS L1 C/A configuration container */
static tp_tracker_config_t gps_l1ca_config = TP_TRACKER_DEFAULT_CONFIG;

/* Forward declarations of interface methods for GPS L1 C/A */
static tracker_interface_function_t tracker_gps_l1ca_init;
static tracker_interface_function_t tracker_gps_l1ca_update;

/** GPS L1 C/A tracker interface */
static const tracker_interface_t tracker_interface_gps_l1ca = {
    .code = CODE_GPS_L1CA,
    .init = tracker_gps_l1ca_init,
    .disable = tp_tracker_disable,
    .update = tracker_gps_l1ca_update,
};

/** GPS L1 C/A tracker interface list element */
static tracker_interface_list_element_t
    tracker_interface_list_element_gps_l1ca = {
        .interface = &tracker_interface_gps_l1ca, .next = 0};

/** Register GPS L1 C/A tracker into the the tracker interface & settings
 *  framework.
 */
void track_gps_l1ca_register(void) {
  lp1_filter_compute_params(&gps_l1ca_config.xcorr_f_params,
                            gps_l1ca_config.xcorr_cof,
                            SECS_MS / GPS_L1CA_BIT_LENGTH_MS);

  tracker_interface_register(&tracker_interface_list_element_gps_l1ca);
}

static void tracker_gps_l1ca_init(tracker_t *tracker_channel) {
  gps_l1ca_tracker_data_t *data = &tracker_channel->gps_l1ca;

  memset(data, 0, sizeof(*data));

  tp_tracker_init(tracker_channel, &gps_l1ca_config);
}

static void tracker_gps_l1ca_update(tracker_t *tracker) {
  u32 cflags = tp_tracker_update(tracker, &gps_l1ca_config);

  bool bit_aligned =
      ((0 != (cflags & TPF_BSYNC_UPD)) && tracker_bit_aligned(tracker));

  if (!bit_aligned) {
    return;
  }

  /* TOW manipulation on bit edge */
  tracker_tow_cache(tracker);

  /* Cross-correlation check for GPS signals */
  tracker_xcorr_update(tracker, &gps_l1ca_config);

  bool confirmed = (0 != (tracker->flags & TRACKER_FLAG_CONFIRMED));
  bool inlock = ((0 != (tracker->flags & TRACKER_FLAG_HAS_PLOCK)) ||
                 (0 != (tracker->flags & TRACKER_FLAG_HAS_FLOCK)));

  if (inlock && confirmed && (TOW_UNKNOWN != (tracker->TOW_ms))) {
    /* Start L2C tracker if not running */
    do_l1ca_to_l2c_handover(tracker->sample_count,
                            tracker->mesid.sat,
                            tracker->code_phase_prompt,
                            tracker->carrier_freq,
                            tracker->cn0,
                            tracker->TOW_ms);
  }
}
