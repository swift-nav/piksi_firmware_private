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
#include "track_gal_e7.h"
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
#include <string.h>

/** GAL E7 parameter section name */
#define GAL_E7_TRACK_SETTING_SECTION "gal_e7_track"

/** GAL E7 configuration container */
static tp_tracker_config_t gal_e7_config = TP_TRACKER_DEFAULT_CONFIG;

/* Forward declarations of interface methods for Beidou2 B11 */
static tracker_interface_function_t tracker_gal_e7_init;
static tracker_interface_function_t tracker_gal_e7_update;

/** GAL E7 tracker interface */
static const tracker_interface_t tracker_interface_gal_e7 = {
    .code = CODE_GAL_E7X,
    .init = tracker_gal_e7_init,
    .update = tracker_gal_e7_update,
    .disable = tp_tracker_disable,
};

static void tracker_gal_e7_init(tracker_t *tracker_channel) {
  gal_e7_config.show_unconfirmed_trackers = true;
  tp_tracker_init(tracker_channel, &gal_e7_config);
}

static void tracker_gal_e7_update(tracker_t *tracker_channel) {
  u32 cflags = tp_tracker_update(tracker_channel, &gal_e7_config);

  bool bit_aligned =
      ((0 != (cflags & TPF_BSYNC_UPD)) && tracker_bit_aligned(tracker_channel));

  if (!bit_aligned) {
    return;
  }

  /* TOW manipulation on bit edge */
  //~ tracker_tow_cache(tracker_channel);
}

/** Register GAL E7 tracker into the the interface & settings framework.
 */
void track_gal_e7_register(void) {
  tracker_interface_register(&tracker_interface_gal_e7);
}
