/*
 * Copyright (C) 2017 Swift Navigation Inc.
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
#include "track_sbas_l1.h"

#include "signal_db/signal_db.h"
#include "track/track_api.h"
#include "track/track_common.h"
#include "track/track_interface.h"
#include "track/track_sid_db.h"
#include "track/track_utils.h"

/* Non-local headers */
#include <acq/manage.h>
#include <platform_track.h>

/* Libraries */
#include <swiftnav/constants.h>
#include <swiftnav/logging.h>
#include <swiftnav/signal.h>

/* STD headers */
#include <string.h>

/** SBAS L1 parameter section name */
#define SBAS_L1_TRACK_SETTING_SECTION "sbas_l1_track"

/** SBAS L1 configuration container */
static tp_tracker_config_t sbas_l1ca_config = TP_TRACKER_DEFAULT_CONFIG;

/* Forward declarations of interface methods for SBAS L1 */
static tracker_interface_function_t tracker_sbas_l1ca_init;
static tracker_interface_function_t tracker_sbas_l1ca_update;

/** SBAS L1 tracker interface */
static const tracker_interface_t tracker_interface_sbas_l1ca = {
    .code = CODE_SBAS_L1CA,
    .init = tracker_sbas_l1ca_init,
    .disable = tp_tracker_disable,
    .update = tracker_sbas_l1ca_update,
};

/** Register SBAS L1 tracker into the the tracker interface & settings
 *  framework.
 */
void track_sbas_l1_register(void) {
  tracker_interface_register(&tracker_interface_sbas_l1ca);
}

static void tracker_sbas_l1ca_init(tracker_t *tracker) {
  tp_tracker_init(tracker, &sbas_l1ca_config);
}

static void tracker_sbas_l1ca_update(tracker_t *tracker) {
  u32 cflags = tp_tracker_update(tracker, &sbas_l1ca_config);

  bool bit_aligned =
      ((0 != (cflags & TPF_BSYNC_UPD)) && tracker_bit_aligned(tracker));

  if (!bit_aligned) {
    return;
  }
  tracker_tow_cache(tracker);
}
