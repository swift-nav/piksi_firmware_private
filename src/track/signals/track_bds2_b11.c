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
#include "track_bds2_b11.h"
#include "signal_db/signal_db.h"
#include "track/track_api.h"
#include "track/track_common.h"
#include "track/track_interface.h"
#include "track/track_utils.h"
#include "track_bds2_b2.h"

/* Non-local headers */
#include <manage.h>
#include <platform_track.h>

/* Libraries */
#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/signal.h>

/* STD headers */
#include <string.h>

/** BDS2 L1 parameter section name */
#define BDS2_B11_TRACK_SETTING_SECTION "bds2_b11_track"

/** BDS2 L1 configuration container */
static tp_tracker_config_t bds2_l1ca_config = TP_TRACKER_DEFAULT_CONFIG;

/* Forward declarations of interface methods for Beidou2 B11 */
static tracker_interface_function_t tracker_bds2_b11_init;
static tracker_interface_function_t tracker_bds2_b11_update;

/** BDS2 B11 tracker interface */
static const tracker_interface_t tracker_interface_bds2_b11 = {
    .code = CODE_BDS2_B1,
    .init = tracker_bds2_b11_init,
    .disable = tp_tracker_disable,
    .update = tracker_bds2_b11_update,
};

static void tracker_bds2_b11_init(tracker_t *tracker_channel) {
  tp_tracker_init(tracker_channel, &bds2_l1ca_config);
}

static void tracker_bds2_b11_update(tracker_t *tracker_channel) {
  u32 cflags = tp_tracker_update(tracker_channel, &bds2_l1ca_config);

  /* If BDS SV is marked unhealthy from B1, also drop B2 tracker */
  if (0 != (tracker_channel->flags & TRACKER_FLAG_UNHEALTHY)) {
    me_gnss_signal_t mesid_drop;
    mesid_drop = construct_mesid(CODE_BDS2_B2, tracker_channel->mesid.sat);
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

  bool confirmed = (0 != (tracker_channel->flags & TRACKER_FLAG_CONFIRMED));
  bool inlock = ((0 != (tracker_channel->flags & TRACKER_FLAG_HAS_PLOCK)) &&
                 (0 != (tracker_channel->flags & TRACKER_FLAG_HAS_FLOCK)));

  if (inlock && confirmed) {
    /* Start B2 tracker if not running */
    bds_b11_to_b2_handover(tracker_channel->sample_count,
                           tracker_channel->mesid.sat,
                           tracker_channel->code_phase_prompt,
                           tracker_channel->carrier_freq,
                           tracker_channel->cn0);
  }
}

/** Register BDS2 B11 tracker into the the tracker interface & settings
 *  framework.
 */
void track_bds2_b11_register(void) {
  tracker_interface_register(&tracker_interface_bds2_b11);
}
