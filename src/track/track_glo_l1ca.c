/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Adel Mamin <adel.mamin@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

/* Local headers */
#include "track_glo_l1ca.h"
#include "track.h"
#include "track_cn0.h"
#include "track_glo_l2ca.h" /* for L1CA to L2CA tracking handover */
#include "track_sid_db.h"

/* Non-local headers */
#include <manage.h>
#include <ndb.h>
#include <platform_track.h>
#include <signal.h>

/* Libraries */
#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/track.h>

/* STD headers */
#include <assert.h>
#include <string.h>

/** GLO L1CA configuration section name */
#define GLO_L1CA_TRACK_SETTING_SECTION "glo_l1ca_track"

static tp_tracker_config_t glo_l1ca_config = TP_TRACKER_DEFAULT_CONFIG;

/* Forward declarations of interface methods for GLO L1CA */
static tracker_interface_function_t tracker_glo_l1ca_init;
static tracker_interface_function_t tracker_glo_l1ca_update;

/** GLO L1CA tracker interface */
static const tracker_interface_t tracker_interface_glo_l1ca = {
    .code = CODE_GLO_L1CA,
    .init = tracker_glo_l1ca_init,
    .disable = tp_tracker_disable,
    .update = tracker_glo_l1ca_update,
};

static tracker_interface_list_element_t tracker_interface_list_glo_l1ca = {
    .interface = &tracker_interface_glo_l1ca, .next = 0};

/** Register GLO L1CA tracker into the the tracker interface & settings
 *  framework.
 */
void track_glo_l1ca_register(void) {
  TP_TRACKER_REGISTER_CONFIG(
      GLO_L1CA_TRACK_SETTING_SECTION, glo_l1ca_config, settings_default_notify);

  tracker_interface_register(&tracker_interface_list_glo_l1ca);
}

static void tracker_glo_l1ca_init(tracker_channel_t *tracker_channel) {
  tp_tracker_init(tracker_channel, &glo_l1ca_config);
}

static void tracker_glo_l1ca_update(tracker_channel_t *tracker_channel) {
  u32 tracker_flags = tp_tracker_update(tracker_channel, &glo_l1ca_config);

  /* GLO L1 C/A-specific ToW manipulation */
  update_tow_glo(tracker_channel, tracker_flags);

  /* If GLO SV is marked unhealthy from L1, also drop L2 tracker */
  if (GLO_SV_UNHEALTHY == tracker_channel->health) {
    me_gnss_signal_t mesid_drop;
    mesid_drop = construct_mesid(CODE_GLO_L2CA, tracker_channel->mesid.sat);
    tracking_channel_drop_unhealthy_glo(mesid_drop);
  }

  bool inlock = ((0 != (tracker_channel->flags & TRACKER_FLAG_HAS_PLOCK)) ||
                 (0 != (tracker_channel->flags & TRACKER_FLAG_HAS_FLOCK)));
  if (inlock && (0 != (tracker_channel->flags & TRACKER_FLAG_CONFIRMED)) &&
      (0 != (tracker_flags & TP_CFLAG_BSYNC_UPDATE)) &&
      tracker_bit_aligned(tracker_channel)) {
    /* Start GLO L2CA tracker if not running */
    do_glo_l1ca_to_l2ca_handover(tracker_channel->sample_count,
                                 tracker_channel->mesid.sat,
                                 tracker_channel->code_phase_prompt,
                                 tracker_channel->carrier_freq,
                                 tracker_channel->cn0);
  }
}
