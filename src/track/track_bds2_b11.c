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
#include "track_cn0.h"
#include "track_sid_db.h"

/* Non-local headers */
#include <manage.h>
#include <ndb.h>
#include <platform_track.h>
#include <signal.h>
#include <track.h>

/* Libraries */
#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/track.h>

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
    .code = CODE_BDS2_B11,
    .init = tracker_bds2_b11_init,
    .disable = tp_tracker_disable,
    .update = tracker_bds2_b11_update,
};

/** BDS2 B11 tracker interface list element */
static tracker_interface_list_element_t
    tracker_interface_list_element_bds2_b11 = {
        .interface = &tracker_interface_bds2_b11, .next = 0};

static void tracker_bds2_b11_init(tracker_channel_t *tracker_channel) {
  tp_tracker_init(tracker_channel, &bds2_l1ca_config);
}

static void tracker_bds2_b11_update(tracker_channel_t *tracker_channel) {
  tp_tracker_update(tracker_channel, &bds2_l1ca_config);
}

/** Register BDS2 B11 tracker into the the tracker interface & settings
 *  framework.
 */
void track_bds2_b11_register(void) {
  tracker_interface_register(&tracker_interface_list_element_bds2_b11);
}

