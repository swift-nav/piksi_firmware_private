/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "track_gps_l1ca.h"
#include "track_gps_l2cm.h" /* for L1C/A to L2 CM tracking handover */
#include "track_api.h"
#include "track_cn0.h"

#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/track.h>

#include <string.h>
#include <stdlib.h>
#include <assert.h>

#include "chconf.h"
#include "signal.h"
#include "board.h"
#include "platform_signal.h"
#include "platform_track.h"
#include "chconf_board.h"

#include "track_profiles.h"
#include "track_profile_utils.h"
#include "shm.h"

/** GPS L1 C/A parameter section name */
#define L1CA_TRACK_SETTING_SECTION "l1ca_track"

/**
 * GPS L1 C/A tracker data container type.
 */
typedef struct {
  tp_tracker_data_t data;  /**< Tracker data */
} gps_l1ca_tracker_data_t;

/** GPS L1 C/A configuration container */
static tp_tracker_config_t gps_l1ca_config = TP_TRACKER_DEFAULT_CONFIG;
/** GPS L1 C/A tracker table */
static tracker_t gps_l1ca_trackers[NUM_GPS_L1CA_TRACKERS]
                                   PLATFORM_TRACK_DATA_TRACKER;
/** GPS L1 C/A tracker data */
static gps_l1ca_tracker_data_t gps_l1ca_tracker_data[NUM_GPS_L1CA_TRACKERS];

/* Forward declarations of interface methods for GPS L1 C/A */
static tracker_interface_function_t tracker_gps_l1ca_init;
static tracker_interface_function_t tracker_gps_l1ca_disable;
static tracker_interface_function_t tracker_gps_l1ca_update;

/** GPS L1 C/A tracker interface */
static const tracker_interface_t tracker_interface_gps_l1ca = {
  .code =         CODE_GPS_L1CA,
  .init =         tracker_gps_l1ca_init,
  .disable =      tracker_gps_l1ca_disable,
  .update =       tracker_gps_l1ca_update,
  .trackers =     gps_l1ca_trackers,
  .num_trackers = NUM_GPS_L1CA_TRACKERS
};

/** GPS L1 C/A tracker interface list element */
static tracker_interface_list_element_t
tracker_interface_list_element_gps_l1ca = {
  .interface = &tracker_interface_gps_l1ca,
  .next = 0
};

/** Register GPS L1 C/A tracker into the the tracker interface & settings
 *  framework.
 */
void track_gps_l1ca_register(void)
{
  TP_TRACKER_REGISTER_CONFIG(L1CA_TRACK_SETTING_SECTION, gps_l1ca_config);

  for (u32 i = 0; i < NUM_GPS_L1CA_TRACKERS; i++) {
    gps_l1ca_trackers[i].active = false;
    gps_l1ca_trackers[i].data = &gps_l1ca_tracker_data[i];
  }

  tracker_interface_register(&tracker_interface_list_element_gps_l1ca);
}

static void tracker_gps_l1ca_init(const tracker_channel_info_t *channel_info,
                                  tracker_common_data_t *common_data,
                                  tracker_data_t *tracker_data)
{
  gps_l1ca_tracker_data_t *data = tracker_data;

  memset(data, 0, sizeof(gps_l1ca_tracker_data_t));

  tp_tracker_init(channel_info, common_data, &data->data, &gps_l1ca_config);
}

static void tracker_gps_l1ca_disable(const tracker_channel_info_t *channel_info,
                                     tracker_common_data_t *common_data,
                                     tracker_data_t *tracker_data)
{
  /* Unused parameters */
  (void)common_data;
  (void)tracker_data;
  tp_tracker_disable(channel_info);
}

static void tracker_gps_l1ca_update(const tracker_channel_info_t *channel_info,
                                    tracker_common_data_t *common_data,
                                    tracker_data_t *tracker_data)
{
  gps_l1ca_tracker_data_t *l1ca_data = tracker_data;
  tp_tracker_data_t *data = &l1ca_data->data;
  u32 cflags = tp_tracker_update(channel_info, common_data, data);

  if (data->lock_detect.outp &&
      data->confirmed &&
      0 != (cflags & TP_CFLAG_BSYNC_UPDATE) &&
      tracker_bit_aligned(channel_info->context)) {
    do_l1ca_to_l2cm_handover(common_data->sample_count,
                             channel_info->sid.sat,
                             common_data->code_phase_early,
                             common_data->carrier_freq,
                             common_data->cn0);
  }
}

