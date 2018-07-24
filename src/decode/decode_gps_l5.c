/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Michele Bavaro <michele@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <assert.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/gnss_time.h>
#include <libswiftnav/logging.h>
#include <string.h>

#include "decode.h"
#include "decode_common.h"
#include "decode_gps_l5.h"
#include "nav_msg/nav_msg.h" /* For BIT_POLARITY_... constants */
#include "ndb/ndb_utc.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "shm/shm.h"
#include "signal_db/signal_db.h"
#include "track/track_decode.h"
#include "track/track_flags.h"

/** GPS L5 decoder data */
typedef struct {
  cnav_msg_t cnav_msg;
  cnav_msg_decoder_t cnav_msg_decoder;
} gps_l5_decoder_data_t;

static decoder_t gps_l5_decoders[NUM_GPS_L5_DECODERS];
static gps_l5_decoder_data_t gps_l5_decoder_data[ARRAY_SIZE(gps_l5_decoders)];

static void decoder_gps_l5_init(const decoder_channel_info_t *channel_info,
                                decoder_data_t *decoder_data);

static void decoder_gps_l5_process(const decoder_channel_info_t *channel_info,
                                   decoder_data_t *decoder_data);

static const decoder_interface_t decoder_interface_gps_l5 = {
    .code = CODE_GPS_L5I,
    .init = decoder_gps_l5_init,
    .disable = decoder_disable,
    .process = decoder_gps_l5_process,
    .decoders = gps_l5_decoders,
    .num_decoders = ARRAY_SIZE(gps_l5_decoders)};

static decoder_interface_list_element_t list_element_gps_l5 = {
    .interface = &decoder_interface_gps_l5, .next = 0};

void decode_gps_l5_register(void) {
  for (u16 i = 0; i < ARRAY_SIZE(gps_l5_decoders); i++) {
    gps_l5_decoders[i].active = false;
    gps_l5_decoders[i].data = &gps_l5_decoder_data[i];
  }

  decoder_interface_register(&list_element_gps_l5);
}

static void decoder_gps_l5_init(const decoder_channel_info_t *channel_info,
                                decoder_data_t *decoder_data) {
  (void)channel_info;
  gps_l5_decoder_data_t *data = decoder_data;
  memset(data, 0, sizeof(gps_l5_decoder_data_t));
  data->cnav_msg.bit_polarity = BIT_POLARITY_UNKNOWN;
  cnav_msg_decoder_init(&data->cnav_msg_decoder);
}

static void decoder_gps_l5_process(const decoder_channel_info_t *channel_info,
                                   decoder_data_t *decoder_data) {
  assert(channel_info);
  assert(decoder_data);

  /* Process incoming nav bits */
  nav_bit_t nav_bit;
  while (tracker_nav_bit_get(channel_info->tracking_channel, &nav_bit)) {
    ;
  }
}
