/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#include "decode_bds_b1.h"

#include <assert.h>
#include <string.h>
#include <swiftnav/logging.h>

#include "decode_common.h"
#include "gnss_capabilities/gnss_capabilities.h"
#include "nav_msg/nav_msg_bds.h"
#include "signal_db/signal_db.h"
#include "track/track_decode.h"

/** BDS B1 decoder data */
typedef struct {
  nav_msg_bds_t nav_msg;
} bds_b1_decoder_data_t;

static decoder_t bds_b1_decoders[NUM_BDS2_B11_DECODERS];

static bds_b1_decoder_data_t bds_b1_decoder_data[ARRAY_SIZE(bds_b1_decoders)];

static void decoder_bds_b1_init(const decoder_channel_info_t *channel_info,
                                decoder_data_t *decoder_data);

static void decoder_bds_b1_process(const decoder_channel_info_t *channel_info,
                                   decoder_data_t *decoder_data);

static const decoder_interface_t decoder_interface_bds_b1 = {
    .code = CODE_BDS2_B1,
    .init = decoder_bds_b1_init,
    .disable = decoder_disable,
    .process = decoder_bds_b1_process,
    .decoders = bds_b1_decoders,
    .num_decoders = ARRAY_SIZE(bds_b1_decoders)};

static decoder_interface_list_element_t list_element_bds_b1 = {
    .interface = &decoder_interface_bds_b1, .next = NULL};

void decode_bds_b1_register(void) {
  for (u16 i = 1; i <= ARRAY_SIZE(bds_b1_decoders); i++) {
    bds_b1_decoders[i - 1].active = false;
    bds_b1_decoders[i - 1].data = &bds_b1_decoder_data[i - 1];
  }

  decoder_interface_register(&list_element_bds_b1);
}

static void decoder_bds_b1_init(const decoder_channel_info_t *channel_info,
                                decoder_data_t *decoder_data) {
  bds_b1_decoder_data_t *data = decoder_data;

  memset(data, 0, sizeof(*data));
  bds_nav_msg_init(&data->nav_msg, &channel_info->mesid);
}

static void decoder_bds_b1_process(const decoder_channel_info_t *channel_info,
                                   decoder_data_t *decoder_data) {
  assert(channel_info);
  assert(decoder_data);

  bds_b1_decoder_data_t *data = decoder_data;

  /* Process incoming nav bits */
  nav_bit_t nav_bit;
  u8 channel = channel_info->tracking_channel;

  while (tracker_nav_bit_received(channel, &nav_bit)) {
    bds_decode_status_t status = bds_data_decoding(&data->nav_msg, nav_bit);
    /* Sync tracker with decoder data */
    nav_data_sync_t from_decoder =
        construct_bds_data_sync(&data->nav_msg, status);
    tracker_data_sync(channel, &from_decoder);
  }
}
