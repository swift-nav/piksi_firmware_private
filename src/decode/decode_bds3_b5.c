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

#include "decode_bds3_b5.h"

#include <assert.h>
#include <string.h>

#include "decode.h"
#include "decode_common.h"
#include "gnss_capabilities/gnss_capabilities.h"
#include "nav_msg/nav_msg_bds.h"
#include "sbp/sbp.h"
#include "sbp/sbp_utils.h"
#include "shm/shm.h"
#include "signal_db/signal_db.h"
#include "timing/timing.h"
#include "track/track_decode.h"
#include "track/track_sid_db.h"

/** BDS3 B2a decoder data */
typedef struct {
  nav_msg_bds_t nav_msg;
} bds3_b5_decoder_data_t;

static decoder_t bds3_b5_decoders[NUM_BDS3_B5_DECODERS];
static bds3_b5_decoder_data_t
    bds3_b5_decoder_data[ARRAY_SIZE(bds3_b5_decoders)];

static void decoder_bds3_b5_init(const decoder_channel_info_t *channel_info,
                                 decoder_data_t *decoder_data);

static void decoder_bds3_b5_process(const decoder_channel_info_t *channel_info,
                                    decoder_data_t *decoder_data);

static const decoder_interface_t decoder_interface_bds3_b5 = {
    .code = CODE_BDS3_B5I,
    .init = decoder_bds3_b5_init,
    .disable = decoder_disable,
    .process = decoder_bds3_b5_process,
    .decoders = bds3_b5_decoders,
    .num_decoders = ARRAY_SIZE(bds3_b5_decoders)};

static decoder_interface_list_element_t list_element_bds3_b5 = {
    .interface = &decoder_interface_bds3_b5, .next = 0};

void decode_bds3_b5_register(void) {
  for (u16 i = 1; i <= ARRAY_SIZE(bds3_b5_decoders); i++) {
    bds3_b5_decoders[i - 1].active = false;
    bds3_b5_decoders[i - 1].data = &bds3_b5_decoder_data[i - 1];
  }

  decoder_interface_register(&list_element_bds3_b5);
}

static void decoder_bds3_b5_init(const decoder_channel_info_t *channel_info,
                                 decoder_data_t *decoder_data) {
  bds3_b5_decoder_data_t *data = decoder_data;

  memset(data, 0, sizeof(*data));
  bds_nav_msg_init(&data->nav_msg, &channel_info->mesid);
}

static void decoder_bds3_b5_process(const decoder_channel_info_t *channel_info,
                                    decoder_data_t *decoder_data) {
  assert(channel_info);
  assert(decoder_data);

  /* Process incoming nav bits */
  nav_bit_t nav_bit;
  u8 channel = channel_info->tracking_channel;

  while (tracker_nav_bit_received(channel, &nav_bit)) {
    ;
  }
}
