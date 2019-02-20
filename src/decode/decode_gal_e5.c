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
#include <string.h>

#include <swiftnav/logging.h>

#include "decode.h"
#include "decode_common.h"
#include "decode_gal_e5.h"
#include "gnss_capabilities/gnss_capabilities.h"
#include "nav_msg/nav_msg_gal.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "shm/shm.h"
#include "signal_db/signal_db.h"
#include "timing/timing.h"
#include "track/track_decode.h"
#include "track/track_sid_db.h"

static decoder_t gal_e5_decoders[NUM_GAL_E5_DECODERS];

static nav_msg_gal_fnav_t gal_e5_decoder_data[ARRAY_SIZE(gal_e5_decoders)];

static void decoder_gal_e5_init(const decoder_channel_info_t *channel_info,
                                decoder_data_t *decoder_data);

static void decoder_gal_e5_process(const decoder_channel_info_t *channel_info,
                                   decoder_data_t *decoder_data);

static const decoder_interface_t decoder_interface_gal = {
    .code = CODE_GAL_E5I,
    .init = decoder_gal_e5_init,
    .disable = decoder_disable,
    .process = decoder_gal_e5_process,
    .decoders = gal_e5_decoders,
    .num_decoders = ARRAY_SIZE(gal_e5_decoders)};

static decoder_interface_list_element_t list_element_gal = {
    .interface = &decoder_interface_gal, .next = NULL};

void decode_gal_e5_register(void) {
  /* workaround for `comparison is always false due to limited range of data
   * type` */
  for (u16 i = 1; i <= ARRAY_SIZE(gal_e5_decoders); i++) {
    gal_e5_decoders[i - 1].active = false;
    gal_e5_decoders[i - 1].data = &gal_e5_decoder_data[i - 1];
  }

  decoder_interface_register(&list_element_gal);
}

static void decoder_gal_e5_init(const decoder_channel_info_t *channel_info,
                                decoder_data_t *decoder_data) {
  (void)channel_info;
  nav_msg_gal_fnav_t *data = decoder_data;

  memset(data, 0, sizeof(*data));
}

static void decoder_gal_e5_process(const decoder_channel_info_t *channel_info,
                                   decoder_data_t *decoder_data) {
  assert(channel_info);
  assert(decoder_data);

  /* Process incoming nav bits */
  nav_bit_t nav_bit;
  u8 channel = channel_info->tracking_channel;

  while (tracker_nav_bit_received(channel, &nav_bit)) {
    ;
  } /* while (tracker_nav_bit_get(channel, &nav_bit)) */
}
