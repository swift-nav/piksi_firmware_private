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
#include <assert.h>
#include <string.h>

#include <libswiftnav/logging.h>

#include "decode.h"
#include "decode_common.h"
#include "decode_gal_e1.h"
#include "gnss_capabilities/gnss_capabilities.h"
#include "nav_msg/nav_msg_gal.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "shm/shm.h"
#include "signal_db/signal_db.h"
#include "timing/timing.h"
#include "track/track_decode.h"
#include "track/track_sid_db.h"

/** Galileo decoder data */
static decoder_t gal_e1_decoders[NUM_GAL_E1_DECODERS];

static nav_msg_gal_inav_t gal_e1_decoder_data[ARRAY_SIZE(gal_e1_decoders)];

static void decoder_gal_e1_init(const decoder_channel_info_t *channel_info,
                                decoder_data_t *decoder_data);

static void decoder_gal_e1_process(const decoder_channel_info_t *channel_info,
                                   decoder_data_t *decoder_data);

static const decoder_interface_t decoder_interface_gal = {
    .code = CODE_GAL_E1B,
    .init = decoder_gal_e1_init,
    .disable = decoder_disable,
    .process = decoder_gal_e1_process,
    .decoders = gal_e1_decoders,
    .num_decoders = ARRAY_SIZE(gal_e1_decoders)};

static decoder_interface_list_element_t list_element_gal = {
    .interface = &decoder_interface_gal, .next = NULL};

void decode_gal_e1_register(void) {
  /* workaround for `comparison is always false due to limited range of data
   * type` */
  for (u16 i = 1; i <= ARRAY_SIZE(gal_e1_decoders); i++) {
    gal_e1_decoders[i - 1].active = false;
    gal_e1_decoders[i - 1].data = &gal_e1_decoder_data[i - 1];
  }

  decoder_interface_register(&list_element_gal);
}

static void decoder_gal_e1_init(const decoder_channel_info_t *channel_info,
                                decoder_data_t *decoder_data) {
  nav_msg_gal_inav_t *data = decoder_data;

  memset(data, 0, sizeof(*data));
  gal_inav_msg_init(data, &channel_info->mesid);
}

static void decoder_gal_e1_process(const decoder_channel_info_t *channel_info,
                                   decoder_data_t *decoder_data) {
  assert(channel_info);
  assert(decoder_data);

  nav_msg_gal_inav_t *data = decoder_data;

  nav_data_sync_t from_decoder;

  /* Process incoming nav bits */
  nav_bit_t nav_bit;
  u8 channel = channel_info->tracking_channel;

  while (tracker_nav_bit_get(channel, &nav_bit)) {
    gal_decode_status_t status = gal_data_decoding(data, nav_bit);
    if (GAL_DECODE_RESET == status) {
      gal_inav_msg_init(data, &channel_info->mesid);
      continue;
    }

    /* Sync tracker with decoder data */
    get_gal_data_sync(data, &from_decoder, status);
    tracker_data_sync(channel, &from_decoder);
  }
}
