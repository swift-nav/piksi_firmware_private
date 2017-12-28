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

#include "decode_sbas_l1.h"
#include "decode.h"

#include <libswiftnav/logging.h>

#include "ephemeris.h"
#include "nav_msg/nav_msg.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "shm/shm.h"
#include "signal_db/signal_db.h"
#include "timing/timing.h"
#include "track.h"
#include "track/track_sid_db.h"

#include <assert.h>
#include <string.h>

/** SBAS L1 decoder data */
typedef struct { nav_msg_t nav_msg; } sbas_l1_decoder_data_t;

static decoder_t sbas_l1_decoders[NUM_SBAS_L1_DECODERS];
static sbas_l1_decoder_data_t
    sbas_l1_decoder_data[ARRAY_SIZE(sbas_l1_decoders)];

static void decoder_sbas_l1_init(const decoder_channel_info_t *channel_info,
                                 decoder_data_t *decoder_data);
static void decoder_sbas_l1_disable(const decoder_channel_info_t *channel_info,
                                    decoder_data_t *decoder_data);
static void decoder_sbas_l1_process(const decoder_channel_info_t *channel_info,
                                    decoder_data_t *decoder_data);

static const decoder_interface_t decoder_interface_sbas_l1 = {
    .code = CODE_SBAS_L1CA,
    .init = decoder_sbas_l1_init,
    .disable = decoder_sbas_l1_disable,
    .process = decoder_sbas_l1_process,
    .decoders = sbas_l1_decoders,
    .num_decoders = ARRAY_SIZE(sbas_l1_decoders)};

static decoder_interface_list_element_t list_element_sbas_l1 = {
    .interface = &decoder_interface_sbas_l1, .next = NULL};

void decode_sbas_l1_register(void) {
  for (u32 i = 0; i < ARRAY_SIZE(sbas_l1_decoders); i++) {
    sbas_l1_decoders[i].active = false;
    sbas_l1_decoders[i].data = &sbas_l1_decoder_data[i];
  }

  decoder_interface_register(&list_element_sbas_l1);
}

static void decoder_sbas_l1_init(const decoder_channel_info_t *channel_info,
                                 decoder_data_t *decoder_data) {
  (void)channel_info;
  sbas_l1_decoder_data_t *data = decoder_data;

  memset(data, 0, sizeof(*data));
  nav_msg_init(&data->nav_msg);
}

static void decoder_sbas_l1_disable(const decoder_channel_info_t *channel_info,
                                    decoder_data_t *decoder_data) {
  (void)channel_info;
  (void)decoder_data;
}

static void decoder_sbas_l1_process(const decoder_channel_info_t *channel_info,
                                    decoder_data_t *decoder_data) {
  sbas_l1_decoder_data_t *data = decoder_data;

  /* Process incoming nav bits */
  u8 channel = channel_info->tracking_channel;
  nav_bit_fifo_element_t nav_bit;
  while (tracking_channel_nav_bit_get(channel, &nav_bit)) {
    /* Don't decode data while in sensitivity mode. */
    if (nav_bit.sensitivity_mode) {
      nav_msg_init(&data->nav_msg);
      continue;
    }
    /* Update TOW */
    nav_data_sync_t from_decoder;
    tracking_channel_data_sync_init(&from_decoder);

    /* TODO: implement this below using SOFT Viterbi */
    /* s8 soft_bit = nav_bit.soft_bit; */
    /* from_decoder.TOW_ms = sbas_nav_msg_update(&data->nav_msg, soft_bit); */

    log_debug_mesid(channel_info->mesid,
                    "from_decoder.TOW_ms %6" PRId32,
                    from_decoder.TOW_ms);

    from_decoder.bit_polarity = data->nav_msg.bit_polarity;
    tracking_channel_data_sync(channel_info->tracking_channel, &from_decoder);
  }
}
