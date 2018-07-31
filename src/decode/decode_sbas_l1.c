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

#include "nav_msg/nav_msg.h"
#include "nav_msg/sbas_msg.h"

#include "decode_common.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "shm/shm.h"
#include "signal_db/signal_db.h"
#include "timing/timing.h"
#include "track/track_decode.h"
#include "track/track_sid_db.h"

#include <assert.h>
#include <string.h>

/** SBAS L1 decoder data */
typedef struct {
  sbas_msg_t sbas_msg;
  sbas_msg_decoder_t sbas_msg_decoder;
  u16 bit_cnt; /**< For navbit data integrity checks */
} sbas_l1_decoder_data_t;

static decoder_t sbas_l1_decoders[NUM_SBAS_L1_DECODERS];
static sbas_l1_decoder_data_t
    sbas_l1_decoder_data[ARRAY_SIZE(sbas_l1_decoders)];

static void decoder_sbas_l1_init(const decoder_channel_info_t *channel_info,
                                 decoder_data_t *decoder_data);

static void decoder_sbas_l1_process(const decoder_channel_info_t *channel_info,
                                    decoder_data_t *decoder_data);

static const decoder_interface_t decoder_interface_sbas_l1 = {
    .code = CODE_SBAS_L1CA,
    .init = decoder_sbas_l1_init,
    .disable = decoder_disable,
    .process = decoder_sbas_l1_process,
    .decoders = sbas_l1_decoders,
    .num_decoders = ARRAY_SIZE(sbas_l1_decoders)};

static decoder_interface_list_element_t list_element_sbas_l1 = {
    .interface = &decoder_interface_sbas_l1, .next = NULL};

void decode_sbas_l1_register(void) {
  for (u16 i = 1; i <= ARRAY_SIZE(sbas_l1_decoders); i++) {
    sbas_l1_decoders[i - 1].active = false;
    sbas_l1_decoders[i - 1].data = &sbas_l1_decoder_data[i - 1];
  }

  decoder_interface_register(&list_element_sbas_l1);
}

static void decoder_sbas_l1_init(const decoder_channel_info_t *channel_info,
                                 decoder_data_t *decoder_data) {
  sbas_l1_decoder_data_t *data = decoder_data;
  memset(data, 0, sizeof(sbas_l1_decoder_data_t));
  data->sbas_msg.sid =
      construct_sid(channel_info->mesid.code, channel_info->mesid.sat);
  data->sbas_msg.tow_ms = TOW_INVALID;
  data->sbas_msg.wn = TOW_INVALID;
  data->sbas_msg.health = SV_HEALTHY;
  data->sbas_msg.bit_polarity = BIT_POLARITY_UNKNOWN;
  sbas_msg_decoder_init(&data->sbas_msg_decoder);
}

static void decoder_sbas_l1_process(const decoder_channel_info_t *channel_info,
                                    decoder_data_t *decoder_data) {
  sbas_l1_decoder_data_t *data = decoder_data;

  /* Process incoming nav bits */
  u8 channel = channel_info->tracking_channel;
  nav_bit_t nav_bit;
  while (tracker_nav_bit_received(channel, &nav_bit)) {
    if ((0 == nav_bit.data) || (nav_bit.cnt != data->bit_cnt)) {
      data->sbas_msg.bit_polarity = BIT_POLARITY_UNKNOWN;
      sbas_msg_decoder_init(&data->sbas_msg_decoder);
      data->bit_cnt = nav_bit.cnt + 1;
      continue;
    }
    data->bit_cnt++;

    /* Update TOW */
    data->sbas_msg.tow_ms = TOW_INVALID;
    data->sbas_msg.wn = TOW_INVALID;
    data->sbas_msg.health = SV_HEALTHY;

    /* Symbol value probability, where 0x00 - 100% of 0, 0xFF - 100% of 1. */
    u8 symbol_probability = nav_bit.data + C_2P7;

    bool decoded = sbas_msg_decoder_add_symbol(
        &data->sbas_msg_decoder, symbol_probability, &data->sbas_msg);

    if (!decoded) {
      /* Nothing decoded, no need to continue. */
      continue;
    }

    nav_data_sync_t from_decoder;
    tracker_data_sync_init(&from_decoder);

    if (TOW_INVALID == data->sbas_msg.tow_ms) {
      /* SBAS message without TOW has been decoded.
       * Bit polarity can still be updated. */
      from_decoder.sync_flags &= ~SYNC_TOW;
    }

    from_decoder.TOW_ms = data->sbas_msg.tow_ms;
    from_decoder.bit_polarity = data->sbas_msg.bit_polarity;
    from_decoder.health = data->sbas_msg.health;
    tracker_data_sync(channel_info->tracking_channel, &from_decoder);
  }
}
