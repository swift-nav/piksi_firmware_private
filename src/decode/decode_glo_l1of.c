/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Tommi Paakki <tommi.paakki@exafore.com>
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

#include <libswiftnav/glo_map.h>
#include <libswiftnav/logging.h>

#include "decode.h"
#include "decode_common.h"
#include "decode_glo_l1of.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "shm/shm.h"
#include "signal_db/signal_db.h"
#include "timing/timing.h"
#include "track/track_decode.h"

/** GLO L1CA decoder data */
typedef struct { nav_msg_glo_t nav_msg; } glo_l1of_decoder_data_t;

static decoder_t glo_l1of_decoders[NUM_GLO_L1OF_DECODERS];
static glo_l1of_decoder_data_t
    glo_l1of_decoder_data[ARRAY_SIZE(glo_l1of_decoders)];

static void decoder_glo_l1of_init(const decoder_channel_info_t *channel_info,
                                  decoder_data_t *decoder_data);
static void decoder_glo_l1of_process(const decoder_channel_info_t *channel_info,
                                     decoder_data_t *decoder_data);

static const decoder_interface_t decoder_interface_glo_l1of = {
    .code = CODE_GLO_L1OF,
    .init = decoder_glo_l1of_init,
    .disable = decoder_disable,
    .process = decoder_glo_l1of_process,
    .decoders = glo_l1of_decoders,
    .num_decoders = ARRAY_SIZE(glo_l1of_decoders)};

static decoder_interface_list_element_t list_element_glo_l1of = {
    .interface = &decoder_interface_glo_l1of, .next = NULL};

void decode_glo_l1of_register(void) {
  for (u16 i = 1; i <= ARRAY_SIZE(glo_l1of_decoders); i++) {
    glo_l1of_decoders[i - 1].active = false;
    glo_l1of_decoders[i - 1].data = &glo_l1of_decoder_data[i - 1];
  }

  decoder_interface_register(&list_element_glo_l1of);
}

static void decoder_glo_l1of_init(const decoder_channel_info_t *channel_info,
                                  decoder_data_t *decoder_data) {
  glo_l1of_decoder_data_t *data = decoder_data;

  memset(data, 0, sizeof(*data));
  nav_msg_init_glo_with_cb(&data->nav_msg, channel_info->mesid);
}

static void decoder_glo_l1of_process(const decoder_channel_info_t *channel_info,
                                     decoder_data_t *decoder_data) {
  glo_l1of_decoder_data_t *data = decoder_data;

  /* Process incoming nav bits */
  nav_bit_t nav_bit;
  const me_gnss_signal_t mesid = channel_info->mesid;
  u8 channel = channel_info->tracking_channel;

  while (tracker_nav_bit_received(channel, &nav_bit)) {
    /* Decode GLO ephemeris. */
    glo_decode_status_t status =
        glo_data_decoding(&data->nav_msg, mesid, nav_bit);
    /* Sync tracker with decoder data */
    glo_data_sync(&data->nav_msg, mesid, channel, status);
  }
  return;
}
