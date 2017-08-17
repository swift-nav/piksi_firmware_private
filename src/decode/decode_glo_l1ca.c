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

#include "decode_glo_l1ca.h"
#include "decode.h"

#include <libswiftnav/glo_map.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/nav_msg_glo.h>

#include "ephemeris.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "shm.h"
#include "signal.h"
#include "timing.h"
#include "track.h"

#include <assert.h>
#include <string.h>
#include "decode_common.h"

/** GLO L1CA decoder data */
typedef struct { nav_msg_glo_t nav_msg; } glo_l1ca_decoder_data_t;

static decoder_t glo_l1ca_decoders[NUM_GLO_L1CA_DECODERS];
static glo_l1ca_decoder_data_t
    glo_l1ca_decoder_data[ARRAY_SIZE(glo_l1ca_decoders)];

static void decoder_glo_l1ca_init(const decoder_channel_info_t *channel_info,
                                  decoder_data_t *decoder_data);
static void decoder_glo_l1ca_disable(const decoder_channel_info_t *channel_info,
                                     decoder_data_t *decoder_data);
static void decoder_glo_l1ca_process(const decoder_channel_info_t *channel_info,
                                     decoder_data_t *decoder_data);

static const decoder_interface_t decoder_interface_glo_l1ca = {
    .code = CODE_GLO_L1CA,
    .init = decoder_glo_l1ca_init,
    .disable = decoder_glo_l1ca_disable,
    .process = decoder_glo_l1ca_process,
    .decoders = glo_l1ca_decoders,
    .num_decoders = ARRAY_SIZE(glo_l1ca_decoders)};

static decoder_interface_list_element_t list_element_glo_l1ca = {
    .interface = &decoder_interface_glo_l1ca, .next = NULL};

void decode_glo_l1ca_register(void) {
  for (u32 i = 0; i < ARRAY_SIZE(glo_l1ca_decoders); i++) {
    glo_l1ca_decoders[i].active = false;
    glo_l1ca_decoders[i].data = &glo_l1ca_decoder_data[i];
  }

  decoder_interface_register(&list_element_glo_l1ca);
}

static void decoder_glo_l1ca_init(const decoder_channel_info_t *channel_info,
                                  decoder_data_t *decoder_data) {
  glo_l1ca_decoder_data_t *data = decoder_data;

  memset(data, 0, sizeof(*data));
  nav_msg_init_glo(&data->nav_msg, channel_info->mesid);
}

static void decoder_glo_l1ca_disable(const decoder_channel_info_t *channel_info,
                                     decoder_data_t *decoder_data) {
  (void)channel_info;
  (void)decoder_data;
}

static void decoder_glo_l1ca_process(const decoder_channel_info_t *channel_info,
                                     decoder_data_t *decoder_data) {
  glo_l1ca_decoder_data_t *data = decoder_data;

  /* Process incoming nav bits */
  nav_bit_fifo_element_t nav_bit;
  me_gnss_signal_t mesid = channel_info->mesid;
  u8 channel = channel_info->tracking_channel;

  while (tracking_channel_nav_bit_get(channel, &nav_bit)) {
    /* Decode GLO ephemeris. */
    glo_decode_status_t status =
        glo_data_decoding(&data->nav_msg, mesid, &nav_bit);
    /* Sync tracker with decoder data */
    if (!glo_data_sync(&data->nav_msg, mesid, channel, status)) {
      return;
    }
  }
  return;
}
