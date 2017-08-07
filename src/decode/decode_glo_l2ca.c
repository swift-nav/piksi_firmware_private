/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Tommi Paakki <tpaakki@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "decode_glo_l2ca.h"
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

/** GLO L2CA decoder data */
typedef struct { nav_msg_glo_t nav_msg; } glo_l2ca_decoder_data_t;

static decoder_t glo_l2ca_decoders[NUM_GLO_L2CA_DECODERS];
static glo_l2ca_decoder_data_t
    glo_l2ca_decoder_data[ARRAY_SIZE(glo_l2ca_decoders)];

static void decoder_glo_l2ca_init(const decoder_channel_info_t *channel_info,
                                  decoder_data_t *decoder_data);
static void decoder_glo_l2ca_disable(const decoder_channel_info_t *channel_info,
                                     decoder_data_t *decoder_data);
static void decoder_glo_l2ca_process(const decoder_channel_info_t *channel_info,
                                     decoder_data_t *decoder_data);

static const decoder_interface_t decoder_interface_glo_l2ca = {
    .code = CODE_GLO_L2CA,
    .init = decoder_glo_l2ca_init,
    .disable = decoder_glo_l2ca_disable,
    .process = decoder_glo_l2ca_process,
    .decoders = glo_l2ca_decoders,
    .num_decoders = ARRAY_SIZE(glo_l2ca_decoders)};

static decoder_interface_list_element_t list_element_glo_l2ca = {
    .interface = &decoder_interface_glo_l2ca, .next = 0};

void decode_glo_l2ca_register(void) {
  for (u32 i = 0; i < ARRAY_SIZE(glo_l2ca_decoders); i++) {
    glo_l2ca_decoders[i].active = false;
    glo_l2ca_decoders[i].data = &glo_l2ca_decoder_data[i];
  }

  decoder_interface_register(&list_element_glo_l2ca);
}

static void decoder_glo_l2ca_init(const decoder_channel_info_t *channel_info,
                                  decoder_data_t *decoder_data) {
  glo_l2ca_decoder_data_t *data = decoder_data;

  memset(data, 0, sizeof(*data));
  nav_msg_init_glo_with_cb(&data->nav_msg, channel_info->mesid);
}

static void decoder_glo_l2ca_disable(const decoder_channel_info_t *channel_info,
                                     decoder_data_t *decoder_data) {
  (void)channel_info;
  (void)decoder_data;
}

static void decoder_glo_l2ca_process(const decoder_channel_info_t *channel_info,
                                     decoder_data_t *decoder_data) {
  glo_l2ca_decoder_data_t *data = decoder_data;

  /* Process incoming nav bits */
  nav_bit_fifo_element_t nav_bit;
  me_gnss_signal_t mesid = channel_info->mesid;
  u8 channel = channel_info->tracking_channel;

  while (tracking_channel_nav_bit_get(channel, &nav_bit)) {
    /* Decode GLO ephemeris. */
    glo_decode_status_t glo_decode_status = glo_data_decoding(&data->nav_msg,
                                                              mesid,
                                                              &nav_bit);
    decode_sync_flags_t flags = 0;
    switch (glo_decode_status) {
    case GLO_DECODE_STRING:
    case GLO_DECODE_POLARITY_LOSS:
      /* Update polarity status if new string has been decoded,
       * or a polarity loss has occurred. */
      flags = SYNC_POLARITY;
      break;
    case GLO_DECODE_DONE:
      /* Store ephemeris and health info */
      save_glo_eph(&data->nav_msg, mesid);
      shm_glo_set_shi(data->nav_msg.eph.sid.sat, data->nav_msg.eph.health_bits);
      /* Update polarity and misc data. */
      flags = (SYNC_POLARITY | SYNC_MISC);
      break;
    case GLO_DECODE_WAIT:
    case GLO_DECODE_SENSITIVITY:
    default:
      continue;
    }

    /* Sync tracker with decoder data */
    if (!glo_data_sync(&data->nav_msg, mesid, channel, flags)) {
      return;
    }
  }
  return;
}
