/*
 * Copyright (C) 2011 - 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "decode_qzss_l1ca.h"
#include "decode.h"

#include <libswiftnav/logging.h>
#include <libswiftnav/nav_msg.h>

#include "ephemeris.h"
#include "ndb.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "shm.h"
#include "signal.h"
#include "timing.h"
#include "track.h"
#include "track/track_sid_db.h"

#include <assert.h>
#include <string.h>

/** QZSS L1 C/A decoder data */
typedef struct { nav_msg_t nav_msg; } qzss_l1ca_decoder_data_t;

static decoder_t qzss_l1ca_decoders[NUM_QZSS_L1CA_DECODERS];
static qzss_l1ca_decoder_data_t
    qzss_l1ca_decoder_data[ARRAY_SIZE(qzss_l1ca_decoders)];

static void decoder_qzss_l1ca_init(const decoder_channel_info_t *channel_info,
                                   decoder_data_t *decoder_data);
static void decoder_qzss_l1ca_disable(
    const decoder_channel_info_t *channel_info, decoder_data_t *decoder_data);
static void decoder_qzss_l1ca_process(
    const decoder_channel_info_t *channel_info, decoder_data_t *decoder_data);

static const decoder_interface_t decoder_interface_qzss_l1ca = {
    .code = CODE_QZS_L1CA,
    .init = decoder_qzss_l1ca_init,
    .disable = decoder_qzss_l1ca_disable,
    .process = decoder_qzss_l1ca_process,
    .decoders = qzss_l1ca_decoders,
    .num_decoders = ARRAY_SIZE(qzss_l1ca_decoders)};

static decoder_interface_list_element_t list_element_qzss_l1ca = {
    .interface = &decoder_interface_qzss_l1ca, .next = NULL};

void decode_qzss_l1ca_register(void) {
  for (u32 i = 0; i < ARRAY_SIZE(qzss_l1ca_decoders); i++) {
    qzss_l1ca_decoders[i].active = false;
    qzss_l1ca_decoders[i].data = &qzss_l1ca_decoder_data[i];
  }

  decoder_interface_register(&list_element_qzss_l1ca);
}

static void decoder_qzss_l1ca_init(const decoder_channel_info_t *channel_info,
                                   decoder_data_t *decoder_data) {
  (void)channel_info;
  qzss_l1ca_decoder_data_t *data = decoder_data;

  memset(data, 0, sizeof(*data));
  nav_msg_init(&data->nav_msg);
}

static void decoder_qzss_l1ca_disable(
    const decoder_channel_info_t *channel_info, decoder_data_t *decoder_data) {
  (void)channel_info;
  (void)decoder_data;
}

static void decoder_qzss_l1ca_process(
    const decoder_channel_info_t *channel_info, decoder_data_t *decoder_data) {
  qzss_l1ca_decoder_data_t *data = decoder_data;

  /* Process incoming nav bits */
  nav_bit_fifo_element_t nav_bit;
  while (
      tracking_channel_nav_bit_get(channel_info->tracking_channel, &nav_bit)) {
    /* Don't decode data while in sensitivity mode. */
    if (nav_bit.sensitivity_mode) {
      nav_msg_init(&data->nav_msg);
      continue;
    }
    /* Update TOW */
    bool bit_val = nav_bit.soft_bit >= 0;
    nav_data_sync_t from_decoder;
    tracking_channel_data_sync_init(&from_decoder);
    from_decoder.TOW_ms = nav_msg_update(&data->nav_msg, bit_val);

    //~ log_info_mesid(channel_info->mesid, "from_decoder.TOW_ms %6d",
    //~ from_decoder.TOW_ms);

    from_decoder.bit_polarity = data->nav_msg.bit_polarity;
    tracking_channel_gps_data_sync(channel_info->tracking_channel,
                                   &from_decoder);
  }
}
