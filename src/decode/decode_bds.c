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
#include <libswiftnav/nav_msg_bds.h>

#include "decode_bds.h"
#include "decode.h"
#include "decode_common.h"
#include "ephemeris.h"
#include "ndb.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "shm.h"
#include "signal.h"
#include "timing.h"
#include "track.h"
#include "track/track_sid_db.h"


/** BDS decoder data */
typedef struct { nav_msg_bds_t nav_msg; } bds_decoder_data_t;

static decoder_t bds_decoders[NUM_BDS2_B11_DECODERS];

static bds_decoder_data_t bds_decoder_data[ARRAY_SIZE(bds_decoders)];

static void decoder_bds_init(const decoder_channel_info_t *channel_info,
                             decoder_data_t *decoder_data);

static void decoder_bds_disable(const decoder_channel_info_t *channel_info,
                                decoder_data_t *decoder_data);

static void decoder_bds_process(const decoder_channel_info_t *channel_info,
                                decoder_data_t *decoder_data);

static void dump_navmsg(const nav_msg_bds_t *n);

static const decoder_interface_t decoder_interface_bds = {
    .code = CODE_BDS2_B11,
    .init = decoder_bds_init,
    .disable = decoder_bds_disable,
    .process = decoder_bds_process,
    .decoders = bds_decoders,
    .num_decoders = ARRAY_SIZE(bds_decoders)};

static decoder_interface_list_element_t list_element_bds = {
    .interface = &decoder_interface_bds, .next = NULL};

void decode_bds_register(void) {
  for (u32 i = 0; i < ARRAY_SIZE(bds_decoders); i++) {
    bds_decoders[i].active = false;
    bds_decoders[i].data = &bds_decoder_data[i];
  }

  decoder_interface_register(&list_element_bds);
}

static void decoder_bds_init(const decoder_channel_info_t *channel_info,
                                decoder_data_t *decoder_data) {
  bds_decoder_data_t *data = decoder_data;

  memset(data, 0, sizeof(*data));
  bds_nav_msg_init(&data->nav_msg, channel_info->mesid.sat);
}

static void decoder_bds_disable(const decoder_channel_info_t *channel_info,
                                   decoder_data_t *decoder_data) {
  (void)channel_info;
  (void)decoder_data;
}


static void dump_navmsg(const nav_msg_bds_t *n) {
  char bitstream[256];
  char tempstr[64];
  sprintf(bitstream, " 3 %02d 0 ", n->prn);
  for (u8 k = 0; k < 10; k++) {
    sprintf(tempstr, "%08" PRIx32 " ", n->subframe_bits[k]);
    strcat(bitstream, tempstr);
  }
  log_info("%s", bitstream);
}

static void decoder_bds_process(const decoder_channel_info_t *channel_info,
                                   decoder_data_t *decoder_data) {
  bds_decoder_data_t *data = decoder_data;

  /* Process incoming nav bits */
  nav_bit_fifo_element_t nav_bit;
  u8 channel = channel_info->tracking_channel;

  while (tracking_channel_nav_bit_get(channel, &nav_bit)) {
    bool bit_val = (nav_bit.soft_bit) >= 0;

    bool tlm_rx = bds_nav_msg_update(&data->nav_msg, bit_val);
    if (tlm_rx) {
      dump_navmsg(&data->nav_msg);
      /* this is risky if there are more than 3 seconds worth of symbols
       * in the nav buffer but it's the only way around the problem of
       * knowing the TOW without decoding the navigation data */
      nav_data_sync_t from_decoder;
      tracking_channel_data_sync_init(&from_decoder);
      gps_time_t now = get_current_gps_time();
      from_decoder.TOW_ms = 6000.0 * rintf(now.tow / 6.0);
      from_decoder.bit_polarity = data->nav_msg.bit_polarity;
      tracking_channel_gps_data_sync(channel_info->tracking_channel,
                                     &from_decoder);
    }
  }
  return;
}
