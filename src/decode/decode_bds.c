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
#include "decode_bds.h"
#include "decode_common.h"
#include "gnss_capabilities/gnss_capabilities.h"
#include "nav_msg/nav_msg_bds.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "shm/shm.h"
#include "signal_db/signal_db.h"
#include "timing/timing.h"
#include "track/track_decode.h"
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
  u32 tow = (((n->frame_words[0] >> 4) << 12) |
             ((n->frame_words[1] >> 18) & 0xfffU)) &
            0xfffffU;
  sprintf(bitstream, " 3 %02d %6" PRIu32 "  ", n->prn, tow);
  for (u8 k = 0; k < BDS_WORD_SUBFR; k++) {
    sprintf(tempstr, "%08" PRIx32 " ", n->frame_words[k]);
    strcat(bitstream, tempstr);
  }
  log_info("%s", bitstream);
}

static void decoder_bds_process(const decoder_channel_info_t *channel_info,
                                decoder_data_t *decoder_data) {
  bds_d1_decoded_data_t dd_d1nav;
  bds_d2_decoded_data_t dd_d2nav;

  assert(channel_info);
  assert(decoder_data);

  memset(&dd_d1nav, 0, sizeof(bds_d1_decoded_data_t));
  memset(&dd_d2nav, 0, sizeof(bds_d2_decoded_data_t));

  bds_decoder_data_t *data = decoder_data;
  me_gnss_signal_t mesid = channel_info->mesid;

  /* Process incoming nav bits */
  nav_bit_fifo_element_t nav_bit;
  u8 channel = channel_info->tracking_channel;

  while (tracker_nav_bit_get(channel, &nav_bit)) {
    bool bit_val = (nav_bit.soft_bit) >= 0;

    bool tlm_rx = bds_nav_msg_update(&data->nav_msg, bit_val);
    if (tlm_rx) {
      dump_navmsg(&data->nav_msg);

      s32 TOWms = BDS_TOW_INVALID;
      nav_data_sync_t from_decoder;
      tracker_data_sync_init(&from_decoder);
      if (bds_d2nav(mesid)) {
        TOWms = bds_d2_process_subframe(&data->nav_msg, mesid, &dd_d2nav);
        from_decoder.TOW_ms = TOWms - 60;
      } else {
        TOWms = bds_d1_process_subframe(&data->nav_msg, mesid, &dd_d1nav);
        from_decoder.TOW_ms = TOWms - 600;
      }
      from_decoder.bit_polarity = data->nav_msg.bit_polarity;
      tracker_data_sync(channel_info->tracking_channel, &from_decoder);
    }
  }
  return;
}
