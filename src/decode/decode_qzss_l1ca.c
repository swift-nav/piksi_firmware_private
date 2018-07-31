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

#include "decode_qzss_l1ca.h"
#include "decode.h"

#include <libswiftnav/logging.h>

#include "decode_common.h"
#include "nav_msg/nav_msg.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "shm/shm.h"
#include "signal_db/signal_db.h"
#include "timing/timing.h"
#include "track/track_decode.h"
#include "track/track_sid_db.h"

#include <assert.h>
#include <string.h>

/** QZSS L1 C/A decoder data */
typedef struct {
  nav_msg_t nav_msg;
  u16 bit_cnt; /**< For navbit data integrity checks */
} qzss_l1ca_decoder_data_t;

static decoder_t qzss_l1ca_decoders[NUM_QZSS_L1CA_DECODERS];
static qzss_l1ca_decoder_data_t
    qzss_l1ca_decoder_data[ARRAY_SIZE(qzss_l1ca_decoders)];

static void decoder_qzss_l1ca_init(const decoder_channel_info_t *channel_info,
                                   decoder_data_t *decoder_data);

static void decoder_qzss_l1ca_process(
    const decoder_channel_info_t *channel_info, decoder_data_t *decoder_data);

static const decoder_interface_t decoder_interface_qzss_l1ca = {
    .code = CODE_QZS_L1CA,
    .init = decoder_qzss_l1ca_init,
    .disable = decoder_disable,
    .process = decoder_qzss_l1ca_process,
    .decoders = qzss_l1ca_decoders,
    .num_decoders = ARRAY_SIZE(qzss_l1ca_decoders)};

static decoder_interface_list_element_t list_element_qzss_l1ca = {
    .interface = &decoder_interface_qzss_l1ca, .next = NULL};

void decode_qzss_l1ca_register(void) {
  /* workaround for `comparison is always false due to limited range of data
   * type` */
  for (u16 i = 1; i <= ARRAY_SIZE(qzss_l1ca_decoders); i++) {
    qzss_l1ca_decoders[i - 1].active = false;
    qzss_l1ca_decoders[i - 1].data = &qzss_l1ca_decoder_data[i - 1];
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

static void decoder_qzss_l1ca_process(
    const decoder_channel_info_t *channel_info, decoder_data_t *decoder_data) {
  qzss_l1ca_decoder_data_t *data = decoder_data;

  /* Process incoming nav bits */
  nav_bit_t nav_bit;
  while (tracker_nav_bit_received(channel_info->tracking_channel, &nav_bit)) {
    if ((0 == nav_bit.data) || (nav_bit.cnt != data->bit_cnt)) {
      nav_msg_init(&data->nav_msg);
      data->bit_cnt = nav_bit.cnt + 1;
      continue;
    }
    data->bit_cnt++;

    /* Update TOW */
    bool bit_val = nav_bit.data > 0;
    nav_data_sync_t from_decoder;
    tracker_data_sync_init(&from_decoder);
    from_decoder.TOW_ms = nav_msg_update(&data->nav_msg, bit_val);

    log_debug_mesid(channel_info->mesid,
                    "from_decoder.TOW_ms %6" PRId32,
                    from_decoder.TOW_ms);

    from_decoder.bit_polarity = data->nav_msg.bit_polarity;
    tracker_data_sync(channel_info->tracking_channel, &from_decoder);
  }
}
