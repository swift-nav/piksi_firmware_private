/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Michele Bavaro <michele@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <assert.h>
#include <inttypes.h>
#include <string.h>
#include <swiftnav/constants.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/logging.h>

#include "decode.h"
#include "decode_common.h"
#include "decode_gps_l2c.h"
#include "decode_qzss_l2c.h"
#include "nav_msg/nav_msg.h" /* For BIT_POLARITY_... constants */
#include "ndb/ndb_utc.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "shm/shm.h"
#include "signal_db/signal_db.h"
#include "track/track_decode.h"
#include "track/track_flags.h"

/** QZSS L2CM decoder data */
typedef gps_l2c_decoder_data_t qzss_l2c_decoder_data_t;

static decoder_t qzss_l2c_decoders[NUM_QZSS_L2C_DECODERS];
static qzss_l2c_decoder_data_t
    qzss_l2c_decoder_data[ARRAY_SIZE(qzss_l2c_decoders)];

static void decoder_qzss_l2c_init(const decoder_channel_info_t *channel_info,
                                 decoder_data_t *decoder_data);

static void decoder_qzss_l2c_process(const decoder_channel_info_t *channel_info,
                                    decoder_data_t *decoder_data);

static const decoder_interface_t decoder_interface_qzss_l2c = {
    .code = CODE_QZS_L2CM,
    .init = decoder_qzss_l2c_init,
    .disable = decoder_disable,
    .process = decoder_qzss_l2c_process,
    .decoders = qzss_l2c_decoders,
    .num_decoders = ARRAY_SIZE(qzss_l2c_decoders)};

static decoder_interface_list_element_t list_element_qzss_l2c = {
    .interface = &decoder_interface_qzss_l2c, .next = 0};

void decode_qzss_l2c_register(void) {
  for (u16 i = 0; i < ARRAY_SIZE(qzss_l2c_decoders); i++) {
    qzss_l2c_decoders[i].active = false;
    qzss_l2c_decoders[i].data = &qzss_l2c_decoder_data[i];
  }

  decoder_interface_register(&list_element_qzss_l2c);
}

static void decoder_qzss_l2c_init(const decoder_channel_info_t *channel_info,
                                 decoder_data_t *decoder_data) {
  decoder_gps_l2c_init(channel_info, decoder_data);
}

static void decoder_qzss_l2c_process(const decoder_channel_info_t *channel_info,
                                     decoder_data_t *decoder_data) {
  decoder_gps_l2c_process(channel_info, decoder_data);
}
