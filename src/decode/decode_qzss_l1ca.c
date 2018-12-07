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

#include <swiftnav/logging.h>

#include "decode.h"
#include "decode_common.h"
#include "decode_gps_l1ca.h"
#include "decode_qzss_l1ca.h"
#include "ephemeris/ephemeris.h"
#include "me_constants.h"
#include "nav_msg/nav_msg.h"
#include "ndb/ndb.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "shm/shm.h"
#include "signal_db/signal_db.h"
#include "timing/timing.h"
#include "track/track_decode.h"
#include "track/track_flags.h"
#include "track/track_sid_db.h"
#include "track/track_state.h"

#include <assert.h>
#include <string.h>

/** QZSS L1 C/A decoder data */
typedef gps_l1ca_decoder_data_t qzss_l1ca_decoder_data_t;

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
  for (u16 i = 0; i < ARRAY_SIZE(qzss_l1ca_decoders); i++) {
    qzss_l1ca_decoders[i].active = false;
    qzss_l1ca_decoders[i].data = &qzss_l1ca_decoder_data[i];
  }

  decoder_interface_register(&list_element_qzss_l1ca);
}

static void decoder_qzss_l1ca_init(const decoder_channel_info_t *channel_info,
                                   decoder_data_t *decoder_data) {
  decoder_gps_l1ca_init(channel_info, decoder_data);
}

static void decoder_qzss_l1ca_process(
    const decoder_channel_info_t *channel_info, decoder_data_t *decoder_data) {
  decoder_gps_l1ca_process(channel_info, decoder_data);
}
