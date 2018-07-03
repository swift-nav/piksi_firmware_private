/*
 * Copyright (C) 2018 Swift Navigation Inc.
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
#include "decode_bds_b2.h"
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

/** BDS B2 decoder data */
typedef struct { nav_msg_bds_t nav_msg; } bds_b2_decoder_data_t;

static decoder_t bds_b2_decoders[NUM_BDS2_B2_DECODERS];

static bds_b2_decoder_data_t bds_b2_decoder_data[ARRAY_SIZE(bds_b2_decoders)];

static void decoder_bds_b2_init(const decoder_channel_info_t *channel_info,
                                decoder_data_t *decoder_data);

static void decoder_bds_b2_process(const decoder_channel_info_t *channel_info,
                                   decoder_data_t *decoder_data);

static const decoder_interface_t decoder_interface_bds_b2 = {
    .code = CODE_BDS2_B2,
    .init = decoder_bds_b2_init,
    .disable = decoder_disable,
    .process = decoder_bds_b2_process,
    .decoders = bds_b2_decoders,
    .num_decoders = ARRAY_SIZE(bds_b2_decoders)};

static decoder_interface_list_element_t list_element_bds_b2 = {
    .interface = &decoder_interface_bds_b2, .next = NULL};

void decode_bds_b2_register(void) {
  for (u16 i = 1; i <= ARRAY_SIZE(bds_b2_decoders); i++) {
    bds_b2_decoders[i - 1].active = false;
    bds_b2_decoders[i - 1].data = &bds_b2_decoder_data[i - 1];
  }

  decoder_interface_register(&list_element_bds_b2);
}

static void decoder_bds_b2_init(const decoder_channel_info_t *channel_info,
                                decoder_data_t *decoder_data) {
  bds_b2_decoder_data_t *data = decoder_data;

  memset(data, 0, sizeof(*data));
  bds_nav_msg_init(&data->nav_msg, channel_info->mesid.sat);
}

static void decoder_bds_b2_process(const decoder_channel_info_t *channel_info,
                                   decoder_data_t *decoder_data) {
  bds_d1_decoded_data_t dd_d1nav;
  bds_d2_decoded_data_t dd_d2nav;

  assert(channel_info);
  assert(decoder_data);

  memset(&dd_d1nav, 0, sizeof(bds_d1_decoded_data_t));
  memset(&dd_d2nav, 0, sizeof(bds_d2_decoded_data_t));

  bds_b2_decoder_data_t *data = decoder_data;
  me_gnss_signal_t mesid = channel_info->mesid;

  /* Process incoming nav bits */
  nav_bit_t nav_bit;
  u8 channel = channel_info->tracking_channel;

  while (tracker_nav_bit_get(channel, &nav_bit)) {
    /* Don't decode data while in sensitivity mode. */
    if (0 == nav_bit) {
      bds_nav_msg_init(&data->nav_msg, mesid.sat);
      continue;
    }

    bool bit_val = nav_bit > 0;

    bool tlm_rx = bds_nav_msg_update(&data->nav_msg, bit_val);
    if (tlm_rx) {
      s32 TOWms = TOW_INVALID;
      nav_data_sync_t from_decoder;
      tracker_data_sync_init(&from_decoder);
      if (bds_d2nav(mesid)) {
        TOWms = bds_d2_process_subframe(&data->nav_msg, mesid, &dd_d2nav);
        if (TOW_INVALID != TOWms) {
          from_decoder.TOW_ms = TOWms - 60;
        }
        if (dd_d2nav.ephemeris_upd_flag) {
          shm_bds_set_shi(dd_d2nav.ephemeris.sid.sat,
                          dd_d2nav.ephemeris.health_bits);
          eph_new_status_t r = ephemeris_new(&dd_d2nav.ephemeris);
          if (EPH_NEW_OK != r) {
            log_warn_mesid(mesid,
                           "Error in BDS d2nav ephemeris processing. "
                           "Eph status: %" PRIu8 " ",
                           r);
          }
          dd_d2nav.ephemeris_upd_flag = false;
        }
      } else {
        TOWms = bds_d1_process_subframe(&data->nav_msg, mesid, &dd_d1nav);
        if (TOW_INVALID != TOWms) {
          from_decoder.TOW_ms = TOWms;
        }
        if (dd_d1nav.ephemeris_upd_flag) {
          shm_bds_set_shi(dd_d1nav.ephemeris.sid.sat,
                          dd_d1nav.ephemeris.health_bits);
          eph_new_status_t r = ephemeris_new(&dd_d1nav.ephemeris);
          if (EPH_NEW_OK != r) {
            log_warn_mesid(mesid,
                           "Error in BDS d1nav ephemeris processing. "
                           "Eph status: %" PRIu8 " ",
                           r);
          }
          dd_d1nav.ephemeris_upd_flag = false;
        }
      }
      from_decoder.bit_polarity = data->nav_msg.bit_polarity;
      tracker_data_sync(channel_info->tracking_channel, &from_decoder);
    }
  }
  return;
}
