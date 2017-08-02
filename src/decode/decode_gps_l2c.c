/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Adel Mamin <adel.mamin@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "decode_gps_l2c.h"
#include "decode.h"

#include <assert.h>
#include <libswiftnav/cnav_msg.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/nav_msg.h> /* For BIT_POLARITY_... constants */
#include <string.h>

#include "cnav_msg_storage.h"
#include "ephemeris.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "shm.h"
#include "signal.h"
#include "track.h"

/** GPS L2 C decoder data */
typedef struct {
  cnav_msg_t cnav_msg;
  cnav_msg_decoder_t cnav_msg_decoder;
} gps_l2c_decoder_data_t;

static decoder_t gps_l2c_decoders[NUM_GPS_L2CM_DECODERS];
static gps_l2c_decoder_data_t
    gps_l2c_decoder_data[ARRAY_SIZE(gps_l2c_decoders)];

static void decoder_gps_l2c_init(const decoder_channel_info_t *channel_info,
                                 decoder_data_t *decoder_data);
static void decoder_gps_l2c_disable(const decoder_channel_info_t *channel_info,
                                    decoder_data_t *decoder_data);
static void decoder_gps_l2c_process(const decoder_channel_info_t *channel_info,
                                    decoder_data_t *decoder_data);

static const decoder_interface_t decoder_interface_gps_l2c = {
    .code = CODE_GPS_L2CM,
    .init = decoder_gps_l2c_init,
    .disable = decoder_gps_l2c_disable,
    .process = decoder_gps_l2c_process,
    .decoders = gps_l2c_decoders,
    .num_decoders = ARRAY_SIZE(gps_l2c_decoders)};

static decoder_interface_list_element_t list_element_gps_l2c = {
    .interface = &decoder_interface_gps_l2c, .next = 0};

void decode_gps_l2c_register(void)
{
  for (u32 i = 0; i < ARRAY_SIZE(gps_l2c_decoders); i++) {
    gps_l2c_decoders[i].active = false;
    gps_l2c_decoders[i].data = &gps_l2c_decoder_data[i];
  }

  decoder_interface_register(&list_element_gps_l2c);
}

static void decoder_gps_l2c_init(const decoder_channel_info_t *channel_info,
                                 decoder_data_t *decoder_data)
{
  (void)channel_info;
  gps_l2c_decoder_data_t *data = decoder_data;
  memset(data, 0, sizeof(gps_l2c_decoder_data_t));
  data->cnav_msg.bit_polarity = BIT_POLARITY_UNKNOWN;
  cnav_msg_decoder_init(&data->cnav_msg_decoder);
}

static void decoder_gps_l2c_disable(const decoder_channel_info_t *channel_info,
                                    decoder_data_t *decoder_data)
{
  (void)channel_info;
  (void)decoder_data;
}

static void decoder_gps_l2c_process(const decoder_channel_info_t *channel_info,
                                    decoder_data_t *decoder_data)
{
  gps_l2c_decoder_data_t *data = decoder_data;

  /* Process incoming nav bits */
  nav_bit_fifo_element_t nav_bit;
  while (
      tracking_channel_nav_bit_get(channel_info->tracking_channel, &nav_bit)) {
    /* Don't decode data while in sensitivity mode. */
    if (nav_bit.sensitivity_mode) {
      data->cnav_msg.bit_polarity = BIT_POLARITY_UNKNOWN;
      cnav_msg_decoder_init(&data->cnav_msg_decoder);
      continue;
    }
    /* Update TOW */
    u8 symbol_probability;
    u32 delay;
    s32 tow_ms;

    /* Symbol value probability, where 0x00 - 100% of 0, 0xFF - 100% of 1. */
    symbol_probability = nav_bit.soft_bit + POW_TWO_7;

    bool decoded = cnav_msg_decoder_add_symbol(
        &data->cnav_msg_decoder, symbol_probability, &data->cnav_msg, &delay);

    if (!decoded) {
      continue;
    }

    shm_gps_set_shi6(channel_info->mesid.sat, !data->cnav_msg.alert);

    if (CNAV_MSG_TYPE_30 == data->cnav_msg.msg_id) {
      if (data->cnav_msg.data.type_30.tgd_valid) {
        log_debug_mesid(
            channel_info->mesid, "TGD %d", data->cnav_msg.data.type_30.tgd);
      }
      if (data->cnav_msg.data.type_30.isc_l1ca_valid) {
        log_debug_mesid(channel_info->mesid,
                        "isc_l1ca %d",
                        data->cnav_msg.data.type_30.isc_l1ca);
      }
      if (data->cnav_msg.data.type_30.isc_l2c_valid) {
        log_debug_mesid(channel_info->mesid,
                        "isc_l2c %d",
                        data->cnav_msg.data.type_30.isc_l2c);
      }

      cnav_msg_put(&data->cnav_msg);

      sbp_send_group_delay(&data->cnav_msg);
    } else if (CNAV_MSG_TYPE_10 == data->cnav_msg.msg_id) {
      log_debug_mesid(channel_info->mesid,
                      "L1 healthy: %s, L2 healthy: %s, L5 healthy: %s",
                      data->cnav_msg.data.type_10.l1_health ? "Y" : "N",
                      data->cnav_msg.data.type_10.l2_health ? "Y" : "N",
                      data->cnav_msg.data.type_10.l5_health ? "Y" : "N");
      cnav_msg_put(&data->cnav_msg);
    }

    tow_ms =
        data->cnav_msg.tow * GPS_CNAV_MSG_LENGTH * GPS_L2C_SYMBOL_LENGTH_MS;
    tow_ms += delay * GPS_L2C_SYMBOL_LENGTH_MS;
    if (tow_ms >= WEEK_MS) {
      tow_ms -= WEEK_MS;
    }

    nav_data_sync_t from_decoder;
    tracking_channel_data_sync_init(&from_decoder);
    from_decoder.TOW_ms = tow_ms;
    from_decoder.bit_polarity = data->cnav_msg.bit_polarity;
    tracking_channel_gps_data_sync(channel_info->tracking_channel,
                                   &from_decoder);

    /* check PRN conformity */
    bool prn_fail = channel_info->mesid.sat != (u16)data->cnav_msg.prn;
    if (prn_fail) {
      log_warn_mesid(channel_info->mesid,
                     "Decoded PRN %" PRIu16 ". X-corr suspect",
                     data->cnav_msg.prn);
    }
    /* set or clear prn_fail flag for L2CM and parent L1CA */
    tracking_channel_set_prn_fail_flag(channel_info->mesid, prn_fail);
  }
}
