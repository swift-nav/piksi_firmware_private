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

#include <libswiftnav/logging.h>
#include <libswiftnav/nav_msg_glo.h>
#include <libswiftnav/glo_map.h>

#include "ephemeris.h"
#include "track.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "signal.h"
#include "ndb.h"
#include "shm.h"
#include "timing.h"

#include <assert.h>
#include <string.h>
#include "decode_common.h"

/** GLO L1 C/A decoder data */
typedef struct {
  nav_msg_glo_t nav_msg;
} glo_l1ca_decoder_data_t;

static decoder_t glo_l1ca_decoders[NUM_GLO_L1CA_DECODERS];
static glo_l1ca_decoder_data_t glo_l1ca_decoder_data[ARRAY_SIZE(glo_l1ca_decoders)];

static void decoder_glo_l1ca_init(const decoder_channel_info_t *channel_info,
                                  decoder_data_t *decoder_data);
static void decoder_glo_l1ca_disable(const decoder_channel_info_t *channel_info,
                                     decoder_data_t *decoder_data);
static void decoder_glo_l1ca_process(const decoder_channel_info_t *channel_info,
                                     decoder_data_t *decoder_data);

static const decoder_interface_t decoder_interface_glo_l1ca = {
  .code =         CODE_GLO_L1CA,
  .init =         decoder_glo_l1ca_init,
  .disable =      decoder_glo_l1ca_disable,
  .process =      decoder_glo_l1ca_process,
  .decoders =     glo_l1ca_decoders,
  .num_decoders = ARRAY_SIZE(glo_l1ca_decoders)
};

static decoder_interface_list_element_t list_element_glo_l1ca = {
  .interface = &decoder_interface_glo_l1ca,
  .next = NULL
};

void decode_glo_l1ca_register(void)
{
  for (u32 i = 0; i < ARRAY_SIZE(glo_l1ca_decoders); i++) {
    glo_l1ca_decoders[i].active = false;
    glo_l1ca_decoders[i].data = &glo_l1ca_decoder_data[i];
  }

  decoder_interface_register(&list_element_glo_l1ca);
}

static void decoder_glo_l1ca_init(const decoder_channel_info_t *channel_info,
                                  decoder_data_t *decoder_data)
{
  glo_l1ca_decoder_data_t *data = decoder_data;

  memset(data, 0, sizeof(*data));
  nav_msg_init_glo_with_cb(&data->nav_msg, channel_info->mesid);
}

static void decoder_glo_l1ca_disable(const decoder_channel_info_t *channel_info,
                                     decoder_data_t *decoder_data)
{
  (void)channel_info;
  (void)decoder_data;
}

static void decoder_glo_l1ca_process(const decoder_channel_info_t *channel_info,
                                     decoder_data_t *decoder_data)
{
  glo_l1ca_decoder_data_t *data = decoder_data;

  /* Process incoming nav bits */
  s8 soft_bit;
  bool sensitivity_mode = true;
  me_gnss_signal_t mesid = channel_info->mesid;

  while (tracking_channel_nav_bit_get(channel_info->tracking_channel,
                                      &soft_bit, &sensitivity_mode)) {
    /* Don't trust polarity information while in sensitivity mode. */
    if (sensitivity_mode) {
      nav_msg_init_glo_with_cb(&data->nav_msg, mesid);
      continue;
    }

    /* Update GLO data decoder */
    bool bit_val = soft_bit >= 0;
    nav_msg_status_t msg_status = nav_msg_update_glo(&data->nav_msg, bit_val);
    if (GLO_STRING_READY != msg_status) {
      continue;
    }

    /* Check for bit errors in the collected string */
    s8 bit_errors = error_detection_glo(&data->nav_msg);
    if (bit_errors != 0) {
      nav_msg_init_glo_with_cb(&data->nav_msg, mesid);
      continue;
    }

    /* Get GLO strings 1 - 5, and decode full ephemeris */
    string_decode_status_t str_status = process_string_glo(&data->nav_msg);
    if (GLO_STRING_DECODE_ERROR == str_status) {
      nav_msg_init_glo_with_cb(&data->nav_msg, mesid);
      continue;
    }
    if (GLO_STRING_DECODE_WAIT == str_status) {
      continue;
    }
    assert(GLO_STRING_DECODE_DONE == str_status);

    /* Store new ephemeris */
    log_info_mesid(mesid,
                    "New ephemeris received [%" PRId16 ", %lf]",
                    data->nav_msg.eph.toe.wn, data->nav_msg.eph.toe.tow);
    eph_new_status_t r = ephemeris_new(&data->nav_msg.eph);
    if (EPH_NEW_OK != r) {
      log_warn_mesid(mesid, "Error in GLO ephemeris processing");
    }

    u16 glo_slot_id = data->nav_msg.eph.sid.sat;
    glo_map_set_slot_id(mesid, glo_slot_id);

    nav_data_sync_t from_decoder;

    tracking_channel_data_sync_init(&from_decoder);

    double TOW_ms = data->nav_msg.gps_time.tow * SECS_MS;
    double rounded_TOW_ms = round(TOW_ms);
    if ((rounded_TOW_ms > INT32_MAX) || (rounded_TOW_ms < 0)) {
      log_warn_mesid(mesid, "Unexpected TOW value: %lf ms", rounded_TOW_ms);
      return;
    }
    from_decoder.TOW_ms = (s32)rounded_TOW_ms;

    double delta_TOW_ns = (TOW_ms - rounded_TOW_ms) * 1e6;
    from_decoder.TOW_residual_ns = data->nav_msg.tau_gps_ns + delta_TOW_ns;

    from_decoder.bit_polarity = data->nav_msg.bit_polarity;
    from_decoder.glo_orbit_slot = data->nav_msg.eph.sid.sat;
    from_decoder.health = data->nav_msg.eph.health_bits;
    tracking_channel_glo_data_sync(channel_info->tracking_channel,
                                   &from_decoder);
  }
  return;
}
