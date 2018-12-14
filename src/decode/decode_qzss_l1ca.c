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
  // for (u16 i = 0; i <= ARRAY_SIZE(qzss_l1ca_decoders); i++) {
  //   qzss_l1ca_decoders[i].active = false;
  //   qzss_l1ca_decoders[i].data = &qzss_l1ca_decoder_data[i];
  // }
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
  s8 prev_polarity = BIT_POLARITY_UNKNOWN;
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
    prev_polarity = data->nav_msg.bit_polarity;
    from_decoder.TOW_ms = nav_msg_update(&data->nav_msg, bit_val);
    from_decoder.bit_polarity = data->nav_msg.bit_polarity;
    /* Let's not update TOW together with fast HCA resolution. */
    if (BIT_POLARITY_UNKNOWN == prev_polarity &&
        BIT_POLARITY_UNKNOWN != from_decoder.bit_polarity) {
      /* Only update polarity. */
      from_decoder.sync_flags = SYNC_POL;
    }
    tracker_data_sync(channel_info->tracking_channel, &from_decoder);
  }

  /* Check if there is a new nav msg subframe to process. */
  if (!subframe_ready(&data->nav_msg)) {
    return;
  }

  /* Decode nav data to temporary structure */
  gps_l1ca_decoded_data_t dd;
  s8 ret = process_subframe(&data->nav_msg, channel_info->mesid, &dd);

  if (ret <= 0) {
    return;
  }
  // return;
  gnss_signal_t l1ca_sid =
      construct_sid(channel_info->mesid.code, channel_info->mesid.sat);

  if (dd.invalid_control_or_data) {
    log_info_mesid(channel_info->mesid, "Invalid control or data element");

    ndb_op_code_t c = ndb_ephemeris_erase(l1ca_sid);

    if (NDB_ERR_NONE == c) {
      log_info_mesid(channel_info->mesid, "ephemeris deleted (1/0)");
    } else if (NDB_ERR_NO_CHANGE != c) {
      log_warn_mesid(
          channel_info->mesid, "error %d deleting ephemeris (1/0)", (int)c);
    }
    return;
  }

  shm_qzss_set_shi_lnav_how_alert(l1ca_sid.sat, !data->nav_msg.alert);

  if (dd.shi_ephemeris_upd_flag) {
    log_debug_mesid(
        channel_info->mesid, "shi_ephemeris: 0x%" PRIx8, dd.shi_ephemeris);
    shm_qzss_set_shi_ephemeris(l1ca_sid.sat, dd.shi_ephemeris);
  }

  /* Health indicates CODE_NAV_STATE_INVALID for L2CM */
  gnss_signal_t l2cm_sid = construct_sid(CODE_QZS_L2CM, l1ca_sid.sat);
  if (shm_signal_unhealthy(l2cm_sid)) {
    /* Clear CNAV data and TOW cache */
    erase_cnav_data(l2cm_sid, l1ca_sid);
  }

  /* Health indicates CODE_NAV_STATE_INVALID */
  if (shm_signal_unhealthy(l1ca_sid)) {
    /* Clear NDB and TOW cache */
    erase_nav_data(l1ca_sid, l1ca_sid);
    /* Clear decoded subframe data */
    nav_msg_clear_decoded(&data->nav_msg);
    return;
  }

  /* Do not use data from sv that is not declared as CODE_NAV_STATE_VALID. */
  if (!shm_navigation_suitable(l1ca_sid)) {
    return;
  }

  if (dd.ephemeris_upd_flag) {
    /* Store new ephemeris to NDB */
    log_debug_mesid(channel_info->mesid,
                    "New ephemeris received [%" PRId16 ", %lf]",
                    dd.ephemeris.toe.wn,
                    dd.ephemeris.toe.tow);
    eph_new_status_t r = ephemeris_new(&dd.ephemeris);

    switch (r) {
      case EPH_NEW_OK:
      case EPH_NEW_ERR:
        break;
      case EPH_NEW_XCORR:
        log_info_mesid(channel_info->mesid,
                       "Channel cross-correlation detected "
                       "(ephe/ephe or ephe/alm check)");
        /* Ephemeris cross-correlates with almanac of another SV */
        tracker_set_xcorr_flag(channel_info->mesid);
        break;
      default:
        break;
    }
  }

}
