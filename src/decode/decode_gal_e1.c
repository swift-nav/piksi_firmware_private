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
#include "decode_common.h"
#include "decode_gal_e1.h"
#include "gnss_capabilities/gnss_capabilities.h"
#include "nav_msg/nav_msg_gal.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "shm/shm.h"
#include "signal_db/signal_db.h"
#include "timing/timing.h"
#include "track/track_decode.h"
#include "track/track_sid_db.h"

/** Galileo decoder data */
static decoder_t gal_e1_decoders[NUM_GAL_E1_DECODERS];

static nav_msg_gal_inav_t gal_e1_decoder_data[ARRAY_SIZE(gal_e1_decoders)];

static void decoder_gal_e1_init(const decoder_channel_info_t *channel_info,
                                decoder_data_t *decoder_data);

static void decoder_gal_e1_process(const decoder_channel_info_t *channel_info,
                                   decoder_data_t *decoder_data);

static const decoder_interface_t decoder_interface_gal_e1 = {
    .code = CODE_GAL_E1B,
    .init = decoder_gal_e1_init,
    .disable = decoder_disable,
    .process = decoder_gal_e1_process,
    .decoders = gal_e1_decoders,
    .num_decoders = ARRAY_SIZE(gal_e1_decoders)};

void decode_gal_e1_register(void) {
  /* workaround for `comparison is always false due to limited range of data
   * type` */
  for (u16 i = 1; i <= ARRAY_SIZE(gal_e1_decoders); i++) {
    gal_e1_decoders[i - 1].active = false;
    gal_e1_decoders[i - 1].data = &gal_e1_decoder_data[i - 1];
  }

  decoder_interface_register(&decoder_interface_gal_e1);
}

static void decoder_gal_e1_init(const decoder_channel_info_t *channel_info,
                                decoder_data_t *decoder_data) {
  nav_msg_gal_inav_t *data = decoder_data;

  memset(data, 0, sizeof(*data));
  gal_inav_msg_init(data, channel_info->mesid.sat);
}

static void decoder_gal_e1_process(const decoder_channel_info_t *channel_info,
                                   decoder_data_t *decoder_data) {
  assert(channel_info);
  assert(decoder_data);

  gal_inav_decoded_t dd;
  gps_time_t t = GPS_TIME_UNKNOWN;
  nav_msg_gal_inav_t *data = decoder_data;
  s32 TOWms = TOW_UNKNOWN;
  nav_data_sync_t from_decoder;

  /* Process incoming nav bits */
  nav_bit_t nav_bit;
  u8 channel = channel_info->channel_id;

  while (tracker_nav_bit_get(channel, &nav_bit)) {
    tracker_data_sync_init(&from_decoder);

    bool upd = gal_inav_msg_update(data, nav_bit);
    if (!upd) continue;

    ephemeris_t *e = &(dd.ephemeris);
    ephemeris_kepler_t *k = &(dd.ephemeris.kepler);
    utc_tm date;
    /* eph_new_status_t estat; */

    inav_data_type_t ret = parse_inav_word(data, &dd, &t);
    switch (ret) {
      case INAV_TOW:
        log_debug_mesid(channel_info->mesid, "WN %d TOW %.3f", t.wn, t.tow);
        TOWms = (s32)rint(t.tow * 1000);
        from_decoder.TOW_ms = TOWms + 2000;
        from_decoder.bit_polarity = data->bit_polarity;
        tracker_data_sync(channel_info->channel_id, &from_decoder);
        break;
      case INAV_EPH:
        make_utc_tm(&(k->toc), &date);
        log_debug("E%02" PRIu8 " %4" PRIu16 " %2" PRIu8 " %2" PRIu8 " %2" PRIu8
                  " %2" PRIu8 " %2" PRIu8 "%19.11E%19.11E%19.11E  ",
                  channel_info->mesid.sat,
                  date.year,
                  date.month,
                  date.month_day,
                  date.hour,
                  date.minute,
                  date.second_int,
                  k->af0,
                  k->af1,
                  k->af2);
        log_debug("    %19.11E%19.11E%19.11E%19.11E  ",
                  (double)k->iode,
                  k->crs,
                  k->dn,
                  k->m0);
        log_debug("    %19.11E%19.11E%19.11E%19.11E  ",
                  k->cuc,
                  k->ecc,
                  k->cus,
                  k->sqrta);
        log_debug("    %19.11E%19.11E%19.11E%19.11E  ",
                  (double)e->toe.tow,
                  k->cic,
                  k->omega0,
                  k->cis);
        log_debug("    %19.11E%19.11E%19.11E%19.11E  ",
                  k->inc,
                  k->crc,
                  k->w,
                  k->omegadot);
        log_debug("    %19.11E%19.11E%19.11E%19.11E  ",
                  k->inc_dot,
                  1.0,
                  (double)e->toe.wn,
                  0.0);
        log_debug("    %19.11E%19.11E%19.11E%19.11E  ",
                  e->ura,
                  (double)e->health_bits,
                  k->tgd_gal_s[0],
                  k->tgd_gal_s[1]);
        log_debug("    %19.11E%19.11E ", rint(t.tow), 0.0);
        dd.ephemeris.sid.code = CODE_GAL_E1B;
        dd.ephemeris.valid = 1;
        shm_gal_set_shi(dd.ephemeris.sid.sat, dd.ephemeris.health_bits);
        /* having the ephemeris on both E1 and E7 is generating
         * discrepancy errors right now..
         * need to figure out a way to refactor that check  */
        /*
        estat = ephemeris_new(&dd.ephemeris);
        if (EPH_NEW_OK != estat) {
          log_warn_mesid(channel_info->mesid,
                         "Error in GAL E1 ephemeris processing. "
                         "Eph status: %" PRIu8 " ",
                         estat);
        }
        */
        break;
      case INAV_UTC:
        log_debug_mesid(channel_info->mesid, "TOW %.3f", t.tow);
        TOWms = (s32)rint(t.tow * 1000);
        from_decoder.TOW_ms = TOWms + 2000;
        from_decoder.bit_polarity = data->bit_polarity;
        tracker_data_sync(channel_info->channel_id, &from_decoder);
        break;
      case INAV_ALM:
        break;
      case INAV_INCOMPLETE:
      default:
        break;
    }
  } /* while (tracker_nav_bit_get(channel, &nav_bit)) */
}
