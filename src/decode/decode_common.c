/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Adel Mamin <adel.mamin@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <assert.h>

#include <libswiftnav/glo_map.h>
#include <libswiftnav/gnss_time.h>
#include <libswiftnav/signal.h>

#include "decode_common.h"
#include "ndb/ndb.h"
#include "piksi_systime.h"
#include "timing/timing.h"
#include "track/track_decode.h"
#include "track/track_sid_db.h"

static gps_time_t glo2gps_with_utc_params_cb(me_gnss_signal_t mesid,
                                             const glo_time_t *glo_t) {
  return glo2gps_with_utc_params(mesid, glo_t);
}

void nav_msg_init_glo_with_cb(nav_msg_glo_t *n, me_gnss_signal_t mesid) {
  nav_msg_init_glo(n, mesid, glo2gps_with_utc_params_cb);
}

glo_decode_status_t glo_data_decoding(nav_msg_glo_t *n,
                                      me_gnss_signal_t mesid,
                                      const nav_bit_fifo_element_t *nav_bit) {
  /* Don't trust polarity information while in sensitivity mode. */
  if (nav_bit->sensitivity_mode) {
    glo_decode_status_t status = GLO_DECODE_SENSITIVITY;
    if (BIT_POLARITY_UNKNOWN != n->bit_polarity) {
      /* If polarity was previously known, report polarity loss. */
      status = GLO_DECODE_POLARITY_LOSS;
    }
    nav_msg_init_glo_with_cb(n, mesid);
    return status;
  }

  /* Update GLO data decoder */
  bool bit_val = nav_bit->soft_bit >= 0;
  nav_msg_status_t msg_status = nav_msg_update_glo(n, bit_val);
  if (GLO_STRING_READY != msg_status) {
    return GLO_DECODE_WAIT;
  }

  /* Check for bit errors in the collected string */
  s8 bit_errors = error_detection_glo(n);
  if (bit_errors != 0) {
    log_warn_mesid(mesid, "Parity error");
    return GLO_DECODE_WAIT;
  }

  piksi_systime_t now;
  piksi_systime_get(&now);
  u32 time_tag_ms = piksi_systime_to_ms(&now);

  /* Get GLO strings 1 - 5, and decode full ephemeris */
  string_decode_status_t str_status = process_string_glo(n, time_tag_ms);
  switch (str_status) {
    case GLO_STRING_DECODE_ERROR:
      nav_msg_init_glo_with_cb(n, mesid);
      return GLO_DECODE_WAIT;
      break;
    case GLO_STRING_DECODE_STRING:
      return GLO_DECODE_POLARITY_UPDATE;
      break;
    case GLO_STRING_DECODE_TOW:
      return GLO_DECODE_TOW_UPDATE;
      break;
    case GLO_STRING_DECODE_EPH:
      return GLO_DECODE_EPH_UPDATE;
      break;
    default:
      assert("GLO string decode error");
  }
  return GLO_DECODE_SENSITIVITY;
}

decode_sync_flags_t get_data_sync_flags(const nav_msg_glo_t *n,
                                        me_gnss_signal_t mesid,
                                        glo_decode_status_t status) {
  decode_sync_flags_t flags = 0;

  switch (status) {
    case GLO_DECODE_POLARITY_UPDATE:
    case GLO_DECODE_POLARITY_LOSS:
      /* Update polarity status if new string has been decoded,
       * or a polarity loss has occurred. */
      flags = SYNC_POL;
      break;
    case GLO_DECODE_TOW_UPDATE:
      flags = (SYNC_POL | SYNC_TOW);
      break;
    case GLO_DECODE_EPH_UPDATE:
      /* Store ephemeris and health info */
      save_glo_eph(n, mesid);
      shm_glo_set_shi(n->eph.sid.sat, n->eph.health_bits);
      /* Update polarity and misc data. */
      flags = (SYNC_POL | SYNC_TOW | SYNC_EPH);
      break;
    case GLO_DECODE_WAIT:
    case GLO_DECODE_SENSITIVITY:
    default:
      break;
  }
  return flags;
}

void save_glo_eph(const nav_msg_glo_t *n, me_gnss_signal_t mesid) {
  log_debug_mesid(mesid,
                  "New ephemeris received [%" PRId16 ", %lf]",
                  n->eph.toe.wn,
                  n->eph.toe.tow);

  u16 glo_slot_id = n->eph.sid.sat;
  glo_map_set_slot_id(mesid, glo_slot_id);

  eph_new_status_t r = ephemeris_new(&n->eph);
  if (EPH_NEW_OK != r) {
    log_warn_mesid(mesid,
                   "Error in GLO ephemeris processing. "
                   "Eph status: %" PRIu8 " ",
                   r);
  }
}

bool glo_data_sync(nav_msg_glo_t *n,
                   me_gnss_signal_t mesid,
                   u8 tracking_channel,
                   glo_decode_status_t status) {
  /* Get data sync flags. */
  decode_sync_flags_t flags = get_data_sync_flags(n, mesid, status);
  /* If flags is empty, no updates needed. */
  if (!flags) {
    return false;
  }

  nav_data_sync_t from_decoder;

  tracker_data_sync_init(&from_decoder);

  from_decoder.sync_flags = flags;

  double TOW_ms = n->gps_time.tow * SECS_MS;
  double rounded_TOW_ms = round(TOW_ms);
  if ((0 != (flags & SYNC_TOW)) &&
      ((rounded_TOW_ms > INT32_MAX) || (rounded_TOW_ms < 0))) {
    log_warn_mesid(mesid, "Unexpected TOW value: %lf ms", rounded_TOW_ms);
    return false;
  }
  from_decoder.TOW_ms = (s32)rounded_TOW_ms;

  double delta_TOW_ns = (TOW_ms - rounded_TOW_ms) * 1e6;
  from_decoder.TOW_residual_ns = delta_TOW_ns;

  from_decoder.bit_polarity = n->bit_polarity;
  from_decoder.glo_orbit_slot = n->eph.sid.sat;
  if (shm_ephe_healthy(&n->eph, n->mesid.code)) {
    from_decoder.glo_health = GLO_SV_HEALTHY;
  } else {
    from_decoder.glo_health = GLO_SV_UNHEALTHY;
  }
  tracker_glo_data_sync(tracking_channel, &from_decoder);
  return true;
}

void erase_nav_data(gnss_signal_t target_sid, gnss_signal_t src_sid) {
  char hf_sid_str[SID_STR_LEN_MAX];
  sid_to_string(hf_sid_str, sizeof(hf_sid_str), src_sid);

  /** NAV data health summary or signal health indicates error -> delete data
   * TODO: Read 8bit health words and utilize "the three MSBs of the eight-bit
   *       health words indicate health of the NAV data in accordance with
   *       the code given in Table 20-VII" (IS-GPS-200H chapter 20.3.3.5.1.3
   *       SV Health). These details indicate which of the subframes are bad.
   */
  if (NDB_ERR_NONE == ndb_almanac_erase(target_sid)) {
    log_info_sid(
        target_sid, "almanac deleted (health flags from %s)", hf_sid_str);
  }

  if (NDB_ERR_NONE == ndb_almanac_erase_by_src(target_sid)) {
    log_info_sid(target_sid,
                 "decoded almanacs deleted (health flags from %s)",
                 hf_sid_str);
  }

  if (NDB_ERR_NONE == ndb_ephemeris_erase(target_sid)) {
    log_info_sid(
        target_sid, "ephemeris deleted (health flags from %s)", hf_sid_str);
  }

  /* Clear TOW cache */
  clear_tow_in_sid_db(target_sid);
}

void erase_cnav_data(gnss_signal_t target_sid, gnss_signal_t src_sid) {
  char hf_sid_str[SID_STR_LEN_MAX];
  sid_to_string(hf_sid_str, sizeof(hf_sid_str), src_sid);

  cnav_msg_clear(target_sid);
  log_debug_sid(
      target_sid, "CNAV data cleared (health flags from %s)", hf_sid_str);

  /* Clear TOW cache */
  clear_tow_in_sid_db(target_sid);
}
