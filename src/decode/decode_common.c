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
#include <libswiftnav/signal.h>
#include <libswiftnav/time.h>
#include <libswiftnav/nav_msg_glo.h>
#include <libswiftnav/glo_map.h>
#include <assert.h>
#include <string.h>

#include "timing.h"
#include "ephemeris.h"
#include "track.h"

#include "sbp.h"
#include "sbp_utils.h"

static gps_time_t glo2gps_with_utc_params_cb(me_gnss_signal_t mesid,
                                             const glo_time_t *glo_t)
{
  return glo2gps_with_utc_params(mesid, glo_t);
}

void nav_msg_init_glo_with_cb(nav_msg_glo_t *n, me_gnss_signal_t mesid)
{
  nav_msg_init_glo(n, mesid, glo2gps_with_utc_params_cb);
}

bool is_glo_decode_ready(nav_msg_glo_t *n,
                         me_gnss_signal_t mesid,
                         nav_bit_fifo_element_t nav_bit)
{
  /* Don't trust polarity information while in sensitivity mode. */
  if (nav_bit.sensitivity_mode) {
    nav_msg_init_glo_with_cb(n, mesid);
    return false;
  }

  /* Update GLO data decoder */
  bool bit_val = nav_bit.soft_bit >= 0;
  nav_msg_status_t msg_status = nav_msg_update_glo(n, bit_val);
  if (GLO_STRING_READY != msg_status) {
    return false;
  }

  /* Check for bit errors in the collected string */
  s8 bit_errors = error_detection_glo(n);
  if (bit_errors != 0) {
    nav_msg_init_glo_with_cb(n, mesid);
    return false;
  }

  u32 time_tag_s = chVTGetSystemTime() / CH_CFG_ST_FREQUENCY;
  /* Get GLO strings 1 - 5, and decode full ephemeris */
  string_decode_status_t str_status = process_string_glo(n,
                                                         time_tag_s);
  if (GLO_STRING_DECODE_ERROR == str_status) {
    nav_msg_init_glo_with_cb(n, mesid);
    return false;
  }
  if (GLO_STRING_DECODE_WAIT == str_status) {
    return false;
  }
  assert(GLO_STRING_DECODE_DONE == str_status);
  return true;
}

void send_glo_fcn_mapping(gps_time_t t)
{
  msg_fcns_glo_t sbp;
  memset(sbp.fcns, GLO_FCN_UNKNOWN, sizeof(sbp.fcns));
  for (u16 i = GLO_FIRST_PRN; i <= NUM_SATS_GLO; i++) {
    gnss_signal_t tmp_sid = construct_sid(CODE_GLO_L1CA, i);
    if (glo_map_valid(tmp_sid)) {
      sbp.fcns[i] = glo_map_get_fcn(tmp_sid);
    }
  }

  sbp.tow_ms = t.tow;
  sbp.wn = t.wn;

  sbp_send_msg(SBP_MSG_FCNS_GLO, sizeof(sbp), (u8*)&sbp);
}

void save_glo_eph(nav_msg_glo_t *n, me_gnss_signal_t mesid)
{
  log_debug_mesid(mesid,
                 "New ephemeris received [%" PRId16 ", %lf]",
                 n->eph.toe.wn, n->eph.toe.tow);

  /* check if previous value of mapped FCN is different */
  u16 pre_fcn = GLO_FCN_UNKNOWN;
  gnss_signal_t sid = n->eph.sid;
  if (glo_map_valid(sid)) {
    pre_fcn = glo_map_get_fcn(sid);
  }

  n->eph.glo.fcn = mesid.sat;

  /* map new value and then send SBP if FCN mapping changed for the GLO SV */
  if (pre_fcn != mesid.sat) {
    glo_map_set_slot_id(mesid, sid.sat);

    gps_time_t t = {.tow = n->eph.toe.tow, .wn = n->eph.toe.wn};
    send_glo_fcn_mapping(t);
  }

  eph_new_status_t r = ephemeris_new(&n->eph);
  if (EPH_NEW_OK != r) {
    log_warn_mesid(mesid, "Error in GLO ephemeris processing. "
                          "Eph status: %"PRIu8" ", r);
  }
}

bool glo_data_sync(nav_msg_glo_t *n,
                   me_gnss_signal_t mesid,
                   u8 tracking_channel)
{
  nav_data_sync_t from_decoder;

  tracking_channel_data_sync_init(&from_decoder);

  double TOW_ms = n->gps_time.tow * SECS_MS;
  double rounded_TOW_ms = round(TOW_ms);
  if ((rounded_TOW_ms > INT32_MAX) || (rounded_TOW_ms < 0)) {
    log_warn_mesid(mesid, "Unexpected TOW value: %lf ms", rounded_TOW_ms);
    return false;
  }
  from_decoder.TOW_ms = (s32)rounded_TOW_ms;

  double delta_TOW_ns = (TOW_ms - rounded_TOW_ms) * 1e6;
  from_decoder.TOW_residual_ns = delta_TOW_ns;

  from_decoder.bit_polarity = n->bit_polarity;
  from_decoder.glo_orbit_slot = n->eph.sid.sat;
  if (signal_healthy(n->eph.valid,
                     n->eph.health_bits,
                     n->eph.ura,
                     n->mesid.code)) {
    from_decoder.glo_health = GLO_SV_HEALTHY;
  } else {
    from_decoder.glo_health = GLO_SV_UNHEALTHY;
  }
  tracking_channel_glo_data_sync(tracking_channel, &from_decoder);
  return true;
}
