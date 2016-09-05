/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Roman Gezikov <rgezikov@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <assert.h>
#include <ch.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/cnav_msg.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/shm.h>
#include <signal.h>
#include "cnav_msg_storage.h"
#include "shm.h"

static MUTEX_DECL(shm_data_access);

#define CODE_NAV_STATE_UNKNOWN_STR "CODE_NAV_STATE_UNKNOWN"
#define CODE_NAV_STATE_INVALID_STR "CODE_NAV_STATE_INVALID"
#define CODE_NAV_STATE_VALID_STR   "CODE_NAV_STATE_VALID"

gps_sat_health_indicators_t gps_shis[NUM_SATS_GPS];

static void bool_shi_2_str(bool set, bool shi, char* str)
{
  str[0] = set ? (shi ? 'Y' : 'N') : '?';
  str[1] = 0;
}

static void int_shi_2_str(bool set, unsigned shi, char* str, int len)
{
  if (set)
    snprintf(str, len, "%d", shi);
  else {
    str[0] = '?';
    str[1] = 0;
  }
}

static void shm_log_sat_state_code(gnss_signal_t sid)
{
  code_nav_state_t s = shm_get_sat_state(sid);
  char* s_str = "?";
  switch (s) {
    case CODE_NAV_STATE_UNKNOWN: s_str = CODE_NAV_STATE_UNKNOWN_STR; break;
    case CODE_NAV_STATE_INVALID: s_str = CODE_NAV_STATE_INVALID_STR; break;
    case CODE_NAV_STATE_VALID: s_str = CODE_NAV_STATE_VALID_STR; break;
  }
  char shi1_str[4], shi4_str[2], shi6_str[2];
  char shi5_l1_str[2], shi5_l2_str[2], shi5_l5_str[2];
  chMtxLock(&shm_data_access);
  gps_sat_health_indicators_t shis = gps_shis[sid.sat - 1];
  chMtxUnlock(&shm_data_access);
  int_shi_2_str(shis.shi1_set, shis.shi1, shi1_str, sizeof(shi1_str));
  bool_shi_2_str(shis.shi4_set, shis.shi4, shi4_str);
  cnav_msg_t cnav_msg10;
  bool shi5_set = cnav_msg_get(sid, CNAV_MSG_TYPE_10, &cnav_msg10);
  cnav_msg_type_10_t m10 = cnav_msg10.data.type_10;
  bool_shi_2_str(shi5_set, m10.l1_health, shi5_l1_str);
  bool_shi_2_str(shi5_set, m10.l2_health, shi5_l2_str);
  bool_shi_2_str(shi5_set, m10.l5_health, shi5_l5_str);
  bool_shi_2_str(shis.shi6_set, shis.shi6, shi6_str);
  log_debug_sid(sid, "State: %s SHI[1:%s, 4:%s, 5:{%s,%s,%s}, 6:%s]",
                s_str, shi1_str, shi4_str,
                shi5_l1_str, shi5_l2_str, shi5_l5_str,
                shi6_str);
}

void shm_log_sat_state(u16 sat)
{
  if(DEBUG) {
    shm_log_sat_state_code(construct_sid(CODE_GPS_L1CA, sat));
    shm_log_sat_state_code(construct_sid(CODE_GPS_L2CM, sat));
  }
}

void shm_gps_set_shi1(u16 sat, u8 new_value)
{
  assert(sat >= GPS_FIRST_PRN && sat <= NUM_SATS_GPS);
  chMtxLock(&shm_data_access);
  gps_shis[sat - 1].shi1 = new_value;
  gps_shis[sat - 1].shi1_set = true;
  chMtxUnlock(&shm_data_access);
  shm_log_sat_state(sat);
}

void shm_gps_set_shi4(u16 sat, bool new_value)
{
  assert(sat >= GPS_FIRST_PRN && sat <= NUM_SATS_GPS);
  chMtxLock(&shm_data_access);
  gps_shis[sat - 1].shi4 = new_value;
  gps_shis[sat - 1].shi4_set = true;
  chMtxUnlock(&shm_data_access);
  shm_log_sat_state(sat);
}

void shm_gps_set_shi6(u16 sat, bool new_value)
{
  assert(sat >= GPS_FIRST_PRN && sat <= NUM_SATS_GPS);
  chMtxLock(&shm_data_access);
  gps_shis[sat - 1].shi6 = new_value;
  gps_shis[sat - 1].shi6_set = true;
  chMtxUnlock(&shm_data_access);
  shm_log_sat_state(sat);
}

code_nav_state_t shm_get_sat_state(gnss_signal_t sid)
{
  if(sid_to_constellation(sid) == CONSTELLATION_GPS) {

    assert(CODE_GPS_L1CA == sid.code || CODE_GPS_L2CM == sid.code);

    /* Check GPS band specific SHIs.
     * All SHIs not set -> UNKNOWN
     * At least any set to 'not healthy' -> INVALID
     * Otherwise (all set are set to 'healthy') -> VALID*/

    /* SHI4. LNAV alert flag (HOW, bit 18) */
    /* SHI1. LNAV Ephemeris SV HEALTH (6 bits, subframe 1, word 3) */

    /* SHI6. CNAV alert flag (bit 38, each message) */
    /* SHI5. CNAV L1, L2 [and L5] health (message type 10 bits 52..54) */
    chMtxLock(&shm_data_access);
    gps_sat_health_indicators_t shis = gps_shis[sid.sat - 1];
    chMtxUnlock(&shm_data_access);

    /* 1. If any available SHI shows bad health return CODE_NAV_STATE_INVALID
     *    immediately.
     * */
    if (shis.shi1_set && !gps_healthy(shis.shi1, sid.code))
      return CODE_NAV_STATE_INVALID;

    if (shis.shi4_set && !shis.shi4)
      return CODE_NAV_STATE_INVALID;

    if (shis.shi6_set && !shis.shi6)
      return CODE_NAV_STATE_INVALID;

    cnav_msg_t cnav_msg10;
    bool msg10_available = cnav_msg_get(sid, CNAV_MSG_TYPE_10, &cnav_msg10);
    if (msg10_available) {
      if (CODE_GPS_L1CA == sid.code) {
        if (!cnav_msg10.data.type_10.l1_health)
          return CODE_NAV_STATE_INVALID;
      } else {
        if (!cnav_msg10.data.type_10.l2_health)
          return CODE_NAV_STATE_INVALID;
      }
    }

    /* All present SHIs show good health.
     * */

    /* 2. For L1CA if SHI1 and SHI4 are available
     * (and they are not negative - that was checked already above)
     * return CODE_NAV_STATE_VALID, otherwise CODE_NAV_STATE_UNKNOWN.
     */
    if (!shis.shi1_set || !shis.shi4_set)
      return CODE_NAV_STATE_UNKNOWN;

    if (CODE_GPS_L1CA == sid.code)
      return CODE_NAV_STATE_VALID;

    /* For L2CM check additionally if SHI5 and SHI6 are present.
     * (they can't be negative - that was checked already above)
     * If they are return CODE_NAV_STATE_VALID.
     */
    if (msg10_available && shis.shi6_set)
      return CODE_NAV_STATE_VALID;
  }

  return CODE_NAV_STATE_UNKNOWN;
}

bool shm_gps_l1ca_tracking_allowed(u16 sat)
{
  gnss_signal_t sid = construct_sid(CODE_GPS_L1CA, sat);
  return shm_get_sat_state(sid) != CODE_NAV_STATE_INVALID;
}

bool shm_gps_l2cm_tracking_allowed(u16 sat)
{
  gnss_signal_t sid = construct_sid(CODE_GPS_L2CM, sat);
  return shm_get_sat_state(sid) != CODE_NAV_STATE_INVALID;
}

bool shm_navigation_suitable(gnss_signal_t sid)
{
  return shm_get_sat_state(sid) == CODE_NAV_STATE_VALID;
}

