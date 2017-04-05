/*
 * Copyright (C) 2016 - 2017 Swift Navigation Inc.
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

static gps_sat_health_indicators_t gps_shis[NUM_SATS_GPS];

static void bool_shi_2_str(bool set, bool shi, char* str)
{
  str[0] = set ? (shi ? 'Y' : 'N') : '?';
  str[1] = 0;
}

static void int_shi_2_str(bool set, unsigned shi, char* str, int len)
{
  if (set) {
    snprintf(str, len, "%d", shi);
  }
  else {
    str[0] = '?';
    str[1] = 0;
  }
}

static void cns_2_str(code_nav_state_t state, char** state_str)
{
  switch (state) {
    case CODE_NAV_STATE_UNKNOWN:
      *state_str = CODE_NAV_STATE_UNKNOWN_STR;
      break;
    case CODE_NAV_STATE_INVALID:
      *state_str = CODE_NAV_STATE_INVALID_STR;
      break;
    case CODE_NAV_STATE_VALID:
      *state_str = CODE_NAV_STATE_VALID_STR;
      break;
    default:
      assert(!"Unsupported value in code_nav_state_t");
      break;
  }
}

/** Output current health state of GSP satellite to the log.
 *  Function does nothing if DEBUG is off.
 *
 * \param shi_name Name of the SHI that was changed last.
 * \param sat GPS satellite ID for which current state should be logged
 *
 */
void shm_log_sat_state(const char* shi_name, u16 sat)
{
  if (DEBUG) {
    code_nav_state_t s_l1 = shm_get_sat_state(construct_sid(CODE_GPS_L1CA, sat));
    gnss_signal_t l2_sid = construct_sid(CODE_GPS_L2CM, sat);
    code_nav_state_t s_l2 = shm_get_sat_state(l2_sid);
    char* s_l1_str;
    char* s_l2_str;
    cns_2_str(s_l1, &s_l1_str);
    cns_2_str(s_l2, &s_l2_str);
    char shi1_str[4], shi4_str[2], shi6_str[2];
    char shi5_l1_str[2], shi5_l2_str[2], shi5_l5_str[2];
    chMtxLock(&shm_data_access);
    gps_sat_health_indicators_t shis = gps_shis[sat - 1];
    chMtxUnlock(&shm_data_access);
    int_shi_2_str(shis.shi1_set, shis.shi1, shi1_str, sizeof(shi1_str));
    bool_shi_2_str(shis.shi4_set, shis.shi4, shi4_str);
    cnav_msg_t cnav_msg10;
    bool shi5_set = cnav_msg_get(l2_sid, CNAV_MSG_TYPE_10, &cnav_msg10);
    cnav_msg_type_10_t m10 = cnav_msg10.data.type_10;
    bool_shi_2_str(shi5_set, m10.l1_health, shi5_l1_str);
    bool_shi_2_str(shi5_set, m10.l2_health, shi5_l2_str);
    bool_shi_2_str(shi5_set, m10.l5_health, shi5_l5_str);
    bool_shi_2_str(shis.shi6_set, shis.shi6, shi6_str);
    log_debug("GPS SV %02d %s update. State {L1:%s, L2:%s} "
              "SHI[1:%s, 4:%s, 5:{%s,%s,%s}, 6:%s]",
              sat, shi_name, s_l1_str, s_l2_str,
              shi1_str, shi4_str, shi5_l1_str, shi5_l2_str, shi5_l5_str,
              shi6_str);
  }
}

/** Update SHI1 for GPS satellite.
 *  Refer to libswiftnav/shm.h for details of SHIs.
 *
 * \param sat GPS satellite ID
 * \param new_value value to set SHI1 to
 */
void shm_gps_set_shi1(u16 sat, u8 new_value)
{
  assert(sat >= GPS_FIRST_PRN && sat <= NUM_SATS_GPS);
  chMtxLock(&shm_data_access);
  gps_shis[sat - 1].shi1 = new_value;
  gps_shis[sat - 1].shi1_set = true;
  chMtxUnlock(&shm_data_access);
  shm_log_sat_state("SHI1", sat);
}

/** Update SHI4 for GPS satellite.
 *  Refer to libswiftnav/shm.h for details of SHIs.
 *
 * \param sat GPS satellite ID
 * \param new_value value to set SHI4 to
 */
void shm_gps_set_shi4(u16 sat, bool new_value)
{
  assert(sat >= GPS_FIRST_PRN && sat <= NUM_SATS_GPS);
  chMtxLock(&shm_data_access);
  gps_shis[sat - 1].shi4 = new_value;
  gps_shis[sat - 1].shi4_set = true;
  chMtxUnlock(&shm_data_access);
  shm_log_sat_state("SHI4", sat);
}

/** Update SHI6 for GPS satellite.
 *  Refer to libswiftnav/shm.h for details of SHIs.
 *
 * \param sat GPS satellite ID
 * \param new_value value to set SHI6 to
 */
void shm_gps_set_shi6(u16 sat, bool new_value)
{
  assert(sat >= GPS_FIRST_PRN && sat <= NUM_SATS_GPS);
  chMtxLock(&shm_data_access);
  gps_shis[sat - 1].shi6 = new_value;
  gps_shis[sat - 1].shi6_set = true;
  chMtxUnlock(&shm_data_access);
  shm_log_sat_state("SHI6", sat);
}

/** Get signal current health state
 *
 * \param sid Signal ID
 *
 * \returns Current health state of the signal
 */
code_nav_state_t shm_get_sat_state(gnss_signal_t sid)
{
  if(sid_to_constellation(sid) == CONSTELLATION_GPS) {

    assert(CODE_GPS_L1CA == sid.code ||
           CODE_GPS_L2CM == sid.code ||
           CODE_GPS_L2CL == sid.code ||
           CODE_GPS_L1P  == sid.code ||
           CODE_GPS_L2P  == sid.code);

    /* Check GPS band specific SHIs.
     * SHI1. LNAV SV HEALTH (6 bits, subframe 1, word 3)
     * SHI4. LNAV alert flag (HOW, bit 18)
     * SHI5. CNAV L1, L2 [and L5] health (message type 10 bits 52..54)
     * SHI6. CNAV alert flag (bit 38, each message)
     */

    /* Retrieve SHI data */
    chMtxLock(&shm_data_access);
    gps_sat_health_indicators_t shis = gps_shis[sid.sat - 1];
    chMtxUnlock(&shm_data_access);

    switch (sid.code) {
    case CODE_GPS_L1CA: {
      /*
      * Return SV_NAV_STATE_INVALID if either of the following:
      * - SHI1 is available and indicates L1CA unhealthy
      * - SHI4 is available and negative
      * - SHI5 is available and indicates L1CA unhealthy
      *
      * Return CODE_NAV_STATE_VALID all conditions below are true:
      * - SHI1 is available and indicates L1CA healthy
      * - SHI4 is available and positive
      * - One of the following:
      *     - SHI5 is unavailable
      *     - SHI5 is available and indicates L1CA healthy
      *
      * Otherwise return CODE_NAV_STATE_UNKNOWN
      */
      if (shis.shi1_set && !gps_healthy(shis.shi1, sid.code)) {
        return CODE_NAV_STATE_INVALID;
      }

      if (shis.shi4_set && !shis.shi4) {
        return CODE_NAV_STATE_INVALID;
      }

      cnav_msg_t cnav_msg10;
      bool msg10_available = cnav_msg_get(sid, CNAV_MSG_TYPE_10, &cnav_msg10);
      if (msg10_available && !cnav_msg10.data.type_10.l1_health) {
        return CODE_NAV_STATE_INVALID;
      }

      if ((shis.shi1_set && gps_healthy(shis.shi1, sid.code)) &&
          (shis.shi4_set && shis.shi4) &&
          ((!msg10_available) ||
           (msg10_available && cnav_msg10.data.type_10.l1_health))
         ) {
        return CODE_NAV_STATE_VALID;
      }

      return CODE_NAV_STATE_UNKNOWN;
    }

    case CODE_GPS_L2CM: {
      /*
       * Return CODE_NAV_STATE_INVALID if either of the following:
       * - SHI1 is available and indicates L2CM unhealthy
       * - SHI6 is available and negative
       * - SHI5 is available and indicates L2CM unhealthy
       *
       * Return CODE_NAV_STATE_VALID if all conditions below are true:
       * - SHI1 is available and indicates L2CM healthy
       * - SHI5 is available and indicates L2CM healthy
       * - SHI6 is available and positive
       *
       * Otherwise return CODE_NAV_STATE_UNKNOWN
       */
      if (shis.shi1_set && !gps_healthy(shis.shi1, sid.code)) {
        return CODE_NAV_STATE_INVALID;
      }

      if (shis.shi6_set && !shis.shi6) {
        return CODE_NAV_STATE_INVALID;
      }

      cnav_msg_t cnav_msg10;
      bool msg10_available = cnav_msg_get(sid, CNAV_MSG_TYPE_10, &cnav_msg10);
      if (msg10_available && !cnav_msg10.data.type_10.l2_health) {
        return CODE_NAV_STATE_INVALID;
      }

      if ((shis.shi1_set && gps_healthy(shis.shi1, sid.code)) &&
          (msg10_available && cnav_msg10.data.type_10.l2_health) &&
          (shis.shi6_set && shis.shi6)
         ) {
        return CODE_NAV_STATE_VALID;
      }

      return CODE_NAV_STATE_UNKNOWN;
    }

    case CODE_GPS_L1P: {
      /*
      * Return SV_NAV_STATE_INVALID if either of the following:
      * - SHI1 is available and indicates L1P unhealthy
      * - SHI4 is available and negative
      * - SHI5 is available and indicates L1P unhealthy
      *
      * Return CODE_NAV_STATE_VALID all conditions below are true:
      * - SHI1 is available and indicates L1P healthy
      * - SHI4 is available and positive
      * - One of the following:
      *     - Both SHI5 and SHI6 are unavailable
      *     - SHI5 is available and indicates L1P healthy
      *
      * Otherwise return CODE_NAV_STATE_UNKNOWN
      */
      if (shis.shi1_set && !gps_healthy(shis.shi1, sid.code)) {
        return CODE_NAV_STATE_INVALID;
      }

      if (shis.shi4_set && !shis.shi4) {
        return CODE_NAV_STATE_INVALID;
      }

      cnav_msg_t cnav_msg10;
      bool msg10_available = cnav_msg_get(sid, CNAV_MSG_TYPE_10, &cnav_msg10);
      if (msg10_available && !cnav_msg10.data.type_10.l1_health) {
        return CODE_NAV_STATE_INVALID;
      }

      if ((shis.shi1_set && gps_healthy(shis.shi1, sid.code)) &&
          (shis.shi4_set && shis.shi4) &&
          ((!shis.shi6_set && !msg10_available) ||
           (msg10_available && cnav_msg10.data.type_10.l1_health))
         ) {
        return CODE_NAV_STATE_VALID;
      }

      return CODE_NAV_STATE_UNKNOWN;
    }

    case CODE_GPS_L2P: {
      /*
      * Return SV_NAV_STATE_INVALID if either of the following:
      * - SHI1 is available and indicates L2P unhealthy
      * - SHI4 is available and negative
      * - SHI5 is available and indicates L2P unhealthy
      *
      * Return CODE_NAV_STATE_VALID all conditions below are true:
      * - SHI1 is available and indicates L2P healthy
      * - SHI4 is available and positive
      * - One of the following:
      *     - Both SHI5 and SHI6 are unavailable
      *     - SHI5 is available and indicates L2P healthy
      *
      * Otherwise return CODE_NAV_STATE_UNKNOWN
      */
      if (shis.shi1_set && !gps_healthy(shis.shi1, sid.code)) {
        return CODE_NAV_STATE_INVALID;
      }

      if (shis.shi4_set && !shis.shi4) {
        return CODE_NAV_STATE_INVALID;
      }

      cnav_msg_t cnav_msg10;
      bool msg10_available = cnav_msg_get(sid, CNAV_MSG_TYPE_10, &cnav_msg10);
      if (msg10_available && !cnav_msg10.data.type_10.l2_health) {
        return CODE_NAV_STATE_INVALID;
      }

      if ((shis.shi1_set && gps_healthy(shis.shi1, sid.code)) &&
          (shis.shi4_set && shis.shi4) &&
          ((!shis.shi6_set && !msg10_available) ||
           (msg10_available && cnav_msg10.data.type_10.l2_health))
         ) {
        return CODE_NAV_STATE_VALID;
      }

      return CODE_NAV_STATE_UNKNOWN;
    }

    case CODE_GLO_L1CA:
    case CODE_GLO_L2CA: /* Functionality is TBD */
      return CODE_NAV_STATE_UNKNOWN;

    case CODE_GPS_L2CL:
      return CODE_NAV_STATE_UNKNOWN;
    case CODE_SBAS_L1CA:
      assert(!"Unsupported code");

    case CODE_INVALID:
    case CODE_COUNT:
    default:
      assert(!"Invalid code");
    }
  }

  return CODE_NAV_STATE_UNKNOWN;
}

/** Check if tracking of signal is allowed.
 *
 * \param sid Signal ID
 *
 * \returns true if signal health of specified signal
 *               is not CODE_NAV_STATE_INVALID, false otherwise
 */
bool shm_tracking_allowed(gnss_signal_t sid)
{
  assert(sid_valid(sid));
  return shm_get_sat_state(sid) != CODE_NAV_STATE_INVALID;
}

/** Check if signal is suitable for navigation
 *
 * \param sat Signal ID
 *
 * \returns true if health of signal is CODE_NAV_STATE_VALID,
 *          false otherwise
 */
bool shm_navigation_suitable(gnss_signal_t sid)
{
  return shm_get_sat_state(sid) == CODE_NAV_STATE_VALID;
}

/** Check if signal is unusable for navigation
 *
 * \param sat Signal ID
 *
 * \returns true if health of signal is not CODE_NAV_STATE_VALID,
 *          false otherwise
 */
bool shm_navigation_unusable(gnss_signal_t sid)
{
  return shm_get_sat_state(sid) != CODE_NAV_STATE_VALID;
}
