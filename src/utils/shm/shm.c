/*
 * Copyright (C) 2016 - 2017 Swift Navigation Inc.
 * Contact: Michele Bavaro <michele@swift-nav.com>
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
#include <libswiftnav/logging.h>
#include <libswiftnav/shm.h>

#include "nav_msg/cnav_msg_storage.h"
#include "nav_msg/nav_msg.h"
#include "ndb/ndb.h"
#include "shm.h"
#include "signal_db/signal_db.h"

static MUTEX_DECL(shm_data_access);

#define CODE_NAV_STATE_UNKNOWN_STR "CODE_NAV_STATE_UNKNOWN"
#define CODE_NAV_STATE_INVALID_STR "CODE_NAV_STATE_INVALID"
#define CODE_NAV_STATE_VALID_STR "CODE_NAV_STATE_VALID"

static gps_sat_health_indicators_t gps_shis[NUM_SATS_GPS];
static glo_sat_health_indicators_t glo_shis[NUM_SATS_GLO];
static bds_sat_health_indicators_t bds_shis[NUM_SATS_BDS2];

static void bool_shi_2_str(bool set, bool shi, char* str) {
  str[0] = set ? (shi ? 'Y' : 'N') : '?';
  str[1] = 0;
}

static void int_shi_2_str(bool set, unsigned shi, char* str, int len) {
  if (set) {
    snprintf(str, len, "%d", shi);
  } else {
    str[0] = '?';
    str[1] = 0;
  }
}

static void cns_2_str(code_nav_state_t state, char** state_str) {
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

/** Get signal current health state
 *
 * \param sid Signal ID
 *
 * \returns Current health state of the signal
 */
static code_nav_state_t shm_get_sat_state(gnss_signal_t sid) {
  /* Skip GLO satellites if they do not have orbit slot decoded. */
  if (IS_GLO(sid) && !glo_slot_id_is_valid(sid.sat)) {
    return CODE_NAV_STATE_UNKNOWN;
  }

  assert(sid_valid(sid));

  /* Check GPS band specific SHIs.
   * SHI1. LNAV SV HEALTH (6 bits, subframe 1, word 3)
   * SHI4. LNAV alert flag (HOW, bit 18)
   * SHI5. CNAV L1, L2 [and L5] health (message type 10 bits 52..54)
   * SHI6. CNAV alert flag (bit 38, each message)
   */

  /* Check GLO SHI.
   * SHI. SV HEALTH (MSB of B || l).
   */

  /* Retrieve SHI data */
  chMtxLock(&shm_data_access);
  gps_sat_health_indicators_t shis = gps_shis[sid.sat - GPS_FIRST_PRN];
  glo_sat_health_indicators_t shi = glo_shis[sid.sat - GLO_FIRST_PRN];
  bds_sat_health_indicators_t shi_bds = bds_shis[sid.sat - BDS2_FIRST_PRN];
  chMtxUnlock(&shm_data_access);

  /* Check common GPS */
  if (IS_GPS(sid)) {
    if (shis.shi1_set && !check_6bit_health_word(shis.shi1, sid.code)) {
      return CODE_NAV_STATE_INVALID;
    }

    almanac_t a;
    ndb_op_code_t oc = ndb_almanac_read(sid, &a);
    if (NDB_ERR_NONE == oc &&
        !check_6bit_health_word(a.health_bits, sid.code)) {
      return CODE_NAV_STATE_INVALID;
    }
  }

  switch (sid.code) {
    case CODE_GPS_L1CA: {
      /*
      * Return SV_NAV_STATE_INVALID if either of the following:
      * - SHI1 is available and indicates L1CA unhealthy
      * - SHI4 is available and negative
      * - SHI5 is available and indicates L1CA unhealthy
      * - almanac health bits are available and indicate L1 unhealthy
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
      if (shis.shi4_set && !shis.shi4) {
        return CODE_NAV_STATE_INVALID;
      }

      cnav_msg_t cnav_msg10;
      bool msg10_available = cnav_msg_get(sid, CNAV_MSG_TYPE_10, &cnav_msg10);
      if (msg10_available && !cnav_msg10.data.type_10.l1_health) {
        return CODE_NAV_STATE_INVALID;
      }

      if ((shis.shi1_set && check_6bit_health_word(shis.shi1, sid.code)) &&
          (shis.shi4_set && shis.shi4) &&
          ((!msg10_available) ||
           (msg10_available && cnav_msg10.data.type_10.l1_health))) {
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
      * - almanac health bits are available and indicate L1 unhealthy
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
      if (shis.shi4_set && !shis.shi4) {
        return CODE_NAV_STATE_INVALID;
      }

      cnav_msg_t cnav_msg10;
      bool msg10_available = cnav_msg_get(sid, CNAV_MSG_TYPE_10, &cnav_msg10);
      if (msg10_available && !cnav_msg10.data.type_10.l1_health) {
        return CODE_NAV_STATE_INVALID;
      }

      if ((shis.shi1_set && check_6bit_health_word(shis.shi1, sid.code)) &&
          (shis.shi4_set && shis.shi4) &&
          ((!shis.shi6_set && !msg10_available) ||
           (msg10_available && cnav_msg10.data.type_10.l1_health))) {
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
       * - almanac health bits are available and indicate L1 unhealthy
       *
       * Return CODE_NAV_STATE_VALID if all conditions below are true:
       * - SHI1 is available and indicates L2CM healthy
       * - SHI5 is available and indicates L2CM healthy
       * - SHI6 is available and positive
       *
       * Otherwise return CODE_NAV_STATE_UNKNOWN
       */
      if (shis.shi6_set && !shis.shi6) {
        return CODE_NAV_STATE_INVALID;
      }

      cnav_msg_t cnav_msg10;
      bool msg10_available = cnav_msg_get(sid, CNAV_MSG_TYPE_10, &cnav_msg10);
      if (msg10_available && !cnav_msg10.data.type_10.l2_health) {
        return CODE_NAV_STATE_INVALID;
      }

      if ((shis.shi1_set && check_6bit_health_word(shis.shi1, sid.code)) &&
          (msg10_available && cnav_msg10.data.type_10.l2_health) &&
          (shis.shi6_set && shis.shi6)) {
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
      * - almanac health bits are available and indicate L1 unhealthy
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
      if (shis.shi4_set && !shis.shi4) {
        return CODE_NAV_STATE_INVALID;
      }

      cnav_msg_t cnav_msg10;
      bool msg10_available = cnav_msg_get(sid, CNAV_MSG_TYPE_10, &cnav_msg10);
      if (msg10_available && !cnav_msg10.data.type_10.l2_health) {
        return CODE_NAV_STATE_INVALID;
      }

      if ((shis.shi1_set && check_6bit_health_word(shis.shi1, sid.code)) &&
          (shis.shi4_set && shis.shi4) &&
          ((!shis.shi6_set && !msg10_available) ||
           (msg10_available && cnav_msg10.data.type_10.l2_health))) {
        return CODE_NAV_STATE_VALID;
      }

      return CODE_NAV_STATE_UNKNOWN;
    }

    /*
    * Same functionality applies for both GLO signals.
    *
    * Return SV_NAV_STATE_INVALID if:
    * - SHI is available and indicates signal unhealthy
    *
    * Return CODE_NAV_STATE_VALID if:
    * - SHI is available and indicates signal healthy
    *
    * Return CODE_NAV_STATE_UNKNOWN otherwise
    */
    case CODE_GLO_L1OF:
    case CODE_GLO_L2OF:
      if (shi.shi_set && (SV_UNHEALTHY == shi.shi)) {
        return CODE_NAV_STATE_INVALID;
      } else if (shi.shi_set && (SV_HEALTHY == shi.shi)) {
        return CODE_NAV_STATE_VALID;
      }
      return CODE_NAV_STATE_UNKNOWN;

    /*
    * Same functionality applies for both BDS signals.
    *
    * Return SV_NAV_STATE_INVALID if:
    * - SHI is available and indicates signal unhealthy
    *
    * Return CODE_NAV_STATE_VALID if:
    * - SHI is available and indicates signal healthy
    *
    * Return CODE_NAV_STATE_UNKNOWN otherwise
    */
    case CODE_BDS2_B11:
    case CODE_BDS2_B2:
      if (shi_bds.shi_set && (SV_UNHEALTHY == shi_bds.shi)) {
        return CODE_NAV_STATE_INVALID;
      } else if (shi_bds.shi_set && (SV_HEALTHY == shi_bds.shi)) {
        return CODE_NAV_STATE_VALID;
      }
      return CODE_NAV_STATE_UNKNOWN;

    case CODE_GPS_L2CL:
      return CODE_NAV_STATE_UNKNOWN;
    case CODE_SBAS_L1CA:
      assert(!"Unsupported code");

    case CODE_INVALID:
    case CODE_COUNT:
    case CODE_GPS_L2CX:
    case CODE_GPS_L5I:
    case CODE_GPS_L5Q:
    case CODE_GPS_L5X:
    case CODE_GAL_E1B:
    case CODE_GAL_E1C:
    case CODE_GAL_E1X:
    case CODE_GAL_E6B:
    case CODE_GAL_E6C:
    case CODE_GAL_E6X:
    case CODE_GAL_E7I:
    case CODE_GAL_E7Q:
    case CODE_GAL_E7X:
    case CODE_GAL_E8:
    case CODE_GAL_E5I:
    case CODE_GAL_E5Q:
    case CODE_GAL_E5X:
    case CODE_QZS_L1CA:
    case CODE_QZS_L2CM:
    case CODE_QZS_L2CL:
    case CODE_QZS_L2CX:
    case CODE_QZS_L5I:
    case CODE_QZS_L5Q:
    case CODE_QZS_L5X:
    default:
      assert(!"Invalid code");
  }
  return CODE_NAV_STATE_UNKNOWN;
}

/** Output current health state of GPS satellite to the log.
 *  Function does nothing if DEBUG is off.
 *
 * \param shi_name Name of the SHI that was changed last.
 * \param sat GPS satellite ID for which current state should be logged
 *
 */
void shm_log_sat_state(const char* shi_name, u16 sat) {
  if (DEBUG) {
    code_nav_state_t s_l1 =
        shm_get_sat_state(construct_sid(CODE_GPS_L1CA, sat));
    gnss_signal_t l2_sid = construct_sid(CODE_GPS_L2CM, sat);
    code_nav_state_t s_l2 = shm_get_sat_state(l2_sid);
    char* s_l1_str;
    char* s_l2_str;
    cns_2_str(s_l1, &s_l1_str);
    cns_2_str(s_l2, &s_l2_str);
    char shi1_str[4], shi4_str[2], shi6_str[2];
    char shi5_l1_str[2], shi5_l2_str[2], shi5_l5_str[2];
    chMtxLock(&shm_data_access);
    gps_sat_health_indicators_t shis = gps_shis[sat - GPS_FIRST_PRN];
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
    log_debug(
        "GPS SV %02d %s update. State {L1:%s, L2:%s} "
        "SHI[1:%s, 4:%s, 5:{%s,%s,%s}, 6:%s]",
        sat,
        shi_name,
        s_l1_str,
        s_l2_str,
        shi1_str,
        shi4_str,
        shi5_l1_str,
        shi5_l2_str,
        shi5_l5_str,
        shi6_str);
  }
}

/** Update SHI1 for GPS satellite.
 *  Refer to libswiftnav/shm.h for details of SHIs.
 *
 * \param sat GPS satellite ID
 * \param new_value value to set SHI1 to
 */
void shm_gps_set_shi1(u16 sat, u8 new_value) {
  assert(sat >= GPS_FIRST_PRN && sat < GPS_FIRST_PRN + NUM_SATS_GPS);
  chMtxLock(&shm_data_access);
  gps_shis[sat - GPS_FIRST_PRN].shi1 = new_value;
  gps_shis[sat - GPS_FIRST_PRN].shi1_set = true;
  chMtxUnlock(&shm_data_access);
  shm_log_sat_state("SHI1", sat);
}

/** Update SHI4 for GPS satellite.
 *  Refer to libswiftnav/shm.h for details of SHIs.
 *
 * \param sat GPS satellite ID
 * \param new_value value to set SHI4 to
 */
void shm_gps_set_shi4(u16 sat, bool new_value) {
  assert(sat >= GPS_FIRST_PRN && sat < GPS_FIRST_PRN + NUM_SATS_GPS);
  chMtxLock(&shm_data_access);
  gps_shis[sat - GPS_FIRST_PRN].shi4 = new_value;
  gps_shis[sat - GPS_FIRST_PRN].shi4_set = true;
  chMtxUnlock(&shm_data_access);
  shm_log_sat_state("SHI4", sat);
}

/** Update SHI6 for GPS satellite.
 *  Refer to libswiftnav/shm.h for details of SHIs.
 *
 * \param sat GPS satellite ID
 * \param new_value value to set SHI6 to
 */
void shm_gps_set_shi6(u16 sat, bool new_value) {
  assert(sat >= GPS_FIRST_PRN && sat < GPS_FIRST_PRN + NUM_SATS_GPS);
  chMtxLock(&shm_data_access);
  gps_shis[sat - GPS_FIRST_PRN].shi6 = new_value;
  gps_shis[sat - GPS_FIRST_PRN].shi6_set = true;
  chMtxUnlock(&shm_data_access);
  shm_log_sat_state("SHI6", sat);
}

/** Update SHI for GLO satellite.
 *  Refer to libswiftnav/shm.h for details of SHI.
 *
 * \param sat GLO satellite ID
 * \param new_value value to set SHI to
 */
void shm_glo_set_shi(u16 sat, u8 new_value) {
  assert(sat >= GLO_FIRST_PRN && sat < GLO_FIRST_PRN + NUM_SATS_GLO);
  chMtxLock(&shm_data_access);
  glo_shis[sat - GLO_FIRST_PRN].shi = new_value;
  glo_shis[sat - GLO_FIRST_PRN].shi_set = true;
  chMtxUnlock(&shm_data_access);
}

/** Update SHI for BDS satellite.
 *  Refer to libswiftnav/shm.h for details of SHI.
 *
 * \param sat BDS satellite ID
 * \param new_value value to set SHI to
 */
void shm_bds_set_shi(u16 sat, u8 new_value) {
  assert(sat >= BDS2_FIRST_PRN && sat < BDS2_FIRST_PRN + NUM_SATS_BDS2);
  chMtxLock(&shm_data_access);
  bds_shis[sat - BDS2_FIRST_PRN].shi = new_value;
  bds_shis[sat - BDS2_FIRST_PRN].shi_set = true;
  chMtxUnlock(&shm_data_access);
}

/** Check if this ephemeris is healthy
 *
 * \param ephe Ephemeris
 * \param code signal code, ephe->sid can't be used as for example L2CM uses
 *             L1CA ephes
 * \return true if the ephemeris is healthy
 *         false otherwise
 */
bool shm_ephe_healthy(const ephemeris_t* ephe, const code_t code) {
  return ephemeris_healthy(ephe, code);
}

/** Check if signal is healthy.
 *
 * \param sid Signal ID
 *
 * \returns true if signal health of specified signal
 *               is CODE_NAV_STATE_INVALID, false otherwise
 */
bool shm_signal_healthy(gnss_signal_t sid) {
  assert(sid_valid(sid));
  return shm_get_sat_state(sid) == CODE_NAV_STATE_VALID;
}

/** Check if signal is unhealthy.
 *
 * \param sid Signal ID
 *
 * \returns true if signal health of specified signal
 *               is CODE_NAV_STATE_INVALID, false otherwise
 */
bool shm_signal_unhealthy(gnss_signal_t sid) {
  assert(sid_valid(sid));
  return shm_get_sat_state(sid) == CODE_NAV_STATE_INVALID;
}

/** Check if signal is suitable for navigation
 *
 * \param sat Signal ID
 *
 * \returns true if health of signal is CODE_NAV_STATE_VALID,
 *          false otherwise
 */
bool shm_navigation_suitable(gnss_signal_t sid) {
  return shm_get_sat_state(sid) == CODE_NAV_STATE_VALID;
}

/** Check if signal is unusable for navigation
 *
 * \param sat Signal ID
 *
 * \returns true if health of signal is not CODE_NAV_STATE_VALID,
 *          false otherwise
 */
bool shm_navigation_unusable(gnss_signal_t sid) {
  return shm_get_sat_state(sid) != CODE_NAV_STATE_VALID;
}

/** Check if signal health is unknown
 *
 * \param sat Signal ID
 *
 * \returns true if health of signal is not CODE_NAV_STATE_UNKNOWN,
 *          false otherwise
 */
bool shm_health_unknown(gnss_signal_t sid) {
  return shm_get_sat_state(sid) == CODE_NAV_STATE_UNKNOWN;
}
