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
#include "piksi_systime.h"
#include "shm.h"
#include "signal_db/signal_db.h"

static MUTEX_DECL(shm_data_access);

#define CODE_NAV_STATE_UNKNOWN_STR "CODE_NAV_STATE_UNKNOWN"
#define CODE_NAV_STATE_INVALID_STR "CODE_NAV_STATE_INVALID"
#define CODE_NAV_STATE_VALID_STR "CODE_NAV_STATE_VALID"

static gps_sat_health_indicators_t gps_shis[NUM_SATS_GPS];
static glo_sat_health_indicators_t glo_shis[NUM_SATS_GLO];
static bds_sat_health_indicators_t bds_shis[NUM_SATS_BDS];
static gal_sat_health_indicators_t gal_shis[NUM_SATS_GAL];

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
      ASSERT(!"Unsupported value in code_nav_state_t");
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

  ASSERT(sid_valid(sid));

  /* Check GPS band specific SHIs.
   * shi_ephemeris, LNAV SV HEALTH (6 bits, subframe 1, word 3)
   * shi_lnav_how_alert, LNAV alert flag (HOW, bit 18)
   * cnav_msg10, CNAV L1, L2 [and L5] health (message type 10 bits 52..54)
   * shi_cnav_alert. CNAV alert flag (bit 38, each message)
   */

  /* Check GLO SHI.
   * SHI. SV HEALTH (MSB of B || l).
   */

  /* Retrieve SHI data */
  chMtxLock(&shm_data_access);
  gps_sat_health_indicators_t shis = gps_shis[sid.sat - GPS_FIRST_PRN];
  glo_sat_health_indicators_t shi = glo_shis[sid.sat - GLO_FIRST_PRN];
  bds_sat_health_indicators_t shi_bds = bds_shis[sid.sat - BDS_FIRST_PRN];
  gal_sat_health_indicators_t shi_gal = gal_shis[sid.sat - GAL_FIRST_PRN];
  chMtxUnlock(&shm_data_access);

  /* Check common GPS */
  if (IS_GPS(sid)) {
    if (shis.shi_ephemeris_set &&
        !check_6bit_health_word(shis.shi_ephemeris, sid.code)) {
      return CODE_NAV_STATE_INVALID;
    }

    /* Return SV_NAV_STATE_INVALID if:
     * - shi_page25_set is set and
     * - shi_page25_set has not aged and
     * - shi_page25_set indicates unhealthy
     */
    if (shis.shi_page25_set &&
        !shm_alma_page25_health_aged(shis.shi_page25_timetag_s) &&
        !check_alma_page25_health_word(shis.shi_page25, sid.code)) {
      return CODE_NAV_STATE_INVALID;
    }

    almanac_t a;
    ndb_op_code_t oc = ndb_almanac_read(sid, &a);
    if (NDB_ERR_NONE == oc &&
        !check_8bit_health_word(a.health_bits, sid.code)) {
      return CODE_NAV_STATE_INVALID;
    }
  }

  switch ((s8)sid.code) {
    case CODE_GPS_L1CA:
    case CODE_GPS_L1P: {
      /*
      * Return SV_NAV_STATE_INVALID if either of the following:
      * - shi_ephemeris_set is available and indicates L1CA/L1P unhealthy
      * - shi_lnav_how_alert is available and negative
      * - cnav_msg10 is available and indicates L1 unhealthy
      * - almanac health bits are available and indicate L1CA/L1P unhealthy
      *
      * Return CODE_NAV_STATE_VALID all conditions below are true:
      * - shi_ephemeris_set is available and indicates L1CA/L1P healthy
      * - shi_lnav_how_alert is available and positive
      * - One of the following:
      *     - cnav_msg10 is unavailable
      *     - cnav_msg10 is available and indicates L1 healthy
      *
      * Otherwise return CODE_NAV_STATE_UNKNOWN
      */
      if (shis.shi_lnav_how_alert_set && !shis.shi_lnav_how_alert) {
        return CODE_NAV_STATE_INVALID;
      }

      cnav_msg_t cnav_msg10;
      bool msg10_available = cnav_msg_get(sid, CNAV_MSG_TYPE_10, &cnav_msg10);
      if (msg10_available && !cnav_msg10.data.type_10.l1_health) {
        return CODE_NAV_STATE_INVALID;
      }

      if ((shis.shi_ephemeris_set &&
           check_6bit_health_word(shis.shi_ephemeris, sid.code)) &&
          (shis.shi_lnav_how_alert_set && shis.shi_lnav_how_alert) &&
          ((!msg10_available) ||
           (msg10_available && cnav_msg10.data.type_10.l1_health))) {
        return CODE_NAV_STATE_VALID;
      }

      return CODE_NAV_STATE_UNKNOWN;
    }

    case CODE_GPS_L2CM:
    case CODE_GPS_L2CL:
    case CODE_GPS_L2CX:
    case CODE_GPS_L2P: {
      /*
       * Return CODE_NAV_STATE_INVALID if either of the following:
       * - shi_ephemeris is available and indicates L2C/L2P unhealthy
       * - shi_cnav_alert is available and negative
       * - cnav_msg10 is available and indicates L2 unhealthy
       * - almanac health bits are available and indicate L2C/L2P unhealthy
       *
       * Return CODE_NAV_STATE_VALID if all conditions below are true:
       * - shi_ephemeris is available and indicates L2C/L2P healthy
       * - cnav_msg10 is available and indicates L2 healthy
       * - shi_cnav_alert is available and positive
       *
       * Otherwise return CODE_NAV_STATE_UNKNOWN
       */
      if (shis.shi_cnav_alert_set && !shis.shi_cnav_alert) {
        return CODE_NAV_STATE_INVALID;
      }

      cnav_msg_t cnav_msg10;
      bool msg10_available = cnav_msg_get(sid, CNAV_MSG_TYPE_10, &cnav_msg10);
      if (msg10_available && !cnav_msg10.data.type_10.l2_health) {
        return CODE_NAV_STATE_INVALID;
      }

      if ((shis.shi_ephemeris_set &&
           check_6bit_health_word(shis.shi_ephemeris, sid.code)) &&
          (shis.shi_cnav_alert_set && shis.shi_cnav_alert)) {
        return CODE_NAV_STATE_VALID;
      }

      return CODE_NAV_STATE_UNKNOWN;
    }

    case CODE_GLO_L1OF:
    case CODE_GLO_L2OF:
      if (shi.shi_set && (SV_UNHEALTHY == shi.shi)) {
        return CODE_NAV_STATE_INVALID;
      } else if (shi.shi_set && (SV_HEALTHY == shi.shi)) {
        return CODE_NAV_STATE_VALID;
      }
      return CODE_NAV_STATE_UNKNOWN;

    case CODE_BDS2_B1:
    case CODE_BDS2_B2:
      if (shi_bds.shi_set && (SV_UNHEALTHY == shi_bds.shi)) {
        return CODE_NAV_STATE_INVALID;
      } else if (shi_bds.shi_set && (SV_HEALTHY == shi_bds.shi)) {
        return CODE_NAV_STATE_VALID;
      }
      return CODE_NAV_STATE_UNKNOWN;

    case CODE_SBAS_L1CA:
      return CODE_NAV_STATE_UNKNOWN;

    case CODE_GAL_E1B:
    case CODE_GAL_E1C:
    case CODE_GAL_E1X:
    case CODE_GAL_E7I:
    case CODE_GAL_E7Q:
    case CODE_GAL_E7X:
      if (shi_gal.shi_set && (SV_UNHEALTHY == shi_gal.shi)) {
        return CODE_NAV_STATE_INVALID;
      } else if (shi_gal.shi_set && (SV_HEALTHY == shi_gal.shi)) {
        return CODE_NAV_STATE_VALID;
      }
      return CODE_NAV_STATE_UNKNOWN;

    case CODE_INVALID:
    case CODE_COUNT:
      ASSERT(!"Invalid code");
      break;

    default:
      return CODE_NAV_STATE_UNKNOWN;
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
    char shi_ephemeris_str[4], shi_lnav_how_alert_str[2], shi_cnav_alert_str[2];
    char shi5_l1_str[2], shi5_l2_str[2], shi5_l5_str[2];
    chMtxLock(&shm_data_access);
    gps_sat_health_indicators_t shis = gps_shis[sat - GPS_FIRST_PRN];
    chMtxUnlock(&shm_data_access);
    int_shi_2_str(shis.shi_ephemeris_set,
                  shis.shi_ephemeris,
                  shi_ephemeris_str,
                  sizeof(shi_ephemeris_str));
    bool_shi_2_str(shis.shi_lnav_how_alert_set,
                   shis.shi_lnav_how_alert,
                   shi_lnav_how_alert_str);
    cnav_msg_t cnav_msg10;
    bool shi5_set = cnav_msg_get(l2_sid, CNAV_MSG_TYPE_10, &cnav_msg10);
    cnav_msg_type_10_t m10 = cnav_msg10.data.type_10;
    bool_shi_2_str(shi5_set, m10.l1_health, shi5_l1_str);
    bool_shi_2_str(shi5_set, m10.l2_health, shi5_l2_str);
    bool_shi_2_str(shi5_set, m10.l5_health, shi5_l5_str);
    bool_shi_2_str(
        shis.shi_cnav_alert_set, shis.shi_cnav_alert, shi_cnav_alert_str);
    log_debug(
        "GPS SV %02d %s update. State {L1:%s, L2:%s} "
        "SHI[1:%s, 4:%s, 5:{%s,%s,%s}, 6:%s]",
        sat,
        shi_name,
        s_l1_str,
        s_l2_str,
        shi_ephemeris_str,
        shi_lnav_how_alert_str,
        shi5_l1_str,
        shi5_l2_str,
        shi5_l5_str,
        shi_cnav_alert_str);
  }
}

/** Update shi_ephemeris for GPS satellite.
 *  Refer to libswiftnav/shm.h for details of SHIs.
 *
 * \param sat GPS satellite ID
 * \param new_value value to set shi_ephemeris to
 */
void shm_gps_set_shi_ephemeris(u16 sat, u8 new_value) {
  ASSERT(sat >= GPS_FIRST_PRN && sat < GPS_FIRST_PRN + NUM_SATS_GPS);
  chMtxLock(&shm_data_access);
  gps_shis[sat - GPS_FIRST_PRN].shi_ephemeris = new_value;
  gps_shis[sat - GPS_FIRST_PRN].shi_ephemeris_set = true;
  chMtxUnlock(&shm_data_access);
  shm_log_sat_state("shi_ephemeris", sat);
}

/** Update shi_page25 for GPS satellite.
 *  Refer to libswiftnav/shm.h for details of SHIs.
 *
 * \param sat GPS satellite ID
 * \param new_value value to set shi_page25 to
 */
void shm_gps_set_shi_page25(u16 sat, u8 new_value) {
  ASSERT(sat >= GPS_FIRST_PRN && sat < GPS_FIRST_PRN + NUM_SATS_GPS);
  if (0 == new_value) {
    return;
  }
  piksi_systime_t now;
  piksi_systime_get(&now);
  u32 timetag_s = piksi_systime_to_s(&now);
  chMtxLock(&shm_data_access);
  gps_shis[sat - GPS_FIRST_PRN].shi_page25 = new_value;
  gps_shis[sat - GPS_FIRST_PRN].shi_page25_set = true;
  gps_shis[sat - GPS_FIRST_PRN].shi_page25_timetag_s = timetag_s;
  chMtxUnlock(&shm_data_access);
}

/** Update shi_lnav_how_alert for GPS satellite.
 *  Refer to libswiftnav/shm.h for details of SHIs.
 *
 * \param sat GPS satellite ID
 * \param new_value value to set shi_lnav_how_alert to
 */
void shm_gps_set_shi_lnav_how_alert(u16 sat, bool new_value) {
  ASSERT(sat >= GPS_FIRST_PRN && sat < GPS_FIRST_PRN + NUM_SATS_GPS);
  chMtxLock(&shm_data_access);
  gps_shis[sat - GPS_FIRST_PRN].shi_lnav_how_alert = new_value;
  gps_shis[sat - GPS_FIRST_PRN].shi_lnav_how_alert_set = true;
  chMtxUnlock(&shm_data_access);
  shm_log_sat_state("shi_lnav_how_alert", sat);
}

/** Update shi_cnav_alert for GPS satellite.
 *  Refer to libswiftnav/shm.h for details of SHIs.
 *
 * \param sat GPS satellite ID
 * \param new_value value to set shi_cnav_alert to
 */
void shm_gps_set_shi_cnav_alert(u16 sat, bool new_value) {
  ASSERT(sat >= GPS_FIRST_PRN && sat < GPS_FIRST_PRN + NUM_SATS_GPS);
  chMtxLock(&shm_data_access);
  gps_shis[sat - GPS_FIRST_PRN].shi_cnav_alert = new_value;
  gps_shis[sat - GPS_FIRST_PRN].shi_cnav_alert_set = true;
  chMtxUnlock(&shm_data_access);
  shm_log_sat_state("shi_cnav_alert", sat);
}

/** Update SHI for GLO satellite.
 *  Refer to libswiftnav/shm.h for details of SHI.
 *
 * \param sat GLO satellite ID
 * \param new_value value to set SHI to
 */
void shm_glo_set_shi(u16 sat, u8 new_value) {
  ASSERT(sat >= GLO_FIRST_PRN && sat < GLO_FIRST_PRN + NUM_SATS_GLO);
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
  ASSERT(sat >= BDS_FIRST_PRN && sat < BDS_FIRST_PRN + NUM_SATS_BDS);
  chMtxLock(&shm_data_access);
  bds_shis[sat - BDS_FIRST_PRN].shi = new_value;
  bds_shis[sat - BDS_FIRST_PRN].shi_set = true;
  chMtxUnlock(&shm_data_access);
}

/** Update SHI for GAL satellite.
 *  Refer to libswiftnav/shm.h for details of SHI.
 *
 * \param sat GAL satellite ID
 * \param new_value value to set SHI to
 */
void shm_gal_set_shi(u16 sat, u8 new_value) {
  ASSERT(sat >= GAL_FIRST_PRN && sat < GAL_FIRST_PRN + NUM_SATS_GAL);
  chMtxLock(&shm_data_access);
  gal_shis[sat - GAL_FIRST_PRN].shi = new_value;
  gal_shis[sat - GAL_FIRST_PRN].shi_set = true;
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

/** Check if this almanac page 25 health indicator has aged
 *
 * \param timetag_s Timetag of reception.
 * \return true if the page 25 health indicator has aged.
 *         false otherwise
 */
bool shm_alma_page25_health_aged(u32 timetag_s) {
  piksi_systime_t now;
  piksi_systime_get(&now);
  u32 timetag_now_s = piksi_systime_to_s(&now);
  if ((timetag_now_s - timetag_s) > SHM_ALMA_PAGE25_HEALTH_AGE_S) {
    return true;
  }
  return false;
}

/** Check if signal is healthy.
 *
 * \param sid Signal ID
 *
 * \returns true if signal health of specified signal
 *               is CODE_NAV_STATE_VALID, false otherwise
 */
bool shm_signal_healthy(gnss_signal_t sid) {
  ASSERT(sid_valid(sid));
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
  ASSERT(sid_valid(sid));
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
