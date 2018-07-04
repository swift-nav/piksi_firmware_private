/*
 * Copyright (C) 2011 - 2017 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *          Gareth McMullin <gareth@swiftnav.com>
 *          Pasi Miettinen <pasi.miettinen@exafore.com>
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
#include <libswiftnav/glo_map.h>
#include <libswiftnav/linear_algebra.h>
#include <libswiftnav/logging.h>
#include <string.h>

#include "ephemeris.h"
#include "ndb/ndb.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "signal_db/signal_db.h"
#include "timing/timing.h"
#include "track/track_flags.h"
#include "track/track_sid_db.h"
#include "track/track_utils.h"

/** Maximum distance for ephemeris-almanac cross-correlation tests [m] */
#define XCORR_MAX_EA_DISTANCE_M 50000.0f

/**
 * Computes cross-correlation positions from NDB almanac
 *
 * \param[in]  sid        GNSS signal identifier.
 * \param[in]  time_s     GPS time for a central position [s].
 * \param[in]  interval_s Time interval between positions position [s].
 * \param[out] pos        Resulting ECEF position
 *
 * \retval true  Positions computed
 * \retval false No suitable almanac or algorithm error
 */
bool xcorr_calc_alm_positions(gnss_signal_t sid,
                              u32 time_s,
                              u32 interval_s,
                              xcorr_positions_t *pos) {
  assert(IS_GPS(sid));

  almanac_t a;
  ndb_op_code_t oc = ndb_almanac_read(sid, &a);
  /* Here we do not care if GPS time is unknown
   * since almanac is used with input time_s. */
  bool data_elem_valid = (NDB_ERR_NONE == oc || NDB_ERR_GPS_TIME_MISSING == oc);

  if (!data_elem_valid) {
    return false;
  }

  gps_time_t t0 = make_gps_time(time_s - interval_s);
  gps_time_t t1 = make_gps_time(time_s);
  gps_time_t t2 = make_gps_time(time_s + interval_s);

  if (!almanac_valid(&a, &t0) || !almanac_valid(&a, &t2)) {
    return false;
  }

  double _[3];
  calc_sat_state_almanac(&a, &t0, pos->early.xyz, _, _, _, _);
  calc_sat_state_almanac(&a, &t1, pos->prompt.xyz, _, _, _, _);
  calc_sat_state_almanac(&a, &t2, pos->late.xyz, _, _, _, _);
  pos->time_s = time_s;
  pos->interval_s = interval_s;
  return true;
}

/**
 * Computes cross-correlation positions from ephemeris
 *
 * \param[in]  e      Ephemeris to use
 * \param[in]  time_s GPS time for a central position [s].
 * \param[out] pos    Resulting ECEF position
 *
 * \retval true  Positions computed
 * \retval false No suitable ephemeris or algorithm error
 */
bool xcorr_calc_eph_positions(const ephemeris_t *e,
                              u32 time_s,
                              xcorr_positions_t *pos) {
  u32 interval_s = e->fit_interval / 2;
  gps_time_t t0 = make_gps_time(time_s - interval_s);
  gps_time_t t1 = make_gps_time(time_s);
  gps_time_t t2 = make_gps_time(time_s + interval_s);

  if (!ephemeris_valid(e, &t0) || !ephemeris_valid(e, &t2)) {
    return false;
  }

  double _[3];
  u8 iode;
  u16 iodc;
  calc_sat_state_n(e, &t0, pos->early.xyz, _, _, _, _, &iodc, &iode);
  calc_sat_state_n(e, &t1, pos->prompt.xyz, _, _, _, _, &iodc, &iode);
  calc_sat_state_n(e, &t2, pos->late.xyz, _, _, _, _, &iodc, &iode);
  pos->time_s = time_s;
  pos->interval_s = interval_s;
  return true;
}

/**
 * Loads cached cross-correlation positions from SID cache or computes it from
 * NDB almanac.
 *
 * \param[in]  sid        GNSS signal identifier.
 * \param[in]  time_s     GPS time for a central position [s].
 * \param[in]  interval_s Time interval between positions [s].
 * \param[out] pos        Resulting ECEF position
 *
 * \retval true  Positions computed or retrieved for cache.
 * \retval false No suitable almanac or algorithm error.
 */
bool xcorr_get_alm_positions(gnss_signal_t sid,
                             u32 time_s,
                             u32 interval_s,
                             xcorr_positions_t *pos) {
  if (!track_sid_db_load_positions(sid, pos)) {
    return false;
  }

  if (pos->time_s == time_s && pos->interval_s == interval_s) {
    return true;
  }

  bool res = xcorr_calc_alm_positions(sid, time_s, interval_s, pos);
  if (res) {
    track_sid_db_update_positions(sid, pos);
  }

  return res;
}

/**
 * Tests if two position sets are close to each other.
 *
 * \param[in] sid0 GNSS signal identifier for \a pos0
 * \param[in] sid1 GNSS signal identifier for \a pos1
 * \param[in] pos0 Positions to compare
 * \param[in] pos1 Positions to compare
 *
 * \retval true If the position errors are less than the threshold
 * \retval false If at least one position has an error above the threshold
 */
bool xcorr_match_positions(gnss_signal_t sid0,
                           gnss_signal_t sid1,
                           const xcorr_positions_t *pos0,
                           const xcorr_positions_t *pos1) {
  bool ok = true;
  double d[3] = {-1, -1, -1};
  for (u8 i = 0; i < 3 && ok; i++) {
    d[i] = vector_distance(3, pos0->epl[i].xyz, pos1->epl[i].xyz);
    ok = (d[i] <= XCORR_MAX_EA_DISTANCE_M);
  }

  char sid_str_[SID_STR_LEN_MAX];
  sid_to_string(sid_str_, sizeof(sid_str_), sid1);
  log_debug_sid(
      sid0, "-> %s distance: %le, %le, %le", sid_str_, d[0], d[1], d[2]);
  return ok;
}

/**
 * Checks if the given position set matches almanac.
 *
 * \param[in] sid0 GNSS signal identifier for \a pos
 * \param[in] sid GNSS signal identifier for almanac
 * \param[in] pos Position to compare against
 *
 * \return Comparison result
 * \retval XCORR_MATCH_RES_OK         Position match detected
 * \retval XCORR_MATCH_RES_NO_ALMANAC No almanac or algorithm error
 * \retval XCORR_MATCH_RES_NO_MATCH   Positions do not match
 *
 * \sa xcorr_match_positions
 * \sa xcorr_get_alm_positions
 */
xcorr_match_res_t xcorr_match_alm_position(gnss_signal_t sid0,
                                           gnss_signal_t sid,
                                           const xcorr_positions_t *pos) {
  xcorr_positions_t alm_pos;
  if (!xcorr_get_alm_positions(sid, pos->time_s, pos->interval_s, &alm_pos)) {
    return XCORR_MATCH_RES_NO_ALMANAC;
  }
  if (xcorr_match_positions(sid, sid0, &alm_pos, pos)) {
    return XCORR_MATCH_RES_OK;
  } else {
    return XCORR_MATCH_RES_NO_MATCH;
  }
}

/**
 * Helper function. Delete ghost ephemeris associated with SID
 * \param sid GNSS SID
 */
static void delete_ghost_ephe(const gnss_signal_t sid) {
  ndb_op_code_t c = ndb_ephemeris_erase(sid);
  if (c == NDB_ERR_NONE) {
    log_info_sid(sid, "removed from NDB as ghost");
  } else {
    log_error_sid(sid, "cannot delete ephemeris from NDB, code %" PRIu8 " ", c);
  }
}

/**
 * The function compares ephemeris against other ones stored in NDB for same
 * GNSS. The function is used to check if the ephemeris in question is result of
 * incorrect decoding due to cross-correlation.
 * \param e Pointer to ephemeris to be checked.
 * \return true if ephemeris in question is duplicate of other one, otherwise
 *         returns false.
 */
static bool xcorr_check_eph_to_eph(const ephemeris_t *e) {
  assert(e != NULL);

  ephemeris_t test_e;
  u16 first_prn, num_sats;
  char *gnss = "";

  if (IS_GPS(e->sid)) {
    first_prn = GPS_FIRST_PRN;
    num_sats = NUM_SATS_GPS;
    gnss = "GPS";
  } else if (IS_BDS2(e->sid)) {
    first_prn = BDS2_FIRST_PRN;
    num_sats = NUM_SATS_BDS2;
    gnss = "BDS";
  } else if (IS_GAL(e->sid)) {
    first_prn = GAL_FIRST_PRN;
    num_sats = NUM_SATS_GAL;
    gnss = "GAL";
  } else if (IS_GLO(e->sid)) {
    /* Checking for GLO ephemeris is useless, because
     * 1) No doppler x-corr can be happen due to frequency division
     * 2) The probability of getting the same ephemeris from different SV due to
     * incorrect decoding is vanishingly small
     * 3) The chance we incorrectly decode slot_id twice is small, so no
     * ephemeris overwriting happens */
    return false;
  } else {
    log_warn("Unsupported GNSS. Ephemeris-to-ephemeris check skipped");
    return false;
  }

  for (u16 i = first_prn; i <= num_sats; i++) {
    if (e->sid.sat == i) {
      /* don't compare with itself */
      continue;
    }
    ndb_op_code_t res =
        ndb_ephemeris_read(construct_sid(e->sid.code, i), &test_e);
    if (NDB_ERR_NONE != res) {
      if (NDB_ERR_BAD_PARAM == res) {
        log_error_sid(e->sid, "Ephemeris NDB read error: bad param");
      }
      /* some problems when read ephemeres from NDB, try next */
      continue;
    }
    gnss_signal_t backup = test_e.sid;
    /* Fake SID before checking, because we want ephemeris_equal compares
     * everything but sid */
    test_e.sid = e->sid;
    if (!ephemeris_equal(e, &test_e)) {
      continue;
    }
    /* restore sid */
    test_e.sid = backup;
    /* Next step is to make sure that stored ephemeris was from real SV
    So, first, check if stored SV is being tracked */
    tracker_t *tc_test = NULL;
    tc_test = tracker_channel_get_by_mesid(
        construct_mesid(test_e.sid.code, test_e.sid.sat));
    if (tc_test) {
      /* stored SV is still being tracked */
      tracker_t *tc_new = NULL;
      tc_new = tracker_channel_get_by_mesid(
          construct_mesid(e->sid.code, e->sid.sat));
      assert(tc_new != NULL);
      /* now check which SV has stronger signal, consider that
       * stronger signal belongs to real SV */
      if (tc_test->cn0 < tc_new->cn0) {
        /* Stored SV is ghost one, so remove it from NDB */
        delete_ghost_ephe(test_e.sid);
        /* and immediately drop tracked ghost signal */
        tracker_set_xcorr_flag(tc_test->mesid);
        return false; /* exit and notify that new ephemeris from real SV and
                         it can be stored in NDB */
      } else {
        /* new ephemeris is Ghost */
        log_info_sid(e->sid,
                     "Ephemeris to ephemeris x-corr suspect. "
                     "Real is %s SV %d",
                     gnss,
                     test_e.sid.sat);
        return true;
      }
    } else {
      /* stored SV is not tracked, remove it from NDB and mark new one as
       * cross-correlated as well, because we actually don't know which one is
       * ghost */
      delete_ghost_ephe(test_e.sid);
      return true;
    }
  }
  return false;
}

/* Compute satellite azimuth and elevation and update the track db
 *
 * \param e ephemeris
 * \param t time at which to calculate the az/el.
 * \param pos_ecef coordinates of the reference point
 *
 * \return  0 on success,
 *         -1 if ephemeris is invalid or time is outside its fit interval
 */
s8 update_azel_from_ephemeris(const ephemeris_t *e,
                              const gps_time_t *t,
                              const double pos_ecef[]) {
  if (!ephemeris_valid(e, t)) {
    return -1;
  }
  double az, el;
  if (0 != calc_sat_az_el(e, t, pos_ecef, &az, &el, false)) {
    return -1;
  }
  track_sid_db_azel_degrees_set(
      e->sid, round(az * R2D), round(el * R2D), nap_timing_count());
  log_debug_sid(e->sid, "Updated elevation from ephemeris %.1f deg", el * R2D);
  return 0;
}

/* Compute satellite azimuth and elevation and update the track db
 *
 * \param a almanac
 * \param t time at which to calculate the az/el.
 * \param pos_ecef coordinates of the reference point
 *
 * \return  0 on success,
 *         -1 if almanac is invalid or time is outside its fit interval
 */
s8 update_azel_from_almanac(const almanac_t *a,
                            const gps_time_t *t,
                            const double pos_ecef[]) {
  if (!almanac_valid(a, t)) {
    return -1;
  }
  double az, el;
  if (0 != calc_sat_az_el_almanac(a, t, pos_ecef, &az, &el)) {
    return -1;
  }
  track_sid_db_azel_degrees_set(
      a->sid, round(az * R2D), round(el * R2D), nap_timing_count());
  log_debug_sid(a->sid, "Updated elevation from almanac %.1f deg", el * R2D);
  return 0;
}

/**
 * Checks that new GPS ephemeris matches its own almanac and none of the other
 * satellites, and if so, pass to it to NDB.
 *
 * \param[in] e New ephemeris structure
 *
 * \return Store result
 * \retval EPH_NEW_OK    New ephemeris passed to NDB
 * \retval EPH_NEW_ERR   Ephemeris invalid or failed to check against almanac
 * \retval EPH_NEW_XCORR Ephemeris matches some other satellite's almanac
 *
 */
eph_new_status_t ephemeris_new(const ephemeris_t *e) {
  if (!sid_supported(e->sid)) {
    /* throw debug message prior to dying */
    log_error_sid(e->sid,
                  "SID not supported, toe %4d %8.1f  valid %u  health %u",
                  e->toe.wn,
                  e->toe.tow,
                  e->valid,
                  e->health_bits);
  }
  assert(sid_supported(e->sid));

  if (!e->valid) {
    log_warn_sid(e->sid, "invalid ephemeris");
    return EPH_NEW_ERR;
  }

  /* compare new ephemeris against all other ephemeris to make sure there is
   * no ephemeris-to-ephemeris cross-correlation */
  if (xcorr_check_eph_to_eph(e)) {
    return EPH_NEW_XCORR;
  }

  /* TODO GLO: Implement ephemeris - almanac cross-checking for GLO */
  if (IS_GPS(e->sid)) {
    xcorr_positions_t alm_pos;
    xcorr_positions_t eph_pos;
    s32 time_s = e->toe.wn * WEEK_SECS + (s32)e->toe.tow;

    /* Compute three test positions over the ephemeris's time of validity */
    if (!xcorr_calc_eph_positions(e, time_s, &eph_pos)) {
      log_warn_sid(e->sid, "Failed to compute reference ECEFs");
      return EPH_NEW_ERR;
    }

    /* Compare against other almanacs */
    for (u32 sv_idx = 0; sv_idx < NUM_SATS_GPS; sv_idx++) {
      gnss_signal_t sid = construct_sid(CODE_GPS_L1CA, sv_idx + GPS_FIRST_PRN);
      if (sid.sat == e->sid.sat) {
        /* Skip self */
        continue;
      }
      if (xcorr_get_alm_positions(
              sid, eph_pos.time_s, eph_pos.interval_s, &alm_pos)) {
        if (xcorr_match_positions(e->sid, sid, &eph_pos, &alm_pos)) {
          /* Matched different almanac - cross correlation detected */
          return EPH_NEW_XCORR;
        }
      }
      /* OK: no data or distance mismatch; continue with another SV */
    }

    /* Compare against own almanac */
    switch (xcorr_match_alm_position(e->sid, e->sid, &eph_pos)) {
      case XCORR_MATCH_RES_OK:
        /* OK, ephemeris matches almanac */
        break;
      case XCORR_MATCH_RES_NO_ALMANAC:
        /* No valid almanac to compare to, should happen only during the
         * first 13 minutes or so after a cold start */
        break;
      case XCORR_MATCH_RES_NO_MATCH:
        /* Own almanac check has failed due to bad data, cross-correlation etc.
         */
        log_warn_sid(e->sid,
                     "Ephemeris does not match with almanac, discarding");

        return EPH_NEW_ERR;
        break;
      default:
        assert(!"Invalid match result");
    }
  }

  ndb_op_code_t oc =
      ndb_ephemeris_store(e, NDB_DS_RECEIVER, NDB_EVENT_SENDER_ID_VOID);
  switch (oc) {
    case NDB_ERR_NONE:
      log_debug_sid(e->sid, "ephemeris saved");
      break;
    case NDB_ERR_NO_CHANGE:
      log_debug_sid(e->sid, "ephemeris is already present");
      break;
    case NDB_ERR_UNCONFIRMED_DATA:
      log_debug_sid(e->sid, "ephemeris is unconfirmed, not saved");
      break;
    case NDB_ERR_OLDER_DATA:
      log_warn_sid(e->sid, "ephemeris is older than one in DB, not saved");
      break;
    case NDB_ERR_GPS_TIME_MISSING:
      log_debug_sid(e->sid, "GPS time unknown, ephemeris in DB not saved");
      break;
    case NDB_ERR_MISSING_IE:
    case NDB_ERR_UNSUPPORTED:
    case NDB_ERR_FILE_IO:
    case NDB_ERR_INIT_DONE:
    case NDB_ERR_BAD_PARAM:
    case NDB_ERR_ALGORITHM_ERROR:
    case NDB_ERR_NO_DATA:
    case NDB_ERR_AGED_DATA:
    default:
      log_warn_sid(e->sid, "error %d storing ephemeris", (int)oc);
      return EPH_NEW_ERR;
  }

  /* if satellite's azimuth and elevation are not yet cached, try to compute
   * them from the newly received ephemeris */
  last_good_fix_t lgf;
  if ((NDB_ERR_NONE == oc || NDB_ERR_UNCONFIRMED_DATA == oc) &&
      NDB_ERR_NONE == ndb_lgf_read(&lgf)) {
    update_azel_from_ephemeris(
        e, &lgf.position_solution.time, lgf.position_solution.pos_ecef);
  }
  return EPH_NEW_OK;
}

static void ephemeris_msg_callback(u16 sender_id,
                                   u8 len,
                                   u8 msg[],
                                   void *context) {
  (void)context;

  if (len != sizeof(msg_ephemeris_gps_t) &&
      len != sizeof(msg_ephemeris_glo_t) &&
      len != sizeof(msg_ephemeris_bds_t) &&
      len != sizeof(msg_ephemeris_gal_t) &&
      len != sizeof(msg_ephemeris_sbas_t)) {
    log_warn("Received ephemeris message of incorrect size %d from peer", len);
    return;
  }

  ephemeris_t e;
  memset(&e, 0, sizeof(e));
  unpack_ephemeris((msg_ephemeris_t *)msg, &e);
  if (!sid_supported(e.sid)) {
    log_warn_sid(
        e.sid,
        "Received ephemeris for an invalid/unsupported satellite from peer");
    return;
  }

  ndb_ephemeris_store(&e, NDB_DS_SBP, sender_id);
}

void ephemeris_setup(void) { sbp_ephe_reg_cbks(ephemeris_msg_callback); }
