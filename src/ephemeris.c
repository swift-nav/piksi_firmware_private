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
#include <string.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/linear_algebra.h>
#include <ch.h>
#include <assert.h>

#include "sbp.h"
#include "sbp_utils.h"
#include "track.h"
#include "timing.h"
#include "ephemeris.h"
#include "signal.h"
#include "ndb.h"

#include <track/track_sid_db.h>

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
                              xcorr_positions_t *pos)
{
  almanac_t a;
  ndb_op_code_t oc = ndb_almanac_read(sid, &a);
  /* Here we do not care if GPS time is unknown
   * since almanac is used with input time_s. */
  bool alma_valid = (NDB_ERR_NONE == oc || NDB_ERR_GPS_TIME_MISSING == oc);

  if (!alma_valid || a.toa.wn <= 0) {
    return false;
  }

  gps_time_t t0 = make_gps_time(time_s - interval_s);
  gps_time_t t1 = make_gps_time(time_s);
  gps_time_t t2 = make_gps_time(time_s + interval_s);

  if (!almanac_valid(&a, &t0) || !almanac_valid(&a, &t2)) {
    return false;
  }

  double _[3];
  calc_sat_state_almanac(&a, &t0, pos->early.xyz, _, _, _);
  calc_sat_state_almanac(&a, &t1, pos->prompt.xyz, _, _, _);
  calc_sat_state_almanac(&a, &t2, pos->late.xyz, _, _, _);
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
                              xcorr_positions_t *pos)
{
  u32 interval_s = e->fit_interval / 2;
  gps_time_t t0 = make_gps_time(time_s - interval_s);
  gps_time_t t1 = make_gps_time(time_s);
  gps_time_t t2 = make_gps_time(time_s + interval_s);

  if (!ephemeris_valid(e, &t0) || !ephemeris_valid(e, &t2)) {
    return false;
  }

  double _[3];
  calc_sat_state_n(e, &t0, pos->early.xyz, _, _, _);
  calc_sat_state_n(e, &t1, pos->prompt.xyz, _, _, _);
  calc_sat_state_n(e, &t2, pos->late.xyz, _, _, _);
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
                             xcorr_positions_t *pos)
{
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
                           const xcorr_positions_t *pos1)
{
  bool ok = true;
  double d[3] = {-1, -1, -1};
  for (u8 i = 0; i < 3 && ok; i++) {
    d[i] = vector_distance(3, pos0->epl[i].xyz, pos1->epl[i].xyz);
    ok = (d[i] <= XCORR_MAX_EA_DISTANCE_M);
  }

  char sid_str_[SID_STR_LEN_MAX];
  sid_to_string(sid_str_, sizeof(sid_str_), sid1);
  log_debug_sid(sid0, "-> %s distance: %le, %le, %le",
                sid_str_, d[0], d[1], d[2]);
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
                                           const xcorr_positions_t *pos)
{
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
eph_new_status_t ephemeris_new(const ephemeris_t *e)
{
  assert(sid_supported(e->sid));

  if (!e->valid) {
    log_warn_sid(e->sid, "invalid ephemeris");
    return EPH_NEW_ERR;
  }

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
    if (xcorr_get_alm_positions(sid, eph_pos.time_s, eph_pos.interval_s,
                                &alm_pos)) {
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
    /* Own almanac check has failed due to bad data, cross-correlation etc. */
    log_warn_sid(e->sid, "Ephemeris does not match with almanac, discarding");

    return EPH_NEW_ERR;
    break;
  default:
    assert(!"Invalid match result");
  }

  ndb_op_code_t oc = ndb_ephemeris_store(e,
                                         NDB_DS_RECEIVER,
                                         NDB_EVENT_SENDER_ID_VOID);
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

  return EPH_NEW_OK;
}

static void ephemeris_msg_callback(u16 sender_id, u8 len, u8 msg[],
                                   void* context)
{
  (void)context;

  if (len != sizeof(msg_ephemeris_t)) {
    log_warn("Received bad ephemeris from peer");
    return;
  }

  ephemeris_t e;
  memset(&e, 0, sizeof(e));
  unpack_ephemeris((msg_ephemeris_t *)msg, &e);
  if (!sid_supported(e.sid)) {
    log_warn("Ignoring ephemeris for invalid sat");
    return;
  }

  ndb_ephemeris_store(&e, NDB_DS_SBP, sender_id);
}

void ephemeris_setup(void)
{
  sbp_ephe_reg_cbks(ephemeris_msg_callback);
}
