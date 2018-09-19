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
  track_sid_db_azel_degrees_set(e->sid, az * R2D, el * R2D, nap_timing_count());
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
  track_sid_db_azel_degrees_set(a->sid, az * R2D, el * R2D, nap_timing_count());
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
  (void)sender_id;

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

  /* storing of received ephemeris into NDB disabled for now, pending testing
  ndb_ephemeris_store(&e, NDB_DS_SBP, sender_id);
  */
}

void ephemeris_setup(void) { sbp_ephe_reg_cbks(ephemeris_msg_callback); }
