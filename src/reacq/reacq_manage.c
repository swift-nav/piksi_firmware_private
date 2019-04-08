/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "main.h"
#include "ndb/ndb.h"
#include "position/position.h"
#include "scheduler_api.h"
#include "search_manager_api.h"
#include "simulator.h"
#include "track/track_sid_db.h"

/**
 * Reacqusition controller initialization.
 *
 * The method performs all reacqusition related initialization.
 *
 * \return None
 */
void init_reacq(void) { sm_init(&acq_all_jobs_state_data); }

/** Update the satellite azimuth & elevation database with current angles
 * \param rcv_pos Approximate receiver position
 * \param t Approximate time
 */
static void update_sat_azel(const double rcv_pos[3], const gps_time_t t) {
  ephemeris_t ephemeris;
  almanac_t almanac;

  /* compute elevation for any valid ephemeris/almanac we can pull from NDB */
  for (u16 sv_index = 0; sv_index < NUM_SATS; sv_index++) {
    /* form a SID with the first code for the constellation */
    gnss_signal_t sid = sv_index_to_sid(sv_index);
    if (!sid_valid(sid)) {
      continue;
    }
    /* try to compute from ephemeris */
    ndb_op_code_t res = ndb_ephemeris_read(sid, &ephemeris);
    if (NDB_ERR_NONE == res || NDB_ERR_UNCONFIRMED_DATA == res) {
      if (0 == update_azel_from_ephemeris(&ephemeris, &t, rcv_pos)) {
        /* success */
        continue;
      }
    }
    /* else try to compute from almanac */
    if (NDB_ERR_NONE == ndb_almanac_read(sid, &almanac)) {
      update_azel_from_almanac(&almanac, &t, rcv_pos);
    }
  }
}

static void update_sat_azel_db(void) {
  time_quality_t time_quality = get_time_quality();
  last_good_fix_t lgf;
  ndb_op_code_t ret = ndb_cached_lgf_read(&lgf);
  if (TIME_UNKNOWN != time_quality && NDB_ERR_NONE == ret &&
      lgf.position_solution.valid && lgf.position_quality >= POSITION_GUESS &&
      !simulation_enabled()) {
    update_sat_azel(lgf.position_solution.pos_ecef, lgf.position_solution.time);
  }
}

/**
 * Reacqusition controller loop.
 *
 * The method performs all reacqusition control functions within acquisition
 * thread scope.
 *
 * \return None
 */
void manage_reacq(void) {
  static reacq_sched_ret_t last_job_type = REACQ_DONE_NOTHING;
  DO_EACH_MS(MAX_AZ_EL_AGE_SEC * SECS_MS / 2, update_sat_azel_db());
  /* Perform reacq jobs */
  sm_restore_jobs(&acq_all_jobs_state_data, last_job_type);
  last_job_type = sch_run(&acq_all_jobs_state_data);
}
