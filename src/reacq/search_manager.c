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
#include <assert.h>
#include <inttypes.h>
#include <string.h>
#include <swiftnav/glo_map.h>
#include "manage.h"
#include "ndb/ndb_lgf.h"
#include "position/position.h"
#include "sbas_select/sbas_select.h"
#include "search_manager_api.h"
#include "timing/timing.h"

/**
 * Check if given constellation is supported.
 *
 * \return true  if constellation is supported.
 *         false otherwise
 */
static bool is_constellation_enabled(constellation_t con) {
  switch (con) {
    case CONSTELLATION_GPS:
      return true;
      break;

    case CONSTELLATION_SBAS:
      return is_sbas_enabled();
      break;

    case CONSTELLATION_GLO:
      return is_glo_enabled();
      break;

    case CONSTELLATION_BDS:
      return is_bds2_enabled();
      break;

    case CONSTELLATION_QZS:
      return is_qzss_enabled();
      break;

    case CONSTELLATION_GAL:
      return is_galileo_enabled();
      break;

    case CONSTELLATION_INVALID:
    case CONSTELLATION_COUNT:
    default:
      assert(!"Unsupported reacq constellation!");
      break;
  }
  return false;
}

/**
 * The function returns start job index according to gnss
 * \param[in] gnss Constellation
 * \return Start index of GNSS in job array
 */
u16 sm_constellation_to_start_index(constellation_t gnss) {
  switch ((s8)gnss) {
    case CONSTELLATION_GPS:
      return 0;
    case CONSTELLATION_GAL:
      return NUM_SATS_GPS;
    case CONSTELLATION_SBAS:
      return NUM_SATS_GPS + NUM_SATS_GAL;
    case CONSTELLATION_QZS:
      return NUM_SATS_GPS + NUM_SATS_GAL + NUM_SATS_SBAS;
    case CONSTELLATION_GLO:
      return NUM_SATS_GPS + NUM_SATS_GAL + NUM_SATS_SBAS + NUM_SATS_QZS;
    case CONSTELLATION_BDS:
      return NUM_SATS_GPS + NUM_SATS_GAL + NUM_SATS_SBAS + NUM_SATS_QZS +
             NUM_SATS_GLO;
    default:
      assert(!"Incorrect constellation");
      return 0;
  }
}

/**
 * Helper function. Return SBAS mask depending on user position
 * \return mask for SBAS SV.
 */
static u32 sbas_limit_mask(void) {
  /* read LGF */
  last_good_fix_t lgf;
  if (NDB_ERR_NONE != ndb_lgf_read(&lgf)) {
    /* cannot read LGF for some reason, so set mask for all possible SBAS SV*/
    return sbas_select_prn_mask(SBAS_WAAS) | sbas_select_prn_mask(SBAS_EGNOS) |
           sbas_select_prn_mask(SBAS_GAGAN) | sbas_select_prn_mask(SBAS_MSAS);
  } else {
    static sbas_system_t sbas_provider = SBAS_NONE;
    sbas_system_t new_provider = sbas_select_provider(&lgf);
    if ((sbas_provider != new_provider) && (SBAS_NONE != sbas_provider)) {
      tracker_set_sbas_provider_change_flag();
    }
    sbas_provider = new_provider;
    return sbas_select_prn_mask(sbas_provider);
  }
}

/** Global search job data */
acq_jobs_state_t acq_all_jobs_state_data;

/* Search manager API functions */

/** Initialize search manager module
 *
 * \param data pointer to global job data
 *
 * \return none
 */
void sm_init(acq_jobs_state_t *data) {
  memset(data, 0, sizeof(acq_jobs_state_t));

  struct init_struct {
    constellation_t gnss;
    u16 first_prn;
  } reacq_gnss[] = {{CONSTELLATION_GPS, GPS_FIRST_PRN},
                    {CONSTELLATION_GAL, GAL_FIRST_PRN},
                    {CONSTELLATION_SBAS, SBAS_FIRST_PRN},
                    {CONSTELLATION_QZS, QZS_FIRST_PRN},
                    {CONSTELLATION_GLO, GLO_FIRST_PRN},
                    {CONSTELLATION_BDS, BDS_FIRST_PRN}};

  for (u16 k = 0; k < ARRAY_SIZE(reacq_gnss); k++) {
    constellation_t gnss = reacq_gnss[k].gnss;
    u16 idx = sm_constellation_to_start_index(gnss);
    u16 num_sv = constellation_to_sat_count(gnss);
    code_t code = constellation_to_l1_code(gnss);

    if (idx + num_sv > REACQ_NUM_SAT) {
      log_error("idx + num_sv < REACQ_NUM_SAT: %d + %d < %d",
                idx,
                num_sv,
                REACQ_NUM_SAT);
      assert(0);
    }

    acq_job_t *job = &data->jobs[idx];
    u16 first_prn = reacq_gnss[k].first_prn;
    for (u16 i = 0; i < num_sv; i++) {
      if (CONSTELLATION_GLO == gnss) {
        /* NOTE: GLO MESID is initialized evenly with all FCNs, so that
         * blind searches are immediately done with whole range of FCNs */
        job[i].mesid = construct_mesid(code, first_prn + (i % GLO_MAX_FCN));
      } else {
        job[i].mesid = construct_mesid(code, first_prn + i);
      }
      job[i].sid = construct_sid(code, first_prn + i);
    }
  }
}

/** Categorizes satellites in "visible", "unknown" and "invisible" groups,
 *  and resets their status to "ready for reacq" following the simple concept of
 *  - do all visible until one unknown or invisible is done (note the loop in
 * `sch_run()`)
 *  - reset all visible to ready
 *  - if there are no more unknowns reset all unknowns to ready
 *  - if there are no more invisible reset all invisible to ready
 *
 * \param jobs_data pointer to job data
 * \param now_ms current time (ms)
 * \param last_job_type last job type
 *
 * \return none
 */
static void sm_restore_jobs(acq_jobs_state_t *jobs_data,
                            u64 now_ms,
                            reacq_sched_ret_t last_job_type) {
  assert(jobs_data != NULL);

  /* if the last job was a visible satellite, don't reset any of the reacq
   * states as the scheduler will continue to consume visibles next time */
  if (REACQ_DONE_VISIBLE == last_job_type) {
    log_warn("last_job_type is REACQ_DONE_VISIBLE");
    return;
  }

  u32 sbas_mask = sbas_limit_mask();
  u32 sbas_start_idx = sm_constellation_to_start_index(CONSTELLATION_SBAS);

  for (u16 i = 0; i < REACQ_NUM_SAT; i++) {
    acq_job_t *job_pt = &jobs_data->jobs[i];
    constellation_t con = code_to_constellation(job_pt->mesid.code);

    if (!is_constellation_enabled(con)) {
      job_pt->state = ACQ_STATE_IDLE;
      continue;
    }

    if (CONSTELLATION_SBAS == con) {
      assert(sbas_start_idx <= i);
      u32 sbas_idx = i - sbas_start_idx;
      /* don't set job for those SBAS SV which are not in our SBAS range,
       * or if we already more than the limit */
      if ((0 == ((sbas_mask >> sbas_idx) & 1)) ||
          (constellation_track_count(CONSTELLATION_SBAS) >=
           SBAS_SV_NUM_LIMIT)) {
        job_pt->state = ACQ_STATE_IDLE;
        continue;
      }
    }

    gnss_signal_t sid = job_pt->sid;
    assert(sid_valid(sid));

    me_gnss_signal_t *mesid = &job_pt->mesid;
    if (CONSTELLATION_GLO == con) {
      u16 glo_fcn = GLO_FCN_UNKNOWN;
      if (glo_map_valid(sid)) {
        glo_fcn = glo_map_get_fcn(sid);
        *mesid = construct_mesid(CODE_GLO_L1OF, glo_fcn);
      }
    }
    assert(mesid_valid(*mesid));

    /* if this mesid is in track, no need for its job */
    if (mesid_is_tracked(*mesid)) {
      job_pt->state = ACQ_STATE_IDLE;
      continue;
    }
    /* if this mesid can't be tracked, no need for its job */
    if (!tracking_startup_ready(*mesid)) {
      job_pt->state = ACQ_STATE_IDLE;
      continue;
    }

    bool visible = false;
    bool known = false;
    bool invisible = false;
    sm_get_visibility_flags(sid, &visible, &known);
    invisible = !visible && known;
    visible = visible && known;

    if (CONSTELLATION_SBAS == con) {
      /* sbas_mask SVs are visible as they were selected by location. */
      visible = true;
    }

    /* save the job category into `sky_status` */
    job_pt->sky_status = known ? (visible ? VISIBLE : INVISIBLE) : UNKNOWN;

    if (visible &&
        ((now_ms - job_pt->stop_time) > REACQ_MIN_SEARCH_INTERVAL_VISIBLE_MS)) {
      /* we should only arrive here once an unknown or invisible satellite has
       * been tried */
      job_pt->state = ACQ_STATE_WAIT;
    } else if ((!known || (CONSTELLATION_GLO == con && !glo_map_valid(sid))) &&
               ((now_ms - job_pt->stop_time) >
                REACQ_MIN_SEARCH_INTERVAL_UNKNOWN_MS)) {
      /* if the scheduler has done nothing or tried an invisible satellite it
       * means that there were no more unknown satellites to search, so reset
       * their status */
      if (REACQ_DONE_UNKNOWN < last_job_type) job_pt->state = ACQ_STATE_WAIT;
    } else if (invisible && ((now_ms - job_pt->stop_time) >
                             REACQ_MIN_SEARCH_INTERVAL_INVISIBLE_MS)) {
      /* if the scheduler last time has done nothing then there were no ready
       * satellites so it's time to put the invisible sats in the search queue
       * again */
      if (REACQ_DONE_INVISIBLE < last_job_type) job_pt->state = ACQ_STATE_WAIT;
    }
  } /* loop jobs */
}

/** Run search manager
 *
 *  Decides when and which jobs need to be run
 *
 * \param jobs_data pointer to job data
 * \param last_job_type what job type was run last
 *
 * \return none
 */
void sm_run(acq_jobs_state_t *jobs_data, reacq_sched_ret_t last_job_type) {
  u64 now_ms = timing_getms();
  sm_calc_all_glo_visibility_flags();
  sm_restore_jobs(jobs_data, now_ms, last_job_type);
}
