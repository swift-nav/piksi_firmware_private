/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Perttu Salmela <psalmela@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#include <string.h>
#include <assert.h>
#include "search_manager_api.h"
#include <timing.h>
#include <shm.h>
#include <manage.h>
#include <ndb.h>
#include <board/nap/nap_common.h>
#include <libswiftnav/sv_visibility.h>

/* Search manager constants */

/** Timeout (ms) defining period between fallback searches of
    visible and unknown SVs */
#define ACQ_FALLBACK_SEARCH_TIMEOUT_MS 8000
/** Timeout (ms) defining period between fallback searches of
    known invisible SVs */
#define ACQ_FALLBACK_SEARCH_TIMEOUT_INVIS_MS 16000
/** Starts fallback searches when last good fix (LGF) is
    older than timeout (ms) */
#define ACQ_LGF_TIMEOUT_MS 30000
/** Starts fallback searches of invisible SVs when last good fix (LGF) is
    older than timeout (ms) */
#define ACQ_LGF_TIMEOUT_INVIS_MS 60000
/** Max user velocity for visibility calculation (m/s) */
#define ACQ_MAX_USER_VELOCITY_MPS 30.0f

/* Search manager functions which call other modules */

static bool sm_is_healthy(gnss_signal_t sid);
static bool sm_is_tracked(gnss_signal_t sid);
static u64 sm_lgf_stamp(void);
static void sm_get_visibility_flags(gnss_signal_t sid,
                                    bool *visible, bool *known);

/** Global search job data */
acq_jobs_state_t acq_all_jobs_state_data;

/* Search manager API functions */

/** Initialize search manager module
 *
 * \param data pointer to global job data
 *
 * \return none
 */
void sm_init(acq_jobs_state_t *data)
{
  memset(data, 0, sizeof(acq_jobs_state_t));

  acq_job_types_e type;

  for (type = 0; type < ACQ_NUM_JOB_TYPES; type++) {
    u32 i;
    for (i = 0; i < NUM_SATS_GPS; i++) {
      data->jobs[type][i].sid.code = CODE_GPS_L1CA;
      data->jobs[type][i].sid.sat = GPS_FIRST_PRN + i;
      data->jobs[type][i].job_type = type;
    }
  }
}

/** Run search manager
 *
 *  Decides when and which jobs need to be run
 *
 * \param jobs_data pointer to job data
 *
 * \return none
 */
void sm_run(acq_jobs_state_t *jobs_data)
{
  u32 i;
  u64 now_ms = timing_getms();
  u64 lgf_age_ms = now_ms - sm_lgf_stamp();

  for (i = 0; i < ACQ_NUM_SVS; i++) {
    acq_job_t *deep_job = &jobs_data->jobs[ACQ_JOB_DEEP_SEARCH][i];
    acq_job_t *fallback_job = &jobs_data->jobs[ACQ_JOB_FALLBACK_SEARCH][i];
    gnss_signal_t sid = deep_job->sid; /* Fallback job has the same sid */
    bool visible, invisible, known;

    /* Initialize jobs to not run */
    deep_job->needs_to_run = false;
    fallback_job->needs_to_run = false;

    /* Check if jobs need to run */
    if (!sm_is_healthy(sid)) {
      continue;
    }
    if (sm_is_tracked(sid)) {
      continue;
    }

    sm_get_visibility_flags(sid, &visible, &known);
    visible = visible && known;
    invisible = !visible && known;

    /* Checks for deep search */
    if (visible) {

      deep_job->cost_hint = ACQ_COST_MIN;
      deep_job->needs_to_run = true;
      deep_job->oneshot = false;

    } else if (!known) {

      deep_job->cost_hint = ACQ_COST_AVG;
      deep_job->needs_to_run = true;
      deep_job->oneshot = false;
    }
    /* Checks for fallback search */
    if ((visible || !known) &&
        lgf_age_ms > ACQ_LGF_TIMEOUT_MS &&
        now_ms - fallback_job->stop_time > ACQ_FALLBACK_SEARCH_TIMEOUT_MS) {

      fallback_job->cost_hint = ACQ_COST_MAX_PLUS;
      fallback_job->needs_to_run = true;
      fallback_job->oneshot = true;

    } else if (invisible &&
               lgf_age_ms > ACQ_LGF_TIMEOUT_INVIS_MS &&
               now_ms - fallback_job->stop_time >
               ACQ_FALLBACK_SEARCH_TIMEOUT_INVIS_MS) {

      fallback_job->cost_hint = ACQ_COST_MAX_PLUS;
      fallback_job->needs_to_run = true;
      fallback_job->oneshot = true;
    }
  } /* loop SVs */
}

/** Get SV visibility flags
 *
 * \param sid SV identifier
 * \param [out] visible set if SV is visible
 * \param [out] known set if SV is known visible or known invisible
 */
static void sm_get_visibility_flags(gnss_signal_t sid,
                                    bool *visible, bool *known)
{
  last_good_fix_t lgf;
  ephemeris_t ephe;

  *visible = false;
  *known = false;

  if (NDB_ERR_NONE == ndb_lgf_read(&lgf) &&
      POSITION_FIX == lgf.position_quality &&
      NDB_ERR_NONE != ndb_ephemeris_read(sid, &ephe)) {

    sv_vis_config_t vis_cfg;

    vis_cfg.e = &ephe;
    vis_cfg.lgf_ecef[0] = lgf.position_solution.pos_ecef[0];
    vis_cfg.lgf_ecef[1] = lgf.position_solution.pos_ecef[1];
    vis_cfg.lgf_ecef[2] = lgf.position_solution.pos_ecef[2];
    vis_cfg.lgf_time = lgf.position_solution.time;
    vis_cfg.user_velocity = ACQ_MAX_USER_VELOCITY_MPS;
    vis_cfg.time_delta = (u32)((nap_timing_count() -
				gps2rxtime(&lgf.position_solution.time)) *
                               RX_DT_NOMINAL);

    sv_visibility_status_get(&vis_cfg, visible, known);
  }
}

/** Check if SV is healthy
 *
 * \param sid SV identifier
 *
 * \return TRUE is SV is healthy, FALSE otherwise
 */
static bool sm_is_healthy(gnss_signal_t sid)
{
  return shm_get_sat_state(sid) != CODE_NAV_STATE_INVALID;
}

/** Check if SV is tracked
 *
 * \param sid SV identifier
 *
 * \return TRUE is SV is tracked, FALSE otherwise
 */
static bool sm_is_tracked(gnss_signal_t sid)
{
  return is_sid_tracked(sid);
}

/** Get HW time of the last good fix (LGF)
 *
 * \return HW time (ms) of the last good fix
 */
static u64 sm_lgf_stamp(void)
{
  last_good_fix_t lgf;
  if (ndb_lgf_read(&lgf) == NDB_ERR_NONE &&
      lgf.position_quality == POSITION_FIX) {

    return (u64)(gps2rxtime(&lgf.position_solution.time)
                 * (RX_DT_NOMINAL * 1000.0));
  }
  log_error("No LGF but in re-acquisition mode");
  /* Return time stamp which would trigger any timeouts */
  return timing_getms() - MAX(ACQ_LGF_TIMEOUT_MS, ACQ_LGF_TIMEOUT_INVIS_MS  );
}

