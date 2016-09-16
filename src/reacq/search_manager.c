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
#ifdef SM_UNIT_TEST
#include <stdio.h>
#else
#include "timing.h"
#include "shm.h"
#include "manage.h"
#include "ndb.h"
#include "board/nap/nap_common.h"
#include <libswiftnav/sv_visibility.h>
#endif

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

/** Check if SV is healthy
 *
 * \param sid SV identifier
 *
 * \return TRUE is SV is healthy, FALSE otherwise
 */
static bool sm_is_healthy(gnss_signal_t sid);

/** Check if SV is tracked
 *
 * \param sid SV identifier
 *
 * \return TRUE is SV is tracked, FALSE otherwise
 */
static bool sm_is_tracked(gnss_signal_t sid);

/** Get current HW time in milliseconds
 *
 * \return HW time in milliseconds
 */
static u64 sm_getms(void);

/** Get HW time of the last good fix (LGF)
 *
 * \return HW time (ms) of the last good fix
 */
static u64 sm_lgf_stamp(void);

/** Get SV visibility flags
 *
 * \param sid SV identifier
 * \param [out] visible set if SV is visible
 * \param [out] known set if SV is known visible or known invisible
 */
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
  u64 now_ms = sm_getms();
  u64 lgf_age_ms = now_ms - sm_lgf_stamp();

  for (i=0; i < ACQ_NUM_SVS; i++) {
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

#ifndef SM_UNIT_TEST

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

  if (ndb_lgf_read(&lgf) == NDB_ERR_NONE &&
      lgf.position_quality == POSITION_FIX &&
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

/** Get current HW time in milliseconds
 *
 * \return HW time in milliseconds
 */
static u64 sm_getms(void)
{
  return (u64)(nap_timing_count() * (RX_DT_NOMINAL * 1000.0));
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
  return sm_getms() - MAX(ACQ_LGF_TIMEOUT_MS, ACQ_LGF_TIMEOUT_INVIS_MS  );
}

#else /* SM_UNIT_TEST */

/** Unit test input data type */
struct test_case_t {
  u32 now_ms;
  u32 health_mask;
  u32 vis_mask, known_mask, track_mask;
  u32 lgf_stamp_ms;
  u32 deep_mask, fallback_mask; /**< Expected results */
};

/** Current unit test case */
static struct test_case_t *test_case;

/** Unit test cases */
static struct test_case_t test_cases[] = {
  {0,0x0, 0x0, 0x0, 0x0, 0,   0x0, 0x0}, /* Nothing */
  {10, 0xffffffff, 0x0, 0x0, 0x0, 0,   0xffffffff,0x0},/* All healthy
                                                          not tracked */
  {10, 0xffffffff, 0x0, 0x0, 0xffffffff, 0,   0x0, 0x0},/* All healthy all
                                                           tracked */
  {20, 0x0, 0x0, 0x0, 0xffffffff, 0,   0x0, 0x0},/* All unhealthy all tracked */
  {30, 0x1, 0x1, 0x1, 0x0, 0,   0x1, 0x0},/* 1 healthy, not tracked, visible */
  {30, 0x1, 0x0, 0x1, 0x0, 0,   0x0, 0x0},/* 1 healthy, not tracked,
                                             invisible */
  {30, 0x1, 0x0, 0x0, 0x0, 0,   0x1, 0x0},/* 1 healthy, not tracked, unknown */
  /* Timeouts */
  {ACQ_LGF_TIMEOUT_MS+1, 0x1, 0x1, 0x1, 0x0, 0,   0x1, 0x1},/* 1 healthy, not
                                                            tracked, visible */
  {ACQ_LGF_TIMEOUT_INVIS_MS+1, 0x1, 0x0, 0x1, 0x0, 0,   0x0, 0x1},/* 1 healthy,
                                                       not tracked,invisible */
  {ACQ_LGF_TIMEOUT_MS+1, 0x1, 0x0, 0x0, 0x0, 0,   0x1, 0x1} /* 1 healthy, not
                                                       tracked, unknown */
};


/** Get SV visibility flags
 *
 * \param sid SV identifier
 * \param [out] visible set if SV is visible
 * \param [known] known set if SV is known visible or known invisible
 */
static void sm_get_visibility_flags(gnss_signal_t sid,
                                    bool *visible, bool *known)
{
  if (test_case->vis_mask & (1 << (sid.sat - GPS_FIRST_PRN))) {
    *visible = true;
  } else {
    *visible = false;
  }
  if (test_case->known_mask & (1 << (sid.sat - GPS_FIRST_PRN))) {
    *known = true;
  } else {
    *known = false;
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
  if (test_case->health_mask & (1 << (sid.sat - GPS_FIRST_PRN))) {
    return true;
  }
  return false;
}

/** Check if SV is tracked
 *
 * \param sid SV identifier
 *
 * \return TRUE is SV is tracked, FALSE otherwise
 */
static bool sm_is_tracked(gnss_signal_t sid)
{
  if (test_case->track_mask & (1 << (sid.sat - GPS_FIRST_PRN))) {
    return true;
  }
  return false;
}

/** Get current HW time in milliseconds
 *
 * \return HW time in milliseconds
 */
static u64 sm_getms(void)
{
  return (u64)test_case->now_ms;
}

/** Get HW time of the last good fix (LGF)
 *
 * \return HW time (ms) of the last good fix
 */
static u64 sm_lgf_stamp(void)
{
  return (u64)test_case->lgf_stamp_ms;
}

/** Test program checking search manager operation
 *
 * \param none
 *
 * \return 1 on failure, 0 othersiwe
 */
int main(int argc, char **argv)
{
  acq_jobs_state_t *data = &acq_all_jobs_state_data;
  int test_ix;

  sm_init(data);

  for(test_ix = 0;
      test_ix < sizeof(test_cases)/sizeof(test_cases[0]);
      test_ix++) {
    u32 gps_run_mask[ACQ_NUM_JOB_TYPES];
    acq_job_types_e type;
    u32 i;

    test_case = &test_cases[test_ix];

    printf("now=%u health=0x%x vis=0x%x knw=0x%x trk=0x%x lgf=%u\n",
           test_case->now_ms, test_case->health_mask, test_case->vis_mask,
           test_case->known_mask, test_case->track_mask,
           test_case->lgf_stamp_ms);

    sm_run(data);

    /* Fill bit masks of jobs which are flagged to run */
    memset(gps_run_mask, 0, sizeof(gps_run_mask));
    for (type = 0; type < ACQ_NUM_JOB_TYPES; type++) {
      for (i = 0; i < ACQ_NUM_SVS; i++) {
        if (data->jobs[type][i].needs_to_run) {
          if (CODE_GPS_L1CA == data->jobs[type][i].sid.code) {
            gps_run_mask[type] |=
              1 << (data->jobs[type][i].sid.sat - GPS_FIRST_PRN);
          }
        }
      }
    }

    printf("Search gps deep=0x%08x fallback=0x%08x",
           gps_run_mask[ACQ_JOB_DEEP_SEARCH],
           gps_run_mask[ACQ_JOB_FALLBACK_SEARCH]);
    if (gps_run_mask[ACQ_JOB_DEEP_SEARCH] != test_case->deep_mask ||
        gps_run_mask[ACQ_JOB_FALLBACK_SEARCH] != test_case->fallback_mask) {
      printf("\tFAIL\n");
      return 1;
    } else {
      printf("\tOK\n");
    }
  } // for

  return 0;
}
#endif /* SM_UNIT_TEST */
