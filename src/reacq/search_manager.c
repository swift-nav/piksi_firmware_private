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
#include <manage.h>

/* Search manager functions which call other modules */

bool sm_is_healthy(gnss_signal_t sid);
bool sm_lgf_stamp(u64 *lgf_stamp);
void sm_get_visibility_flags(gnss_signal_t sid, bool *visible, bool *known);

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
      data->jobs[type][i].mesid = construct_mesid(CODE_GPS_L1CA,
                                                  GPS_FIRST_PRN + i);
      data->jobs[type][i].job_type = type;
    }
  }
}

/** Checks if deep searches need to run
 *
 * \param jobs_data pointer to job data
 *
 * \return none
 */
static void sm_deep_search_run(acq_jobs_state_t *jobs_data)
{
  u32 i;
  for (i = 0; i < ACQ_NUM_SVS; i++) {
    acq_job_t *deep_job = &jobs_data->jobs[ACQ_JOB_DEEP_SEARCH][i];
    me_gnss_signal_t mesid = deep_job->mesid;
    bool visible, known;

    assert(sid_valid(mesid2sid(mesid)));

    assert(deep_job->job_type < ACQ_NUM_JOB_TYPES);

    /* Initialize jobs to not run */
    deep_job->needs_to_run = false;

    /* Check if jobs need to run */
    if (mesid_is_tracked(mesid)) {
      continue;
    }

    sm_get_visibility_flags(mesid2sid(mesid), &visible, &known);
    visible = visible && known;

    if (visible) {
      deep_job->cost_hint = ACQ_COST_MIN;
      deep_job->cost_delta = 0;
      deep_job->needs_to_run = true;
      deep_job->oneshot = false;
    }
  } /* loop SVs */
}
/** Checks if fallback searches need to run
 *
 * \param jobs_data pointer to job data
 * \param now_ms current time (ms)
 * \param lgf_age_ms age of the last good fix (ms)
 *
 * \return none
 */
static void sm_fallback_search_run(acq_jobs_state_t *jobs_data,
                                   u64 now_ms,
                                   u64 lgf_age_ms)
{
  u32 i;
  for (i = 0; i < ACQ_NUM_SVS; i++) {
    acq_job_t *fallback_job = &jobs_data->jobs[ACQ_JOB_FALLBACK_SEARCH][i];
    me_gnss_signal_t mesid = fallback_job->mesid;
    bool visible, invisible, known;

    assert(fallback_job->job_type < ACQ_NUM_JOB_TYPES);

    assert(sid_valid(mesid2sid(mesid)));

    /* Initialize jobs to not run */
    fallback_job->needs_to_run = false;

    /* Check if jobs need to run */
    if (mesid_is_tracked(mesid)) {
      continue;
    }

    sm_get_visibility_flags(mesid2sid(mesid), &visible, &known);
    visible = visible && known;
    invisible = !visible && known;

    if (visible &&
        lgf_age_ms >= ACQ_LGF_TIMEOUT_VIS_AND_UNKNOWN_MS &&
        now_ms - fallback_job->stop_time >
        ACQ_FALLBACK_SEARCH_TIMEOUT_VIS_AND_UNKNOWN_MS) {
      fallback_job->cost_hint = ACQ_COST_AVG;
      fallback_job->cost_delta = ACQ_COST_DELTA_VISIBLE_MS;
      fallback_job->needs_to_run = true;
      fallback_job->oneshot = true;
    } else if (!known &&
        lgf_age_ms >= ACQ_LGF_TIMEOUT_VIS_AND_UNKNOWN_MS &&
        now_ms - fallback_job->stop_time >
        ACQ_FALLBACK_SEARCH_TIMEOUT_VIS_AND_UNKNOWN_MS) {
      fallback_job->cost_hint = ACQ_COST_MAX_PLUS;
      fallback_job->cost_delta = ACQ_COST_DELTA_UNKNOWN_MS;
      fallback_job->needs_to_run = true;
      fallback_job->oneshot = true;
    } else if (invisible &&
               lgf_age_ms >= ACQ_LGF_TIMEOUT_INVIS_MS &&
               now_ms - fallback_job->stop_time >
               ACQ_FALLBACK_SEARCH_TIMEOUT_INVIS_MS) {
      fallback_job->cost_hint = ACQ_COST_MAX_PLUS;
      fallback_job->cost_delta = ACQ_COST_DELTA_INVISIBLE_MS;
      fallback_job->needs_to_run = true;
      fallback_job->oneshot = true;
    }
  } /* loop SVs */
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
  u64 now_ms = timing_getms();
  u64 lgf_ms, lgf_age_ms;
  if (sm_lgf_stamp(&lgf_ms) && (now_ms >= lgf_ms)) {
    lgf_age_ms = now_ms - lgf_ms;
  } else {
    lgf_age_ms = MAX(ACQ_LGF_TIMEOUT_VIS_AND_UNKNOWN_MS,
                     ACQ_LGF_TIMEOUT_INVIS_MS);
  }
  
  sm_deep_search_run(jobs_data);
  sm_fallback_search_run(jobs_data, now_ms, lgf_age_ms);
}

