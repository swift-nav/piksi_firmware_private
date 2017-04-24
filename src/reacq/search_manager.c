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
#include "glo_map.h"

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
      data->jobs_gps[type][i].mesid = construct_mesid(CODE_GPS_L1CA,
                                                      GPS_FIRST_PRN + i);
      /* for GPS SID it's just copy of MESID */
      data->jobs_gps[type][i].sid = construct_sid(CODE_GPS_L1CA,
                                                  GPS_FIRST_PRN + i);
      data->jobs_gps[type][i].job_type = type;
    }
    for (i = 0; i < NUM_SATS_GLO; i++) {
      /* NOTE: MESID will be constructed on the fly */
      data->jobs_glo[type][i].sid = construct_sid(CODE_GLO_L1CA,
                                                  GLO_FIRST_PRN + i);
      data->jobs_glo[type][i].job_type = type;
    }
  }
}

/** Checks if deep searches need to run got GPS SV
 *
 * \param jobs_data pointer to job data
 *
 * \return none
 */
static void sm_deep_search_run_gps(acq_jobs_state_t *jobs_data)
{
  u32 i;
  for (i = 0; i < NUM_SATS_GPS; i++) {
    acq_job_t *deep_job = &jobs_data->jobs_gps[ACQ_JOB_DEEP_SEARCH][i];
    me_gnss_signal_t mesid = deep_job->mesid;
    gnss_signal_t sid = deep_job->sid;

    bool visible, known;

    assert(mesid_valid(mesid));
    assert(sid_valid(sid));
    assert(!is_glo_sid(mesid));

    assert(deep_job->job_type < ACQ_NUM_JOB_TYPES);

    /* Initialize jobs to not run */
    deep_job->needs_to_run = false;

    /* Check if jobs need to run */
    if (mesid_is_tracked(mesid)) {
      continue;
    }

    sm_get_visibility_flags(sid, &visible, &known);
    visible = visible && known;

    if (visible) {
      deep_job->cost_hint = ACQ_COST_MIN;
      deep_job->cost_delta = 0;
      deep_job->needs_to_run = true;
      deep_job->oneshot = false;
    }
  } /* loop SVs */
}

/** Checks if deep searches need to run for GLO SV
 *
 * \param jobs_data pointer to job data
 *
 * \return none
 */
static void sm_deep_search_run_glo(acq_jobs_state_t *jobs_data)
{
  u32 i;
  for (i = 0; i < NUM_SATS_GLO; i++) {
    acq_job_t *deep_job = &jobs_data->jobs_glo[ACQ_JOB_DEEP_SEARCH][i];
    me_gnss_signal_t *mesid = &deep_job->mesid;
    gnss_signal_t sid = deep_job->sid;
    u16 glo_fcn = glo_map_get_fcn(sid);

    if (GLO_FCN_UNKNOWN == glo_fcn) {
      /* if there is no any mapping go through all GLO FCN and pick one */
      for (glo_fcn = GLO_MIN_FCN; glo_fcn <= GLO_MAX_FCN; glo_fcn++) {
        if (mesid_is_tracked(construct_mesid(CODE_GLO_L1CA, glo_fcn))) {
          continue;
        } else {
          deep_job->glo_blind_search = true;
          break;
        }
      }
    }

    *mesid = construct_mesid(CODE_GLO_L1CA, glo_fcn);

    bool visible = false;
    bool known = false;

    assert(mesid_valid(*mesid));
    assert(sid_valid(sid));
    assert(is_glo_sid(*mesid));

    assert(deep_job->job_type < ACQ_NUM_JOB_TYPES);

    /* Initialize jobs to not run */
    deep_job->needs_to_run = false;

    if (!deep_job->glo_blind_search) {
      sm_get_visibility_flags(sid, &visible, &known);
      visible = visible && known;
    }

    if (visible || deep_job->glo_blind_search) {
      deep_job->cost_hint = ACQ_COST_MIN;
      deep_job->cost_delta = 0;
      deep_job->needs_to_run = true;
      deep_job->oneshot = false;
    }
  } /* loop SVs */
}

/** Checks if fallback searches need to run for GPS SV
 *
 * \param jobs_data pointer to job data
 * \param now_ms current time (ms)
 * \param lgf_age_ms age of the last good fix (ms)
 *
 * \return none
 */
static void sm_fallback_search_run_gps(acq_jobs_state_t *jobs_data,
                                       u64 now_ms,
                                       u64 lgf_age_ms)
{
  u32 i;
  for (i = 0; i < NUM_SATS_GPS; i++) {
    acq_job_t *fallback_job = &jobs_data->jobs_gps[ACQ_JOB_FALLBACK_SEARCH][i];
    me_gnss_signal_t mesid = fallback_job->mesid;
    gnss_signal_t sid = fallback_job->sid;

    bool visible, invisible, known;

    assert(fallback_job->job_type < ACQ_NUM_JOB_TYPES);

    assert(mesid_valid(mesid));
    assert(!is_glo_sid(mesid));

    /* Initialize jobs to not run */
    fallback_job->needs_to_run = false;

    /* Check if jobs need to run */
    if (mesid_is_tracked(mesid)) {
      continue;
    }

    sm_get_visibility_flags(sid, &visible, &known);
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

/** Checks if fallback searches need to run for GLO SV
 *
 * \param jobs_data pointer to job data
 * \param now_ms current time (ms)
 * \param lgf_age_ms age of the last good fix (ms)
 *
 * \return none
 */
static void sm_fallback_search_run_glo(acq_jobs_state_t *jobs_data,
                                       u64 now_ms,
                                       u64 lgf_age_ms)
{
  u32 i;
  for (i = 0; i < NUM_SATS_GLO; i++) {
    acq_job_t *fallback_job = &jobs_data->jobs_glo[ACQ_JOB_FALLBACK_SEARCH][i];
    me_gnss_signal_t *mesid = &fallback_job->mesid;
    gnss_signal_t sid = fallback_job->sid;
    u16 glo_fcn = glo_map_get_fcn(sid);
    if (GLO_FCN_UNKNOWN == glo_fcn) {
      /* if there is no any mapping go through all GLO FCN and pick one */
      for (glo_fcn = GLO_MIN_FCN; glo_fcn <= GLO_MAX_FCN; glo_fcn++) {
        if (mesid_is_tracked(construct_mesid(CODE_GLO_L1CA, glo_fcn))) {
          continue;
        } else {
          fallback_job->glo_blind_search = true;
          break;
        }
      }
    }

    *mesid = construct_mesid(CODE_GLO_L1CA, glo_fcn);

    bool visible = false;
    bool invisible = false;
    bool known = false;

    assert(fallback_job->job_type < ACQ_NUM_JOB_TYPES);

    assert(mesid_valid(*mesid));
    assert(sid_valid(sid));
    assert(is_glo_sid(*mesid));

    /* Initialize jobs to not run */
    fallback_job->needs_to_run = false;

    if (!fallback_job->glo_blind_search) {
      sm_get_visibility_flags(sid, &visible, &known);
      visible = visible && known;
      invisible = !visible && known;
    }

    if (visible &&
        lgf_age_ms >= ACQ_LGF_TIMEOUT_VIS_AND_UNKNOWN_MS &&
        now_ms - fallback_job->stop_time >
        ACQ_FALLBACK_SEARCH_TIMEOUT_VIS_AND_UNKNOWN_MS) {
      fallback_job->cost_hint = ACQ_COST_AVG;
      fallback_job->cost_delta = ACQ_COST_DELTA_VISIBLE_MS;
      fallback_job->needs_to_run = true;
      fallback_job->oneshot = true;
    } else if ((!known || fallback_job->glo_blind_search) &&
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
  
  sm_deep_search_run_gps(jobs_data);
  if (is_glo_enabled()) {
    sm_deep_search_run_glo(jobs_data);
  }
  sm_fallback_search_run_gps(jobs_data, now_ms, lgf_age_ms);
  if (is_glo_enabled()) {
    sm_fallback_search_run_glo(jobs_data, now_ms, lgf_age_ms);
  }
}

