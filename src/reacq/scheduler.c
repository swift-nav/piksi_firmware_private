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
#include <swiftnav/glo_map.h>
#include "dum/dum.h"
#include "manage.h"
#include "scheduler_api.h"
#include "task_generator_api.h"
#include "timing/timing.h"

#include "soft_macq/soft_macq_main.h"

/* Scheduler utils funcions */
void sch_send_acq_profile_msg(const acq_job_t *job,
                              const acq_result_t *acq_result,
                              bool peak_found);
u16 sm_constellation_to_start_index(constellation_t gnss);

/* Scheduler constants */

/** Avoid busy loop by sleeping if there are no jobs to run. */
#define ACQ_SLEEP_TIMEOUT_MS 10

/** Select next job to run
 *
 *  Loops jobs, updates their state if search manager
 *  has set them to run/not run and selects the job
 *  which should run next.
 *
 * \param jobs_data pointer to job data
 * \param job_to_run result pointer of job selected

 * \return true if need to continue, false if it's OK
 *         to re-run the schedule after this round
 */
static bool sch_select_job(acq_jobs_state_t *jobs_data, acq_job_t **job_to_run) {
  assert(job_to_run);

  (*job_to_run) = NULL;

  u16 idx = sm_constellation_to_start_index(jobs_data->constellation);
  u16 num_sats = constellation_to_sat_count(jobs_data->constellation);

  /* Run first ready and visible satellite */
  acq_job_t *job = &jobs_data->jobs[idx];
  for (u8 i = 0; i < num_sats; i++, job++) {
    if ((ACQ_STATE_WAIT == job->state) && (VISIBLE == job->sky_status)) {
      (*job_to_run) = job;
      return true;
    }
  }

  /* Run first unknown satellite */
  job = &jobs_data->jobs[idx];
  for (u8 i = 0; i < num_sats; i++, job++) {
    /* Triggers only on ACQ_COST_MAX_PLUS cost hint */
    if ((ACQ_STATE_WAIT == job->state) && (UNKNOWN == job->sky_status)) {
      (*job_to_run) = job;
      return false;
    }
  }

  /* Run first invisible satellite */
  job = &jobs_data->jobs[idx];
  for (u8 i = 0; i < num_sats; i++, job++) {
    if ((ACQ_STATE_WAIT == job->state) && (INVISIBLE == job->sky_status)) {
      (*job_to_run) = job;
      return false;
    }
  }

  return false;
}

/** GLO specific function.
 * The function sets Frequency slot to search if we are in blind search mode
 *
 * \param job pointer to job to run
 */
static void sch_glo_fcn_set(acq_job_t *job) {
  if (NULL == job) {
    return;
  }

  u16 slot_id1, slot_id2;
  if (!glo_map_valid(job->sid)) {
    bool next = true;
    do {
      /* FCN not mapped to GLO slot ID, so perform blind search, just pick next
       * FCN */
      job->mesid.sat++;
      if (job->mesid.sat > GLO_MAX_FCN) {
        job->mesid.sat = GLO_MIN_FCN;
      }
      /* now check if the selected frequency already mapped to other slot id */
      if (glo_map_get_slot_id(job->mesid.sat, &slot_id1, &slot_id2) == 0) {
        /* selected frequency is not mapped to other slot id, so use it for
         * acquisition */
        next = false;
      }
    } while (next);
    job->mesid.code = job->sid.code;
  }
}

/** Common part of scheduler for all constellations
 *
 * \param job pointer to job to run
 */
static void sch_run_common(acq_job_t *job) {
  if (NULL == job) {
    chThdSleepMilliseconds(ACQ_SLEEP_TIMEOUT_MS);
    return;
  }
  assert(mesid_valid(job->mesid));

  job->start_time = timing_getms();
  tg_fill_task(job);

  acq_result_t acq_result;
  acq_task_search_params_t *acq_param = &job->task_data;
  bool peak_found = soft_multi_acq_search(job->mesid,
                                     acq_param->doppler_min_hz,
                                     acq_param->doppler_max_hz,
                                     &acq_result);

  job->stop_time = timing_getms();
  job->state = ACQ_STATE_IDLE;

  if (peak_found) { /* Send to track */
    u16 glo_orbit_slot = GLO_ORBIT_SLOT_UNKNOWN;
    if (IS_GLO(job->mesid)) {
      glo_orbit_slot = get_orbit_slot(job->mesid.sat);
    }

    me_gnss_signal_t mesid_trk = job->mesid;
    float cp = acq_result.cp;
    float df_hz = acq_result.df_hz;

    tracking_startup_params_t tracking_startup_params = {
        .mesid = mesid_trk,
        .sample_count = acq_result.sample_count,
        .doppler_hz = df_hz,
        .code_phase = cp,
        .chips_to_correlate = code_to_chip_count(mesid_trk.code),
        .cn0_init = acq_result.cn0,
        .glo_slot_id = glo_orbit_slot};
    job->state = ACQ_STATE_IDLE;

    tracking_startup_request(&tracking_startup_params);
  }

  sch_send_acq_profile_msg(job, &acq_result, peak_found);

  /* Send result of an acquisition to the host. */
  acq_result_send(job->mesid, acq_result.cn0, acq_result.cp, acq_result.df_hz);
}

/* Search manager API functions */

/** Run scheduler
 *
 * \param jobs_data pointer to job data
 *
 * \return none
 */
void sch_run(acq_jobs_state_t *jobs_data) {
  acq_job_t *job;
  bool run = false;
  do {
    run = sch_select_job(jobs_data, &job);
    if (CONSTELLATION_GLO == jobs_data->constellation) {
      sch_glo_fcn_set(job);
    }
    sch_run_common(job);
  } while (run);
}
