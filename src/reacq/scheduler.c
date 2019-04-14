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

#include "acq/manage.h"
#include "dum/dum.h"
#include "reacq/reacq_sbp_utility.h"
#include "scheduler_api.h"
#include "soft_macq/soft_macq_main.h"
#include "task_generator_api.h"
#include "timing/timing.h"

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
reacq_sched_ret_t sch_select_job(acq_jobs_context_t *jobs_data,
                                 acq_job_t **job_to_run) {
  assert(job_to_run);

  (*job_to_run) = NULL;

  /* Run first ready and visible satellite */
  acq_job_t *job = &jobs_data->jobs[0];
  for (u8 i = 0; i < REACQ_NUM_SAT; i++, job++) {
    if ((ACQ_STATE_WAIT == job->state) && (VISIBLE == job->sky_status)) {
      (*job_to_run) = job;
      log_debug("reacq: %3d %2d +1", job->mesid.sat, job->mesid.code);
      return REACQ_DONE_VISIBLE;
    }
  }

  /* Run first unknown satellite */
  job = &jobs_data->jobs[0];
  for (u8 i = 0; i < REACQ_NUM_SAT; i++, job++) {
    if ((ACQ_STATE_WAIT == job->state) && (UNKNOWN == job->sky_status)) {
      (*job_to_run) = job;
      log_debug("reacq: %3d %2d  0", job->mesid.sat, job->mesid.code);
      return REACQ_DONE_UNKNOWN;
    }
  }

  /* Run first invisible satellite */
  job = &jobs_data->jobs[0];
  for (u8 i = 0; i < REACQ_NUM_SAT; i++, job++) {
    if ((ACQ_STATE_WAIT == job->state) && (INVISIBLE == job->sky_status)) {
      (*job_to_run) = job;
      log_debug("reacq: %3d %2d -1", job->mesid.sat, job->mesid.code);
      return REACQ_DONE_INVISIBLE;
    }
  }

  log_debug("sch_select_job() found nothing to do");

  return REACQ_DONE_NOTHING;
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

  job->start_time_ms = timing_getms();
  tg_fill_task(job);

  acq_result_t acq_result;
  acq_task_search_params_t *acq_param = &job->task_data;
  bool peak_found = soft_multi_acq_search(job->mesid,
                                          acq_param->doppler_min_hz,
                                          acq_param->doppler_max_hz,
                                          &acq_result);

  job->stop_time_ms = timing_getms();
  job->state = ACQ_STATE_IDLE;

  if (peak_found) { /* Send to track */
    u16 glo_orbit_slot = GLO_ORBIT_SLOT_UNKNOWN;
    if (IS_GLO(job->mesid)) {
      glo_orbit_slot = sm_mesid_to_sat(job->mesid);
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

/** Schedule all visible satellites until one unknown or invisible is done
 *
 * \param jobs_data pointer to job data
 *
 * \return which type of job was run last (including no job)
 */
reacq_sched_ret_t sch_run(acq_jobs_context_t *jobs_data) {
  reacq_sched_ret_t ret = REACQ_DONE_NOTHING;
  acq_job_t *job;
  /* `sch_select_job()` can also do nothing and return NULL in `job` */
  ret = sch_select_job(jobs_data, &job);
  /* `sch_run_common()` just sleeps on a NULL `job` */
  sch_run_common(job);
  return ret;
}
