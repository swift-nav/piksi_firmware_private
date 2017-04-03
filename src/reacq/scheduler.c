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
#include <assert.h>
#include "scheduler_api.h"
#include "task_generator_api.h"
#include <manage.h>
#include <track.h>
#include <timing.h>
#include <dum.h>

/* Scheduler utils funcions */
void sch_send_acq_profile_msg(const acq_job_t *job,
                              const acq_result_t *acq_result,
                              bool peak_found);
/* Scheduler constants */

/** Avoid busy loop by sleeping if there are no jobs to run. */
#define ACQ_SLEEP_TIMEOUT_MS 1000

/** Initialize job cost
 *
 *  Initializes job cost according to given cost
 *  hint. Loops other runnable jobs to find minimum,
 *  maximum and average cost of other jobs.
 *
 * \param init_job job whose cost is initialized
 * \param all_jobs_data pointer to jobs data
 *
 * \return none
 */
void sch_initialize_cost(acq_job_t *init_job,
                         const acq_jobs_state_t *all_jobs_data)
{
  acq_job_types_e type;
  u32 min_cost = 0;
  u32 max_cost = 0;
  bool min_found = false;
  bool max_found = false;
  u32 avg = 0;
  u32 num_jobs = 0;
  
  for (type = 0; type < ACQ_NUM_JOB_TYPES; type++) {
    int i;
    for (i = 0; i < ACQ_NUM_SVS; i++) {
      const acq_job_t *job = &all_jobs_data->jobs[type][i];
      if (job->state != ACQ_STATE_WAIT) {
        continue; /* Check only jobs which can run */
      }
      if (!max_found || job->cost > max_cost) {
        max_cost = job->cost;
        max_found = true;
      }
      if (!min_found || job->cost < min_cost) {
        min_cost = job->cost;
        min_found = true;
      }
      avg = avg + job->cost;
      assert(avg >= job->cost);
      num_jobs++;
    }
  }
  if (0 != num_jobs) {
    avg = avg / num_jobs;
  }

  switch (init_job->cost_hint) {
  case ACQ_COST_MIN:
    init_job->cost = MAX(init_job->cost, min_cost);
    break;
  case ACQ_COST_AVG:
    if (0 == init_job->cost) {
      init_job->cost = avg + init_job->cost_delta;
    } else {
      init_job->cost = MIN(init_job->cost, avg + init_job->cost_delta);
    }
    break;
  case ACQ_COST_MAX:
    init_job->cost = max_cost;
    break;
  case ACQ_COST_MAX_PLUS:
    if (0 == init_job->cost) {
      init_job->cost = max_cost + init_job->cost_delta;
    } else {
      init_job->cost = MIN(init_job->cost, max_cost + init_job->cost_delta);
    }
    break;
  default:
    assert(!"Invalid cost hint");
    init_job->cost = max_cost + init_job->cost_delta;
    break;
  }
  
}

/** Limit job cost
 *
 *  Avoid continously increasing job costs by subtracting
 *  minimum of costs from all the job costs. Only the
 *  difference between costs matters for scheduling,
 *  not the absolute value.
 *
 * \param all_jobs_data pointer to jobs data
 * \param cost cumulative cost of just finished job
 *
 * \return none
 */
static void sch_limit_costs(acq_jobs_state_t *all_jobs_data, u32 cost)
{
  acq_job_types_e type;
  u32 min_cost = cost;

  for (type = 0; type < ACQ_NUM_JOB_TYPES; type++) {
    int i;
    for (i = 0; i < ACQ_NUM_SVS; i++) {
      const acq_job_t *job = &all_jobs_data->jobs[type][i];
      if (job->state != ACQ_STATE_WAIT) {
        continue; /* Select only jobs which can run */
      }
      if (job->cost < min_cost) {
        min_cost = job->cost;
      }
    }
  }
  if (min_cost != 0) {
    for (type = 0; type < ACQ_NUM_JOB_TYPES; type++) {
      int i;
      for (i = 0; i < ACQ_NUM_SVS; i++) {
        acq_job_t *job = &all_jobs_data->jobs[type][i];
        if (job->state != ACQ_STATE_WAIT) {
          continue; /* Select only jobs which can run */
        }
        if (job->cost < min_cost) {
          job->cost = 0;
        } else {
          job->cost -= min_cost;
        }
      }
    }
  }
}

/** Select next job to run
 *
 *  Loops jobs, updates their state if search manager
 *  has set them to run/not run and selects the job 
 *  which should run next.
 *
 * \param jobs_data pointer to job data
 *
 * \return job to be run or NULL if there is no job to run
 */
acq_job_t *sch_select_job(acq_jobs_state_t *jobs_data)
{
  acq_job_types_e type;
  acq_job_t *job_to_run = NULL;

  /* Update state and initialize first cost with max, min, avg cost hints */
  for (type = 0; type < ACQ_NUM_JOB_TYPES; type++) {
    int i;
    for (i = 0; i < ACQ_NUM_SVS; i++) {
      acq_job_t *job = &jobs_data->jobs[type][i];
      acq_task_t *task = &job->task_data;
      assert(job->job_type < ACQ_NUM_JOB_TYPES);
      if (ACQ_STATE_WAIT == job->state && !job->needs_to_run) {
        job->state = ACQ_STATE_IDLE;
      }
      if (ACQ_STATE_WAIT == job->state && job->needs_restart) {
        task->task_index = ACQ_UNINITIALIZED_TASKS;
      }
      if (ACQ_STATE_IDLE == job->state &&
          job->needs_to_run &&
          ACQ_COST_MAX_PLUS != job->cost_hint) {
        job->state = ACQ_STATE_WAIT;
        sch_initialize_cost(job, jobs_data);
        task->task_index = ACQ_UNINITIALIZED_TASKS;
      }
    }
  }
  /* Initialize the cost with max_plus cost hint only after jobs
     with max, min, or avg cost hints are initialized since
     the intention of max_plus is to get high cost. 
     Select the job with minimum cost in the same loop. */
  for (type = 0; type < ACQ_NUM_JOB_TYPES; type++) {
    int i;
    for (i = 0; i < ACQ_NUM_SVS; i++) {
      acq_job_t *job = &jobs_data->jobs[type][i];
      acq_task_t *task = &job->task_data;
      assert(job->job_type < ACQ_NUM_JOB_TYPES);
      /* Triggers only on ACQ_COST_MAX_PLUS cost hint */
      if (ACQ_STATE_IDLE == job->state &&
          job->needs_to_run &&
          ACQ_COST_MAX_PLUS == job->cost_hint) {
        sch_initialize_cost(job, jobs_data);
        job->state = ACQ_STATE_WAIT;
        task->task_index = ACQ_UNINITIALIZED_TASKS;
      }
      /* Find minimum cost */
      if (ACQ_STATE_WAIT == job->state) {
        if (NULL == job_to_run || job->cost < job_to_run->cost) {
          job_to_run = job;
        }
      }
    }
  }
  
  return job_to_run;
}

/* Search manager API functions */

/** Run scheduler
 *
 *  Schedules search jobs to be run and runs the acquisition on HW.
 *  Scheduling principle follows the Completely Fair Scheduler (CFS) 
 *  but instead of having ordered tree as underlying data structure, the jobs
 *  are stored simply in fixed array.
 *
 *  Scheduler maintains the amount of HW time (cost) provided to a given job,
 *  and selects the job with minimum cost to run.
 *
 *  For CFS see e.g.:
 *  http://www.ibm.com/developerworks/linux/library/l-completely-fair-scheduler/
 *  https://en.wikipedia.org/wiki/Completely_Fair_Scheduler
 *
 * \param jobs_data pointer to job data
 *
 * \return none
 */
void sch_run(acq_jobs_state_t *jobs_data)
{
  acq_job_t *job = sch_select_job(jobs_data);
  acq_task_t *task = &job->task_data;
  acq_task_search_params_t *acq_param;
  acq_result_t acq_result;
  bool peak_found;

  if (NULL == job) {
    chThdSleepMilliseconds(ACQ_SLEEP_TIMEOUT_MS);
    return;
  } 
    
  task->task_index++;
  if (0 == task->task_index) {
    tg_fill_task(job);
  } else {
    assert(!"Expecting only task index 0 in Phase 1");
    task->task_index = 0;
  }
  /* Sanity check */
  if (task->task_index > task->number_of_tasks ||
      task->task_index > ACQ_MAX_NUM_TASKS) {
    assert(!"Too many tasks");
    task->task_index = 0;
  }
    
  acq_param = &task->task_array[task->task_index];
  job->state = ACQ_STATE_RUN;

  assert(mesid_valid(job->mesid));

  job->start_time = timing_getms();

  peak_found = acq_search(job->mesid,
                          acq_param->doppler_min_hz,
                          acq_param->doppler_max_hz,
                          acq_param->freq_bin_size_hz,
                          &acq_result);

  job->stop_time = timing_getms();

  /* Update cost with spent HW time. Limit with 1 ms minimum
     since 0 update would stuck scheduling. */
  job->cost += (u32)MAX(1, job->stop_time - job->start_time);
  sch_limit_costs(jobs_data, job->cost);

  /* It is unclear should peak checks take place in acq module
     or here. */
  if (peak_found && acq_result.cn0 < acq_param->cn0_threshold_dbhz) {
    peak_found = false;
  }

  if (peak_found) { /* Send to track */
    tracking_startup_params_t tracking_startup_params = {
      .mesid = job->mesid,
      .sample_count = acq_result.sample_count,
      .carrier_freq = acq_result.cf,
      .code_phase = acq_result.cp,
      .chips_to_correlate = code_to_chip_count(job->mesid.code),
      .cn0_init = acq_result.cn0,
      .elevation = TRACKING_ELEVATION_UNKNOWN
    };
    task->task_index = ACQ_UNINITIALIZED_TASKS;
    job->state = ACQ_STATE_IDLE;

    tracking_startup_request(&tracking_startup_params);

  } else { /* No peak */
    if (task->task_index >= task->number_of_tasks - 1) {
      /* No more tasks to run */
      task->task_index = ACQ_UNINITIALIZED_TASKS;
      if (job->oneshot) {
        job->state = ACQ_STATE_IDLE;
      } else {
        job->state = ACQ_STATE_WAIT;
      }
    } else {
      /* Task index is incremented when task is run on HW */
      job->state = ACQ_STATE_WAIT;
    }
  } /* No peak */

  sch_send_acq_profile_msg(job, &acq_result, peak_found);

  /* Send result of an acquisition to the host. */
  acq_result_send(job->mesid, acq_result.cn0, acq_result.cp, acq_result.cf);
}

