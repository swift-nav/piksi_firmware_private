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
#ifndef SWIFTNAV_SEARCH_MANAGER_API_H
#define SWIFTNAV_SEARCH_MANAGER_API_H

#include <signal.h>

/* Search manager constants */

/** Timeout (ms) defining period between fallback searches of
    visible and unknown SVs */
#define ACQ_FALLBACK_SEARCH_TIMEOUT_VIS_AND_UNKNOWN_MS 8000
/** Timeout (ms) defining period between fallback searches of
    known invisible SVs */
#define ACQ_FALLBACK_SEARCH_TIMEOUT_INVIS_MS 16000
/** Starts fallback searches when last good fix (LGF) is
    older than timeout (ms) */
#define ACQ_LGF_TIMEOUT_VIS_AND_UNKNOWN_MS 1
/** Starts fallback searches of invisible SVs when last good fix (LGF) is
    older than timeout (ms) */
#define ACQ_LGF_TIMEOUT_INVIS_MS 1
/** Max user velocity for visibility calculation (m/s) */
#define ACQ_MAX_USER_VELOCITY_MPS 30.0f

/** Number of SVs whose search jobs are managed */
#define ACQ_NUM_SVS (NUM_SATS_GPS)

/** Maximum number of tasks. 1 when there is no splitter for
    dividing jobs into smaller tasks */
#define ACQ_MAX_NUM_TASKS 1
/** Uninitialized task index */
#define ACQ_UNINITIALIZED_TASKS -1

/** Job cost delta used to avoid clustering of job with equal priority. */
#define ACQ_COST_DELTA_VISIBLE_MS 30

/** Job cost delta used to avoid clustering of job with equal priority. */
#define ACQ_COST_DELTA_UNKNOWN_MS 50

/** Job cost delta used to avoid clustering of job with equal priority. */
#define ACQ_COST_DELTA_INVISIBLE_MS 100

/** This job cost delta is added to the minimum cost computed across all jobs.
    The resulting cost is assigned to the job, which has the hint ACQ_COST_MIN.
    It lets other minimum cost jobs have chance to run as they will have
    marginally smaller cost. */
#define ACQ_COST_DELTA_MIN_MS 1

/** Search job types */
typedef enum {
  ACQ_JOB_DEEP_SEARCH,     /**< Deep job type */
  ACQ_JOB_FALLBACK_SEARCH, /**< Fallback search job type */
  ACQ_NUM_JOB_TYPES        /**< Number of job types */
} acq_job_types_e;

/** Cost hint for job scheduler */
typedef enum {
  ACQ_COST_MIN, /**< Initialize cost to minimum of all jobs */
  ACQ_COST_AVG, /**< Initialize cost to average of all jobs */
  ACQ_COST_MAX, /**< Initialize cost to maximum of all jobs */
  ACQ_COST_MAX_PLUS /**< Initialize cost to maximum of all jobs plus
                       ACQ_COST_DELTA_MS */
} acq_cost_hint_e;

/** State of job in scheduling */
typedef enum {
  ACQ_STATE_IDLE, /**< Job is idling */
  ACQ_STATE_WAIT, /**< Job waits to get running */
  ACQ_STATE_RUN,  /**< Job is running */
} acq_job_scheduling_state_e;

/** Acquisition parameters which are passed to hardware */
typedef struct {
  float doppler_min_hz; /**< Search window minimum frequency (Hz) */
  float doppler_max_hz; /**< Search window maximum frequency (Hz) */
  float freq_bin_size_hz; /**< Frequency bin size (Hz) */
  float cn0_threshold_dbhz; /**< Peaks bellow this are disregarded (dBHz) */
  u8 integration_time_ms; /**< Coherent integration time (ms) */
 } acq_task_search_params_t;

/** Search task defines smallest unit of search work which is passed
    to hardware */
typedef struct {
  u16 number_of_tasks;
  s16 task_index;
  acq_task_search_params_t task_array[ACQ_MAX_NUM_TASKS];
} acq_task_t;

/** Search jobs */
typedef struct {
  gnss_signal_t sid;         /**< SV identifier */
  acq_job_types_e job_type;  /**< Job type */
  u64 start_time;            /**< HW millisecond when job started */
  u64 stop_time;             /**< HW millisecond when job finished */
  u32 cost;                  /**< Cost of job in terms of spent HW time 
                                (milliseconds) */
  acq_cost_hint_e cost_hint; /**< Tells how the cost is initialized */
  u32 cost_delta;            /**< Cost delta */
  bool needs_to_run;         /**< Set when this job needs to run */
  bool oneshot;              /**< Oneshot jobs do not continue automatically 
                                when completed */
  acq_job_scheduling_state_e state; /**< Scheduling state */
  bool needs_restart;        /**< Set if this job needs to be restarted */
  acq_task_t task_data;      /**< Search area is divided into smaller tasks */
} acq_job_t;

/** Container for all the jobs */
typedef struct {
  acq_job_t jobs[ACQ_NUM_JOB_TYPES][ACQ_NUM_SVS]; /**< job for each SV for each
                                                     job type */
} acq_jobs_state_t;

/** Global data of all the jobs is shared between search manager
    and scheduler */
extern acq_jobs_state_t acq_all_jobs_state_data;

void sm_init(acq_jobs_state_t *data);
void sm_run(acq_jobs_state_t *jobs_data);

#endif /* SWIFTNAV_SEARCH_MANAGER_API_H */
