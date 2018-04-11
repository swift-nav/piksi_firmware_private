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

#include "signal_db/signal_db.h"

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

/** Re-acq priority mask length in bits */
#define REACQ_PRIORITY_CYCLE (30)

/** GPS will have high re-acq priority if less than limit SVs is tracked */
#define LOW_GPS_L1CA_SV_LIMIT (6)
/** SBAS will have high re-acq priority if less than limit SVs is tracked */
#define LOW_SBAS_L1CA_SV_LIMIT (1)

/** Re-acq priority levels. */
typedef enum reacq_prio_level_e {
  REACQ_NORMAL_PRIO,
  REACQ_GPS_HIGH_PRIO,
  REACQ_SBAS_HIGH_PRIO,
  REACQ_PRIO_COUNT,
} reacq_prio_level_t;

/** Search job types */
typedef enum {
  ACQ_JOB_DEEP_SEARCH,     /**< Deep job type */
  ACQ_JOB_FALLBACK_SEARCH, /**< Fallback search job type */
  ACQ_NUM_JOB_TYPES        /**< Number of job types */
} acq_job_types_e;

/** Cost hint for job scheduler */
typedef enum {
  ACQ_COST_MIN,     /**< Initialize cost to minimum of all jobs */
  ACQ_COST_AVG,     /**< Initialize cost to average of all jobs */
  ACQ_COST_MAX,     /**< Initialize cost to maximum of all jobs */
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
  float doppler_min_hz;     /**< Search window minimum frequency (Hz) */
  float doppler_max_hz;     /**< Search window maximum frequency (Hz) */
  float freq_bin_size_hz;   /**< Frequency bin size (Hz) */
  float cn0_threshold_dbhz; /**< Peaks bellow this are disregarded (dBHz) */
  u8 integration_time_ms;   /**< Coherent integration time (ms) */
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
  me_gnss_signal_t mesid;    /**< ME SV identifier */
  gnss_signal_t sid;         /**< SV identifier, used to fetch ephemeris */
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
  bool needs_restart;   /**< Set if this job needs to be restarted */
  acq_task_t task_data; /**< Search area is divided into smaller tasks */
} acq_job_t;

/** Container for all the jobs */
typedef struct {
  /**< jobs for GPS, GLO and SBAS for each job type.
   * Sequence of the job must be fixed: GPS, GLO, SBAS. New constellation
   * must be added at the end.
   * Start index of any used GNSS can be obtain using function
   * sm_constellation_to_start_index() */
  acq_job_t jobs[ACQ_NUM_JOB_TYPES][NUM_SATS_GPS + NUM_SATS_GLO +
                                    NUM_SATS_SBAS + NUM_SATS_BDS2 +
                                    NUM_SATS_QZS];
  constellation_t constellation;
  u8 priority_counter;
} acq_jobs_state_t;

/** Global data of all the jobs is shared between search manager
    and scheduler */
extern acq_jobs_state_t acq_all_jobs_state_data;

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void sm_init(acq_jobs_state_t *data);
void sm_constellation_select(acq_jobs_state_t *jobs_data);
void sm_run(acq_jobs_state_t *jobs_data);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* SWIFTNAV_SEARCH_MANAGER_API_H */
