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
#ifndef SWIFTNAV_SEARCH_MANAGER_API_H
#define SWIFTNAV_SEARCH_MANAGER_API_H

#include <stdbool.h>

#include "search_manager_utils.h"
#include "signal_db/signal_db.h"

/* Search manager constants */

/** Minimum interval between searches of a visible SV (ms) */
#define REACQ_MIN_SEARCH_INTERVAL_VISIBLE_MS 500

/** Minimum interval between searches of an unknown SV (ms) */
#define REACQ_MIN_SEARCH_INTERVAL_UNKNOWN_MS 10000

/** Minimum interval between searches of an invisible SV (ms) */
#define REACQ_MIN_SEARCH_INTERVAL_INVISIBLE_MS 60000

/** High priority GPS search happens below this number of reacquired satellites
 */
#define LOW_GPS_L1CA_SV_LIMIT 6

/** Total number of re-acq slots */
#define REACQ_NUM_SAT                                                          \
  (NUM_SATS_GPS + NUM_SATS_GAL + NUM_SATS_SBAS + NUM_SATS_QZS + NUM_SATS_GLO + \
   NUM_SATS_BDS)

/** Predicted status of satellite. */
typedef enum { INVISIBLE = -1, UNKNOWN = 0, VISIBLE = +1 } visibility_t;

/** Scheduler return value. */
typedef enum {
  REACQ_DONE_VISIBLE,
  REACQ_DONE_UNKNOWN,
  REACQ_DONE_INVISIBLE,
  REACQ_DONE_NOTHING,
} reacq_sched_ret_t;

/** State of job in scheduling */
typedef enum {
  ACQ_STATE_IDLE, /**< Job is idling */
  ACQ_STATE_WAIT, /**< Job waits to get running */
} acq_job_scheduling_state_e;

/** Acquisition parameters which are passed to hardware */
typedef struct {
  float doppler_min_hz;     /**< Search window minimum frequency (Hz) */
  float doppler_max_hz;     /**< Search window maximum frequency (Hz) */
  float freq_bin_size_hz;   /**< Frequency bin size (Hz) */
  float cn0_threshold_dbhz; /**< Peaks bellow this are disregarded (dBHz) */
  u8 integration_time_ms;   /**< Coherent integration time (ms) */
} acq_task_search_params_t;

/** Search jobs */
typedef struct {
  me_gnss_signal_t mesid;  /**< ME SV identifier */
  gnss_signal_t sid;       /**< SV identifier, used to fetch ephemeris */
  u64 start_time;          /**< HW millisecond when job finished */
  u64 stop_time;           /**< HW millisecond when job finished */
  visibility_t sky_status; /**< Set when this job needs to run */
  acq_job_scheduling_state_e state;   /**< Scheduling state */
  acq_task_search_params_t task_data; /**< Acquisition parameters */
} acq_job_t;

/** Container for all the jobs */
typedef struct {
  /**< jobs for all constellations
   * Start index of any used GNSS can be obtain using function
   * sm_constellation_to_start_index() */
  acq_job_t jobs[REACQ_NUM_SAT];
} acq_jobs_state_t;

/** Global data of all the jobs is shared between search manager
    and scheduler */
extern acq_jobs_state_t acq_all_jobs_state_data;

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void sm_init(acq_jobs_state_t *data);
void sm_restore_jobs(acq_jobs_state_t *jobs_data,
                     reacq_sched_ret_t last_job_type);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* SWIFTNAV_SEARCH_MANAGER_API_H */
