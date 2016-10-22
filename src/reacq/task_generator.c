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

#include "task_generator_api.h"

/** Doppler bin size (Hz) */
#define ACQ_DOPPLER_BIN_SIZE_HZ 168
/** Integration time. 
    Exact value is 3.957e-3, should it be 4 or 3.957e-3? */
#define ACQ_INTEGRATION_TIME_4MS 4 
/** CN0 threshold for accepted peak */
#define ACQ_PEAK_CN0_THRESHOLD_DBHZ 37.0f
/** Default maximum Doppler (Hz) */
#define ACQ_DOPPLER_MAX_HZ 8500
/** Default minimum Doppler (Hz) */
#define ACQ_DOPPLER_MIN_HZ -8500


/** Fills job task(s) with acquisition parameters
 *
 * \param job job in which task data is filled
 *
 * \return none
 */

void tg_fill_task(acq_job_t *job)
{
  acq_task_search_params_t *acq_param;
  s16 task_index = 0; /* Single task in Phase 1 */
  job->task_data.number_of_tasks = 1;
  acq_param = &job->task_data.task_array[task_index];
  acq_param->freq_bin_size_hz = ACQ_DOPPLER_BIN_SIZE_HZ;
  acq_param->integration_time_ms = ACQ_INTEGRATION_TIME_4MS;
  acq_param->cn0_threshold_dbhz = ACQ_PEAK_CN0_THRESHOLD_DBHZ;

  switch(job->job_type) {
  case ACQ_JOB_DEEP_SEARCH:
    /* TBD do we already have the uncertainty management module.
       Predicted values should be used here. */
    acq_param->doppler_min_hz = ACQ_DOPPLER_MIN_HZ;
    acq_param->doppler_max_hz = ACQ_DOPPLER_MAX_HZ;
    break;
  case ACQ_JOB_FALLBACK_SEARCH:
    acq_param->doppler_min_hz = ACQ_DOPPLER_MIN_HZ;
    acq_param->doppler_max_hz = ACQ_DOPPLER_MAX_HZ;
    break;
  case ACQ_NUM_JOB_TYPES:
  default:
    log_error("Invalid jobtype %d",(s32)job->job_type);
    acq_param->doppler_min_hz = ACQ_DOPPLER_MIN_HZ;
    acq_param->doppler_max_hz = ACQ_DOPPLER_MAX_HZ;
    break;
  }
}
/** Checks if job search space has changed drasticaly
 *
 *  Sets restart flag on job data if serach space has changed
 *  drastically. This function is not implemented in Phase 1
 *  reacquistion logic.
 *
 * \param job job whose search space is checked.
 *
 * \return none
 */
void tg_check_uncertainty_change(acq_job_t *job)
{
  (void)job;
}
