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
#include <dum.h>
#include <ndb.h>
#include <timing.h>
#include "task_generator_api.h"
#include "manage.h"
#include "search_manager_api.h"

/** Integration time. */
#define ACQ_INTEGRATION_TIME_4MS 4

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
  acq_param->freq_bin_size_hz = ACQ_FULL_CF_STEP;
  acq_param->integration_time_ms = ACQ_INTEGRATION_TIME_4MS;
  acq_param->cn0_threshold_dbhz = ACQ_THRESHOLD;

  float default_doppler_min = code_to_sv_doppler_min(job->mesid.code) +
                              code_to_tcxo_doppler_min(job->mesid.code);
  float default_doppler_max = code_to_sv_doppler_max(job->mesid.code) +
                              code_to_tcxo_doppler_max(job->mesid.code);

  switch(job->job_type) {
  case ACQ_JOB_DEEP_SEARCH:
  {
    last_good_fix_t lgf;
    gps_time_t now = get_current_time();

    if (TOW_UNKNOWN != now.tow &&
        WN_UNKNOWN != now.wn &&
        NDB_ERR_NONE == ndb_lgf_read(&lgf)) {
      dum_get_doppler_wndw(&job->mesid, &now, &lgf,
                           ACQ_MAX_USER_VELOCITY_MPS,
                           &acq_param->doppler_min_hz,
                           &acq_param->doppler_max_hz);
      break;
    } /* else fall through */
  }
  case ACQ_JOB_FALLBACK_SEARCH:
    acq_param->doppler_min_hz = default_doppler_min;
    acq_param->doppler_max_hz = default_doppler_max;
    break;
  case ACQ_NUM_JOB_TYPES:
  default:
    assert(!"Invalid jobtype");
    acq_param->doppler_min_hz = default_doppler_min;
    acq_param->doppler_max_hz = default_doppler_max;
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
