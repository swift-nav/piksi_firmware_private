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

#include "manage.h"

#include <assert.h>
#include <ndb/ndb.h>
#include "dum/dum.h"
#include "me_constants.h"
#include "search_manager_api.h"
#include "task_generator_api.h"
#include "timing/timing.h"

/** Integration time. */
#define ACQ_INTEGRATION_TIME_4MS 4

/** Fills job task(s) with acquisition parameters
 *
 * \param job job in which task data is filled
 *
 * \return none
 */

void tg_fill_task(acq_job_t *job) {
  acq_task_search_params_t *acq_param = &job->task_data;
  acq_param->freq_bin_size_hz = ACQ_FULL_CF_STEP;
  acq_param->integration_time_ms = ACQ_INTEGRATION_TIME_4MS;
  acq_param->cn0_threshold_dbhz = ACQ_THRESHOLD;

  float default_doppler_min = code_to_sv_doppler_min(job->sid.code) +
                              code_to_tcxo_doppler_min(job->sid.code);
  float default_doppler_max = code_to_sv_doppler_max(job->sid.code) +
                              code_to_tcxo_doppler_max(job->sid.code);


  last_good_fix_t lgf;
  gps_time_t now = get_current_time();

  if (TOW_UNKNOWN != now.tow && WN_UNKNOWN != now.wn &&
      NDB_ERR_NONE == ndb_lgf_read(&lgf)) {
    dum_get_doppler_wndw(&job->sid,
                         &now,
                         &lgf,
                         MAX_USER_VELOCITY_MPS,
                         &acq_param->doppler_min_hz,
                         &acq_param->doppler_max_hz);
  } else {
    acq_param->doppler_min_hz = default_doppler_min;
    acq_param->doppler_max_hz = default_doppler_max;
  }
}
/** Checks if job search space has changed drastically
 *
 *  Sets restart flag on job data if search space has changed
 *  drastically. This function is not implemented in Phase 1
 *  reacquistion logic.
 *
 * \param job job whose search space is checked.
 *
 * \return none
 */
void tg_check_uncertainty_change(acq_job_t *job) { (void)job; }
