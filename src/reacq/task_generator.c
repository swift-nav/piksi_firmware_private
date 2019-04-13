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
#include <ndb/ndb.h>

#include "acq/manage.h"
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

  const code_t code = job->mesid.code;
  acq_param->doppler_min_hz =
      code_to_sv_doppler_min(code) + code_to_tcxo_doppler_min(code);
  acq_param->doppler_max_hz =
      code_to_sv_doppler_max(code) + code_to_tcxo_doppler_max(code);

  u16 sat = sm_mesid_to_sat(job->mesid);
  if (GLO_ORBIT_SLOT_UNKNOWN == sat) {
    return;
  }
  gps_time_t now = get_current_time();
  if (!gps_time_valid(&now)) {
    return;
  }
  last_good_fix_t lgf;
  if (NDB_ERR_NONE != ndb_cached_lgf_read(&lgf)) {
    return;
  }

  gnss_signal_t sid = construct_sid(job->mesid.code, sat);
  dum_get_doppler_wndw(&sid,
                       &now,
                       &lgf,
                       MAX_USER_VELOCITY_MPS,
                       &acq_param->doppler_min_hz,
                       &acq_param->doppler_max_hz);
}
