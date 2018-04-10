/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "sbas_reacq.h"
#include <assert.h>
#include <libswiftnav/signal.h>
#include "hal/piksi_systime.h"
#include "search_manager_api.h"

/** Raised priorities last this long [s] */
#define SBAS_REACQ_PRIORITY_TIMEOUT_S 60

/** Raises re-acq priority of mesid */
void sbas_reacq_prioritize(const me_gnss_signal_t *mesid) {
  assert(IS_SBAS(*mesid));

  const u16 idx = sm_constellation_to_start_index(CONSTELLATION_SBAS);
  acq_jobs_state_t *jobs_data = &acq_all_jobs_state_data;

  u16 i = mesid_to_code_index(*mesid);
  acq_job_t *job = &jobs_data->jobs[ACQ_JOB_DEEP_SEARCH][idx + i];

  job->priority.raise = true;
}

/**
 * Returns the mask of SBAS signals with raised priorities.
 * The returned mask is a result of AND operation with input mask.
 * If no SBAS signals with elevated priorities, then input mask is returned
 * \param mask the mask of selected SBAS signals base on SBAS provider in use
 * \return The mask of SBAS signals with elevated priorities or input mask
 */
u32 sbas_reacq_get_priority_mask(u32 mask) {
  const u16 idx = sm_constellation_to_start_index(CONSTELLATION_SBAS);
  const u16 num_sv = constellation_to_sat_count(CONSTELLATION_SBAS);
  acq_jobs_state_t *jobs_data = &acq_all_jobs_state_data;

  u32 priomask = 0;

  for (u8 i = 0; i < num_sv; i++) {
    acq_job_t *job = &jobs_data->jobs[ACQ_JOB_DEEP_SEARCH][idx + i];

    if (job->priority.raise) {
      job->priority.duration_s = SBAS_REACQ_PRIORITY_TIMEOUT_S;
      piksi_systime_get(&job->priority.started_at);
      job->priority.raise = false;
      log_info_mesid(job->mesid,
                     "re-acq prioritized for %" PRIu8 " seconds",
                     job->priority.duration_s);
    }

    if (0 == (mask & ((u32)1 << i))) {
      job->priority.duration_s = 0;
      continue;
    }
    if (0 == job->priority.duration_s) {
      continue;
    }
    u64 elapsed_s = piksi_systime_elapsed_since_s(&job->priority.started_at);
    if (elapsed_s > job->priority.duration_s) {
      job->priority.duration_s = 0;
      continue;
    }
    priomask |= (u32)1 << i;
  }

  return (0 == priomask) ? mask : priomask;
}
