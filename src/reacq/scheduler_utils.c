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

#include "search_manager_api.h"
#include "reacq_sbp_utility.h"
#include <sbp_utils.h>
#include <manage.h>

/** Populate acq_sv_profile message and send it out
 *
 * \param job job data which is filled in message
 * \param acq_result acquisition results
 * \param peak_found true if job found peak and acq_result contains
 *        peak details, otherwise false
 *
 * \return none
 */

void sch_send_acq_profile_msg(const acq_job_t *job,
			      const acq_result_t *acq_result,
			      bool peak_found)
{
  acq_sv_profile_t prof;
  /* In Phase 1, task 0 covers full job search space range. 
     If there were more tasks, they all together would cover job search
     space range */
  const acq_task_search_params_t *acq_params = &job->task_data.task_array[0];
  
  prof.job_type = (u8)job->job_type;
  prof.status = peak_found;
  prof.cn0 = (u16)(10 * acq_result->cn0);
  prof.int_time = acq_params->integration_time_ms;
  prof.sid = sid_to_sbp(job->sid);
  prof.bin_width = acq_params->freq_bin_size_hz;
  prof.timestamp = (u32)job->stop_time;
  prof.time_spent = (u32)1000*(job->stop_time - job->start_time);
  prof.cf_min = (s32)acq_params->doppler_min_hz;
  prof.cf_max = (s32)acq_params->doppler_max_hz;
  prof.cf = (s32)acq_result->cf;
  prof.cp = (u32)acq_result->cp;

  reacq_sbp_data_process(&prof);
}
