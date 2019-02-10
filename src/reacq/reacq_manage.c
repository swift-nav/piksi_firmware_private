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

#include "scheduler_api.h"
#include "search_manager_api.h"

/**
 * Reacqusition controller initialization.
 *
 * The method performs all reacqusition related initialization.
 *
 * \return None
 */
void init_reacq(void) { sm_init(&acq_all_jobs_state_data); }

/**
 * Reacqusition controller loop.
 *
 * The method performs all reacqusition control functions within acquisition
 * thread scope.
 *
 * \return None
 */
void manage_reacq(void) {
  static reacq_sched_ret_t last_job_type = REACQ_DONE_NOTHING;
  sm_run(&acq_all_jobs_state_data, last_job_type);
  last_job_type = sch_run(&acq_all_jobs_state_data);
}
