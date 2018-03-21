/*
 * Copyright (C) 2014-2017 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#ifndef STARLING_CALC_PVT_THREAD_TIME_MATCHED_H
#define STARLING_CALC_PVT_THREAD_TIME_MATCHED_H

#define TIME_MATCHED_THREAD_NAME "time matched obs"

/* Warn on 15 second base station observation latency */
#define BASE_LATENCY_TIMEOUT 15

/** number of milliseconds before SPP resumes in pseudo-absolute mode */
#define DGNSS_TIMEOUT_MS 5000

typedef enum {
  SOLN_MODE_LOW_LATENCY,
  SOLN_MODE_TIME_MATCHED,
  SOLN_MODE_NO_DGNSS
} dgnss_solution_mode_t;

void time_matched_obs_thread(void *arg);

#endif
