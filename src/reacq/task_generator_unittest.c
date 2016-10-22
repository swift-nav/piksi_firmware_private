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
#include <string.h>
#include <assert.h>
#include "task_generator_api.h"

/* Compile unit test in src/reacq directory with:
 gcc  -Wall -std=c99 task_generator.c task_generator_unittest.c \
 ../../libswiftnav/src/signal.c \
 -I.. -I../../libswiftnav/include -I ../../src/board/v3/ \
 -I../../ChibiOS/os/rt/include/ -I../../ChibiOS/os/rt/templates -o tg_unittest
*/

/** Doppler bin size (Hz) */
#define EXPECTED_DOPPLER_BIN_SIZE_HZ 168
/** Integration time. 
    Exact value is 3.957e-3, should it be 4 or 3.957e-3? */
#define EXPECTED_INTEGRATION_TIME_4MS 4 
/** CN0 threshold for accepted peak */
#define EXPECTED_PEAK_CN0_THRESHOLD_DBHZ 37.0f
/** Default maximum Doppler (Hz) */
#define EXPECTED_DOPPLER_MAX_HZ 8500
/** Default minimum Doppler (Hz) */
#define EXPECTED_DOPPLER_MIN_HZ -8500

/** Test program checking task generator operation
 *
 * Test failure triggers assertion
 *
 * \param none
 *
 * \return 0
 */
int main(int argc, char **argv)
{
  /* There is not much to check in Phase 1 */
  acq_job_t job;
  acq_task_search_params_t *acq_param =  &job.task_data.task_array[0];
  memset(&job, 0, sizeof(job));

  job.job_type = ACQ_JOB_FALLBACK_SEARCH;
  tg_fill_task(&job);
  
  assert(job.task_data.number_of_tasks == 1);
  assert(EXPECTED_DOPPLER_BIN_SIZE_HZ == acq_param->freq_bin_size_hz);
  assert(EXPECTED_INTEGRATION_TIME_4MS == acq_param->integration_time_ms);
  assert(EXPECTED_PEAK_CN0_THRESHOLD_DBHZ == acq_param->cn0_threshold_dbhz);

  /* No uncertainty module yet */
  job.job_type = ACQ_JOB_DEEP_SEARCH;
  tg_fill_task(&job);  

  assert(job.task_data.number_of_tasks == 1);
  assert(EXPECTED_DOPPLER_BIN_SIZE_HZ == acq_param->freq_bin_size_hz);
  assert(EXPECTED_INTEGRATION_TIME_4MS == acq_param->integration_time_ms);
  assert(EXPECTED_PEAK_CN0_THRESHOLD_DBHZ == acq_param->cn0_threshold_dbhz);

  return 0;
}
