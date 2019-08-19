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
#include <string.h>

#include "acq/manage.h"
#include "gtest/gtest.h"
#include "task_generator_api.h"

/** Doppler bin size (Hz) */
#define EXPECTED_DOPPLER_BIN_SIZE_HZ ACQ_FULL_CF_STEP
/** Integration time.
    Exact value is 3.957e-3, should it be 4 or 3.957e-3? */
#define EXPECTED_INTEGRATION_TIME_4MS 4
/** CN0 threshold for accepted peak */
#define EXPECTED_PEAK_CN0_THRESHOLD_DBHZ ACQ_THRESHOLD

/** Test program checking task generator operation
 *
 * Test failure triggers assertion
 *
 * \return 0
 */
TEST(task_generator_test, test_task_generator) {
  /* There is not much to check in Phase 1 */
  acq_job_t job;
  acq_task_search_params_t *acq_param = &job.task_data;
  float doppler_min = code_to_sv_doppler_min(CODE_GPS_L1CA) +
                      code_to_tcxo_doppler_min(CODE_GPS_L1CA);
  float doppler_max = code_to_sv_doppler_max(CODE_GPS_L1CA) +
                      code_to_tcxo_doppler_max(CODE_GPS_L1CA);

  memset(&job, 0, sizeof(job));
  job.mesid = construct_mesid(CODE_GPS_L1CA, 1);

  tg_fill_task(&job);

  EXPECT_EQ(EXPECTED_DOPPLER_BIN_SIZE_HZ, acq_param->freq_bin_size_hz);
  EXPECT_EQ(EXPECTED_INTEGRATION_TIME_4MS, acq_param->integration_time_ms);
  EXPECT_EQ(EXPECTED_PEAK_CN0_THRESHOLD_DBHZ, acq_param->cn0_threshold_dbhz);
  EXPECT_EQ(doppler_min, acq_param->doppler_min_hz);
  EXPECT_EQ(doppler_max, acq_param->doppler_max_hz);
}
