/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#include <math.h>

#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>

#include "gtest/gtest.h"

#include "run_stats/run_stats.h"

TEST(run_stats_test, test_run_stats) {
  running_stats_t stat;

  running_stats_init(&stat, 0.01);

  for (double i = -1.; i < 0; i += 1e-3) {
    running_stats_update(&stat, i);
  }

  double mean = running_stats_get_mean(&stat);
  EXPECT_TRUE(fabs(mean) <= 0.1);
  double std = running_stats_get_std(&stat);
  EXPECT_TRUE(fabs(std) <= 1e-1);

  for (double i = 0; i < 1; i += 1e-3) {
    running_stats_update(&stat, i);
  }

  mean = running_stats_get_mean(&stat);
  EXPECT_TRUE(fabs(mean - 0.9) <= 0.11);

  std = running_stats_get_std(&stat);
  EXPECT_TRUE(fabs(std) <= 1e-1);
}
