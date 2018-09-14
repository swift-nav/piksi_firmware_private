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
#include <stdio.h>

#include <swiftnav/constants.h>
#include <swiftnav/logging.h>

#include "gtest/gtest.h"

#include "run_stats/run_stats.h"

TEST(run_stats_test, test_run_stats) {
  running_stats_t stat = {.n = 1, .sum = 1, .sum_of_squares = 1};
  double mean, std;

  running_stats_init(&stat);
  EXPECT_TRUE(stat.n == 0 && stat.sum == 0 && stat.sum_of_squares == 0);

  for (int i = 0; i < 10; i++) {
    running_stats_update(&stat, (double)i);
  }
  EXPECT_TRUE(stat.n == 10 && stat.sum == 45 && stat.sum_of_squares == 285);

  running_stats_get_products(&stat, &mean, &std);
  EXPECT_TRUE((mean - 4.5) <= 1e-6 && (std - 3.027650) <= 1e-6);

  running_stats_init(&stat);
  for (int i = 0; i > -10; i--) {
    running_stats_update(&stat, (double)i);
  }
  EXPECT_TRUE(stat.n == 10 && stat.sum == -45 && stat.sum_of_squares == 285);

  running_stats_get_products(&stat, &mean, &std);
  EXPECT_TRUE((mean + 4.5) <= 1e-6 && (std - 3.027650) <= 1e-6);
}
