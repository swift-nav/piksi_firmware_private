/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef CLOCK_FILTER_H
#define CLOCK_FILTER_H

#include <swiftnav/common.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/single_epoch_solver.h>

typedef struct {
  u64 tc;               /**< NAP tick of the estimate */
  gps_time_t t_gps;     /**< GPS time estimate at tc. */
  double tick_length_s; /** The nominal length of the tick in seconds */
  double clock_rate;    /**< Clock rate estimate wrt to the nominal tick */
  double P[2][2];       /**< State covariance matrix. */
} clock_est_state_t;

#ifdef __cplusplus
extern "C" {
#endif

void propagate_clock_state(clock_est_state_t *clock_state, u64 tc);
void update_clock_state(clock_est_state_t *clock_state,
                        const gnss_solution *sol);

#ifdef __cplusplus
}
#endif

#endif /* CLOCK_FILTER_H */
