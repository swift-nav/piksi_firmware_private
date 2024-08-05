/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <starling/cycle_slip.h>
#include <swiftnav/common.h>
#include <swiftnav/linear_algebra.h>

#include <assert.h>

/**
 * Check if there was possible cycle slip from two consecutive lock times.
 *
 * Note: Implements algorithm from RTCM 3.2 page 207.
 *
 * \param p Minimum possible lock time at previous epoch [s]
 * \param n Minimum possible lock time at current epoch [s]
 * \param dt time increment from previous to current epoch [s]
 * \return If true there was a possible cycle slip
 *
 */
bool was_cycle_slip(double p, double n, double dt) {
  assert(dt > FLOAT_EQUALITY_EPS);

  if (p > n) {
    return true;
  }
  if (double_approx_eq(p, n) && (dt >= p)) {
    return true;
  }
  if (double_approx_eq(p, n) && (dt < p)) {
    return false;
  }
  if ((p < n) && (dt >= (2 * n - p))) {
    return true;
  }
  if ((p < n) && (n < dt) && (dt < (2 * n - p))) {
    return true;
  }
  if ((p < n) && (dt <= n)) {
    return false;
  }
  assert(!"was_cycle_slip: should not get here");
}
