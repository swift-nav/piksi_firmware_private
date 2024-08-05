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

#ifndef LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_FUNCTION_H
#define LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_FUNCTION_H

#include <pvt_engine/eigen_types.h>
#include <pvt_engine/pvt_return_codes.h>

namespace pvt_engine {
/**
 * This class defines the (possibly) nonlinear function which maps states from
 * one observations model to observations. It's used to compute the innovations
 * in the Extended Kalman Filter update.
 */
class ObservationModelFunction {
 public:
  /*
   * The only function which is relevant to the user is the operator() which
   * acts like `h(x)` on the state x.
   */
  virtual double operator()(const VectorMaxStateDimd_t &state) const = 0;

  virtual ~ObservationModelFunction() = default;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_FUNCTION_H
