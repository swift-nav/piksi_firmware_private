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

#ifndef LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_FUNCTION_LINEAR_H
#define LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_FUNCTION_LINEAR_H

#include <pvt_engine/common_data.h>
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/observation.h>
#include <pvt_engine/observation_model_function.h>
#include <pvt_engine/pvt_return_codes.h>

namespace pvt_engine {

class LinearObservationModelFunction : public ObservationModelFunction {
 public:
  explicit LinearObservationModelFunction(VectorMaxStateDimd_t &h_row);
  LinearObservationModelFunction(const LinearObservationModelFunction &other) =
      delete;

  LinearObservationModelFunction &operator=(
      const LinearObservationModelFunction &other);

  double operator()(const VectorMaxStateDimd_t &state) const override;

  void set_data(const VectorMaxStateDimd_t &h_row, s32 model_offset);

 private:
  VectorMaxStateDimd_t &h_row_;
  int h_row_offset_;
  int h_row_size_;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_FUNCTION_LINEAR_H
