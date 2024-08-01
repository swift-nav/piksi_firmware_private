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

#ifndef LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_FUNCTION_POSITION_H
#define LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_FUNCTION_POSITION_H

#include <swiftnav/constants.h>

#include <pvt_engine/common_data.h>
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/line_of_sight.h>
#include <pvt_engine/observation.h>
#include <pvt_engine/observation_model_function.h>
#include <pvt_engine/pvt_return_codes.h>

namespace pvt_engine {

class ObservationModelFunctionPosition : public ObservationModelFunction {
 public:
  ObservationModelFunctionPosition();

  void set_data(const Observation &obs,
                const VectorMaxBaselineStatesd_t &linearization_point);

 protected:
  Eigen::Vector3d sagnac_rotation(double tof,
                                  const Eigen::Vector3d &sat_pos) const;

  Observation obs_;
  VectorMaxBaselineStatesd_t linearization_point_;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_FUNCTION_POSITION_H
