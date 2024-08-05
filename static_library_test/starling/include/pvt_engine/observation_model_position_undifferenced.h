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

#ifndef LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_POSITION_UNDIFFERENCED_H
#define LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_POSITION_UNDIFFERENCED_H

#include <pvt_engine/line_of_sight.h>
#include <pvt_engine/observation_model_position.h>

namespace pvt_engine {

class ObservationModelPositionUndifferenced : public ObservationModelPosition {
 public:
  explicit ObservationModelPositionUndifferenced(
      const CommonData &common_data,
      const ObservationModelConfiguration &config,
      const VarianceHandlerConfiguration &variance_config);

  // Returns the part of the H matrix defined within this observation model.
  // This will be the piece of 1 row of H defined by this observation model
  PRC get_observation_model_vector(
      const Eigen::Vector3d &linearization_point, const Observation &obs,
      VectorMaxStateDimd_t *obs_vector,
      ObservationModelFunction *obs_model_function) const final;

  using ObservationModelInterface::get_observation_model_vector;

 private:
  Eigen::Vector3d get_initial_position() const override;
};
}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_POSITION_UNDIFFERENCED_H
