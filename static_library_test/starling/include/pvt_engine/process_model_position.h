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

#ifndef LIBSWIFTNAV_PVT_ENGINE_PROCESS_MODEL_POSITION_H
#define LIBSWIFTNAV_PVT_ENGINE_PROCESS_MODEL_POSITION_H

#include <pvt_engine/common_data.h>
#include <pvt_engine/observation_model_interface.h>
#include <pvt_engine/process_model_no_internal_state.h>

// Offset of the velocity term from the equivalent position term and accel from
// the velocity term
#define VEL_ACC_OFFSET 3

namespace pvt_engine {

class ProcessModelPosition : public ProcessModelNoInternalState {
 public:
  explicit ProcessModelPosition(
      const CommonData &common_data, ObservationModelInterface *obs_model,
      const ProcessModelConfiguration &config,
      OBSERVATION_MODEL_POSITION_MODE position_model_mode);

  ProcessModelPosition &operator=(const ProcessModelPosition &other) {
    config_ = other.config_;
    return *this;
  }

  // Initialization of the Observation model
  bool update_config(
      const ProcessModelConfiguration &config,
      OBSERVATION_MODEL_POSITION_MODE position_model_mode) override;

  // Returns the part of the F and Q matrix defined within this observation
  // model.
  PRC get_process_model(const double &delta_time, MatrixMaxStateDimd_t *F,
                        MatrixMaxStateDimd_t *Q,
                        DoF_container *dof_loss) const override;

 private:
  Eigen::Matrix3d get_process_noise() const;
  double get_dof_loss(const double &delta_time, const s32 &axis) const;
  const CommonData &common_data_;
  bool is_velocity_model_active() const;
  bool is_acceleration_model_active() const;
};

}  // namespace pvt_engine
#endif  // LIBSWIFTNAV_PVT_ENGINE_PROCESS_MODEL_POSITION_H
