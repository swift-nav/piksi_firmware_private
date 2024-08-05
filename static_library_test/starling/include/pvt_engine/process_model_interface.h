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

#ifndef LIBSWIFTNAV_PVT_ENGINE_PROCESS_MODEL_INTERFACE_H
#define LIBSWIFTNAV_PVT_ENGINE_PROCESS_MODEL_INTERFACE_H

#include <starling/observation.h>

#include <pvt_engine/eigen_types.h>
#include <pvt_engine/observation_model_interface.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/pvt_types.h>

namespace pvt_engine {

/** \defgroup process_model_interface ProcessModelInterface
 * Defines the abstract interface for a process model. Each Observation model
 * should have a corresponding
 * Process model that describes the transition of the state between epochs
 * \{ */

class ProcessModelInterface {
 public:
  explicit ProcessModelInterface(
      const ProcessModelConfiguration &config,
      OBSERVATION_MODEL_POSITION_MODE position_model_mode,
      MODEL_TYPE model_type, ObservationModelInterface *obs_model)
      : config_(config),
        position_model_mode_(position_model_mode),
        model_type_(model_type),
        observation_model_(*obs_model) {}

  ProcessModelInterface &operator=(const ProcessModelInterface &other) {
    config_ = other.config_;
    position_model_mode_ = other.position_model_mode_;
    model_type_ = other.model_type_;
    return *this;
  }

  virtual ~ProcessModelInterface() = default;

  virtual PRC initialize(
      const ProcessModelConfiguration &config,
      OBSERVATION_MODEL_POSITION_MODE position_model_mode) = 0;

  virtual bool update_config(
      const ProcessModelConfiguration &config,
      OBSERVATION_MODEL_POSITION_MODE position_model_mode) {
    config_ = config;
    position_model_mode_ = position_model_mode;
    return false;
  }

  virtual PRC get_process_model(const double &delta_time,
                                MatrixMaxStateDimd_t *F,
                                MatrixMaxStateDimd_t *Q,
                                DoF_container *dof_loss) const = 0;

 protected:
  void init_F_and_Q_blocks(MatrixMaxStateDimd_t *F,
                           MatrixMaxStateDimd_t *Q) const {
    assert(F);
    assert(Q);
    F->resize(observation_model_.get_number_of_states(),
              observation_model_.get_number_of_states());
    F->setIdentity();
    Q->resize(observation_model_.get_number_of_states(),
              observation_model_.get_number_of_states());
    Q->setZero();
  }

  ProcessModelConfiguration config_;
  OBSERVATION_MODEL_POSITION_MODE position_model_mode_;
  MODEL_TYPE model_type_;

  // Reference to the total number of states associated with the
  // process/observation model pair in the filter
  // This is shared across process and observation models.
  const ObservationModelInterface &observation_model_;
};
// \}
}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_PROCESS_MODEL_INTERFACE_H
