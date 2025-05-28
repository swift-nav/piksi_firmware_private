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

#ifndef LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_POSITION_H
#define LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_POSITION_H

#include <vector>

#include <pvt_common/containers/map.h>
#include <pvt_engine/common_data.h>
#include <pvt_engine/estimator_interface.h>
#include <pvt_engine/observation_model_interface.h>
#include <pvt_engine/pvt_types.h>
#include <swiftnav/macros.h>

namespace pvt_engine {

class ObservationModelPosition : public ObservationModelInterface {
 public:
  ObservationModelPosition &operator=(const ObservationModelPosition &other);

  using ObservationModelInterface::get_observation_model_vector;

  s32 get_max_size() const override;

  PRC initialize(const ObservationModelConfiguration &config) final;

  bool update_config(const ObservationModelConfiguration &config) override;

  // Returns the part of the H matrix defined within this observation model.
  // This will be the piece of 1 row of H defined by this observation model
  PRC get_observation_model_vector(
      const VectorMaxKinematicStatesd_t &linearization_point,
      const Observation &obs, VectorMaxStateDimd_t *obs_vector,
      ObservationModelFunction *obs_model_function,
      s32 model_offset) const override = 0;

  // Iterate through the sdiffs and adjust the estimator for any added or
  // deleted states
  PRC update_states(const FilterObservationHandler &observation_handler,
                    s32 state_offset, EstimatorInterface *estimator) final;

  // NOTE: this is linked to the below function - since the position type is
  // stored in the sat field, we just need to cast to get it out
  static POSITION_STATE_LABELS get_state_label(
      const StateIdentifier &state_id) {
    return static_cast<POSITION_STATE_LABELS>(state_id.sid.sat);
  }

  // Baseline model is an exception - each obs relates to all baseline states
  // NOTE: this is linked to the above function - store the position type that
  // this state refers to (an enum) in the sat field
  static StateIdentifier get_state_id(
      const POSITION_STATE_LABELS &state_label) {
    StateIdentifier key;
    if (state_label <= INVALID_POSITION_STATE ||
        state_label >= MAX_POSITION_STATE) {
      detailed_log_error("State label %i is invalid", state_label);
    }
    key.sid.sat = static_cast<u16>(state_label);
    key.sid.code = CODE_COUNT;
    key.obs_type = MAX_OBSERVATION_TYPE;
    return key;
  }

 protected:
  explicit ObservationModelPosition(
      const CommonData &common_data,
      const ObservationModelConfiguration &config,
      const VarianceHandlerConfiguration &variance_config,
      MODEL_TYPE model_type);

  virtual double get_initial_position(
      const POSITION_STATE_LABELS &label) const = 0;

  bool is_state_observed(
      const ObservationIdentifier &obs_id, const s32 state_offset,
      const EstimatorInterface &estimator) const override SWIFT_ATTR_NORETURN;

  // Baseline model is an exception - each obs relates to all baseline states
  StateIdentifier get_state_id(
      const ObservationIdentifier &obs_id SWIFT_ATTR_UNUSED) const final {
    log_error("ObservationModelPosition::get_key called.");
    assert(false);
    return StateIdentifier();
  }

  Eigen::Vector3d compute_baseline_vector(
      const Eigen::Vector3d &baseline_estimate) const;

  void init_inverse_var_sum(
      const VarianceHandlerConfiguration &variance_config) override;

  bool create_state(POSITION_STATE_LABELS label);

  bool is_position_model_active() const;

  bool is_average_velocity_model_active() const;

  bool is_instantaneous_velocity_model_active() const;

  bool is_acceleration_model_active() const;

  double get_initial_variance(const StateIdentifier &state_id) const final;

  double get_dof_loss(const StateIdentifier &state_id) const override;

  // Reference to common data structures
  const CommonData &common_data_;

  double inverse_var_sum;
};
}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_POSITION_H
