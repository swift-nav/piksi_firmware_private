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

#ifndef LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_TROPOSPHERE_H
#define LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_TROPOSPHERE_H

#include <pvt_engine/common_data.h>
#include <pvt_engine/estimator_interface.h>
#include <pvt_engine/observation_model_interface.h>
#include <pvt_engine/pvt_types.h>
#include <pvt_engine/troposphere_model_gpt2.h>
#include <swiftnav/macros.h>

namespace pvt_engine {

class ObservationModelTroposphere : public ObservationModelInterface {
 public:
  explicit ObservationModelTroposphere(
      const ObservationModelConfiguration &config,
      MODEL_TYPE model_type = TROPOSPHERE_MODEL);

  s32 get_max_size() const override;

  PRC initialize(const ObservationModelConfiguration &config) final;

  // Returns the part of the H matrix defined within this observation model.
  // This will be the piece of 1 row of H defined by this observation model
  PRC get_observation_model_vector(
      const VectorMaxKinematicStatesd_t &linearization_point,
      const Observation &obs, VectorMaxStateDimd_t *obs_vector,
      ObservationModelFunction *obs_model_function,
      s32 model_offset) const override;

  using ObservationModelInterface::get_observation_model_vector;

  // Iterate through the sdiffs and adjust the estimator for any added or
  // deleted states
  PRC update_states(const FilterObservationHandler &observation_handler,
                    s32 state_offset, EstimatorInterface *estimator) override;

  bool is_state_observed(const ObservationIdentifier &obs_id,
                         const s32 state_offset,
                         const EstimatorInterface &estimator) const override;

  double get_initial_variance(const StateIdentifier &state_id) const final;

 private:
  // Not necessary as all obs have the same tropo parameter
  StateIdentifier get_state_id(
      const ObservationIdentifier &obs_id SWIFT_ATTR_UNUSED) const override {
    log_error("ObservationModelTroposphere::get_key called.");
    assert(false);
    return StateIdentifier();
  }

  StateIdentifier get_state_id() const {
    StateIdentifier state_id;
    state_id.sid.code = CODE_COUNT;
    return state_id;
  }

  void init_inverse_var_sum(
      const VarianceHandlerConfiguration &variance_config) override;

  double get_dof_loss(const StateIdentifier &state_id) const override;

  TroposphereModelGPT2 troposphere_model_;

  double inverse_var_sum;
};
}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_TROPOSPHERE_H
