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

#ifndef LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_PHASE_WINDUP_H
#define LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_PHASE_WINDUP_H

#include <pvt_engine/observation_model_phase_wavelength_dependent.h>

namespace pvt_engine {

class ObservationModelPhaseWindup
    : public ObservationModelPhaseWavelengthDependent {
 public:
  explicit ObservationModelPhaseWindup(
      const ObservationModelConfiguration &config,
      const VarianceHandlerConfiguration &variance_config,
      MODEL_TYPE model_type = PHASE_WINDUP_MODEL);

  s32 get_max_size() const override;

 private:
  PRC handle_LoL(const FilterObservationIdSet &LoLs_this_epoch,
                 const s32 state_offset, EstimatorInterface *estimator) final;

  StateIdentifier get_state_id(
      const ObservationIdentifier &obs_id) const override;

  void init_inverse_var_sum(
      const VarianceHandlerConfiguration &variance_config) override;

  double get_initial_state(const StateIdentifier &state_id) const override;

  double get_initial_variance(const StateIdentifier &state_id) const final;

  double get_dof_loss(const StateIdentifier &state_id) const override;

  double inverse_var_sum;
};
}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_PHASE_WINDUP_H
