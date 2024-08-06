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

#ifndef LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_AMBIGUITIES_H
#define LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_AMBIGUITIES_H

#include <pvt_engine/ambiguity_map.h>
#include <pvt_engine/observation_model_phase_wavelength_dependent.h>

namespace pvt_engine {

class ObservationModelAmbiguities
    : public ObservationModelPhaseWavelengthDependent {
 public:
  explicit ObservationModelAmbiguities(
      const ObservationModelConfiguration &config,
      const VarianceHandlerConfiguration &variance_config,
      MODEL_TYPE model_type = AMB_MODEL);

  PRC initialize(const ObservationModelConfiguration &config) override;

  bool update_config(const ObservationModelConfiguration &config) override;

  s32 get_max_size() const override;

  PRC get_ambiguity_indices(
      ambiguities::AmbiguityIndexMap *amb_index_map) const;

  PRC reinitialize_signals(const FilterObservationIdSet &obs_ids,
                           const s32 state_offset,
                           EstimatorInterface *estimator);

 private:
  PRC handle_LoL(const FilterObservationIdSet &LoLs_this_epoch,
                 const s32 state_offset, EstimatorInterface *estimator) final;

  void init_inverse_var_sum(
      const VarianceHandlerConfiguration &variance_config) override;

  StateIdentifier get_state_id(
      const ObservationIdentifier &obs_id) const override;

  double get_initial_state(const StateIdentifier &state_id) const override;

  double get_initial_variance(const StateIdentifier &state_id) const final;

  double get_dof_loss(const StateIdentifier &state_id) const override;

  double inverse_var_sum;

  ambiguities::AmbiguityMap constrained_ambs_map;
};
}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_AMBIGUITIES_H
