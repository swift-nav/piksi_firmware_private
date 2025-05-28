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

#ifndef LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_PHASE_WAVELENGTH_DEPENDENT_H
#define LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_PHASE_WAVELENGTH_DEPENDENT_H

#include <pvt_engine/observation_model_interface.h>
#include <pvt_engine/pvt_types.h>
#include <swiftnav/signal.h>

namespace pvt_engine {

class ObservationModelPhaseWavelengthDependent
    : public ObservationModelInterface {
 public:
  PRC initialize(const ObservationModelConfiguration &config) override;

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

 protected:
  explicit ObservationModelPhaseWavelengthDependent(
      const ObservationModelConfiguration &config, MODEL_TYPE model_type);

 private:
  virtual PRC handle_LoL(const FilterObservationIdSet &LoLs_this_epoch,
                         const s32 state_offset,
                         EstimatorInterface *estimator) = 0;

  virtual double get_initial_state(const StateIdentifier &state_id) const = 0;

  optional<gps_time_t> last_processed_epoch;
};
}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_PHASE_WAVELENGTH_DEPENDENT_H
