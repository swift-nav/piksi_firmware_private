/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_PVT_ENGINE_PROCESS_MODEL_IONOSPHERE_H
#define LIBSWIFTNAV_PVT_ENGINE_PROCESS_MODEL_IONOSPHERE_H

#include <pvt_engine/observation_model_ionosphere.h>
#include <pvt_engine/process_model_no_internal_state.h>

namespace pvt_engine {

class ProcessModelIonosphere : public ProcessModelNoInternalState {
 public:
  explicit ProcessModelIonosphere(
      const CommonData &common_data, ObservationModelIonosphere &obs_model_iono,
      const ProcessModelConfiguration &config,
      OBSERVATION_MODEL_POSITION_MODE position_model_mode,
      MODEL_TYPE model_type);
  ProcessModelIonosphere &operator=(const ProcessModelIonosphere & /*rhs*/) {
    return *this;
  }
  PRC initialize(const ProcessModelConfiguration &config,
                 OBSERVATION_MODEL_POSITION_MODE position_model_mode) override;

  // Returns the part of the F and Q matrix defined within this observation
  // model.
  PRC get_process_model(const double &delta_time, MatrixMaxStateDimd_t *F,
                        MatrixMaxStateDimd_t *Q,
                        DoF_container *dof_loss) const override;

 private:
  double get_dof_loss(const SatIdentifier &sat, const double &delta_time) const;
  double get_process_noise(const SatIdentifier &sat,
                           const double &delta_time) const;

  const CommonData &common_data_;

  // Mutable because the get_initial_variance function needs to modify it and
  // that should be const. This is only a private variable with very limited
  // use.
  mutable optional<double> baseline_length_km_;

  const ObservationModelIonosphere &obs_model_iono_;
};

}  // namespace pvt_engine
#endif  // LIBSWIFTNAV_PVT_ENGINE_PROCESS_MODEL_IONOSPHERE_H
