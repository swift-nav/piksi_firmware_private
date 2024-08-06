/*
 * Copyright (C) 2021 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_OBSERVATION_MODEL_HARDWARE_BIASES_ON_PSEUDORANGE_ONLY_H
#define LIBSWIFTNAV_OBSERVATION_MODEL_HARDWARE_BIASES_ON_PSEUDORANGE_ONLY_H

#include <pvt_engine/common_data.h>
#include <pvt_engine/estimator_interface.h>
#include <pvt_engine/frequency_manager.h>
#include <pvt_engine/observation_model_interface.h>
#include <pvt_engine/pvt_types.h>

namespace pvt_engine {

class ObservationModelHardwareBiasesOnPseudorangeOnly
    : public ObservationModelDecoupledClock {
 public:
  explicit ObservationModelHardwareBiasesOnPseudorangeOnly(
      const ObservationModelConfiguration &config,
      const VarianceHandlerConfiguration &variance_config,
      MODEL_TYPE model_type = HARDWARE_BIASES_ON_PSEUDORANGE_ONLY_MODEL);

  s32 get_max_size() const override;

  using ObservationModelInterface::get_observation_model_vector;

 private:
  StateIdentifier get_state_id(
      const ObservationIdentifier &obs_id) const override;

  void init_inverse_var_sum(
      const VarianceHandlerConfiguration &variance_config) override;

  double get_initial_variance(const StateIdentifier &state_id) const final;

  double get_dof_loss(const StateIdentifier &state_id) const override;
};
// \}
}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_OBSERVATION_MODEL_HARDWARE_BIASES_ON_PSEUDORANGE_ONLY_H
