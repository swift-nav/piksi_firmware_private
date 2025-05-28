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

#ifndef LIBSWIFTNAV_PVT_ENGINE_PROCESS_MODEL_DIAGONAL_H
#define LIBSWIFTNAV_PVT_ENGINE_PROCESS_MODEL_DIAGONAL_H

#include <pvt_engine/observation_model_interface.h>
#include <pvt_engine/process_model_no_internal_state.h>

namespace pvt_engine {

class ProcessModelDiagonal : public ProcessModelNoInternalState {
 public:
  explicit ProcessModelDiagonal(
      ObservationModelInterface *obs_model,
      const ProcessModelConfiguration &config,
      OBSERVATION_MODEL_POSITION_MODE position_model_mode,
      MODEL_TYPE model_type);

  // Returns the part of the F and Q matrix defined within this observation
  // model.
  PRC get_process_model(const double &delta_time, MatrixMaxStateDimd_t *F,
                        MatrixMaxStateDimd_t *Q,
                        DoF_container *dof_loss) const override;

 private:
  double get_dof_loss(const double &delta_time) const;
  double get_process_noise(const double &delta_time) const;
};

}  // namespace pvt_engine
#endif  // LIBSWIFTNAV_PVT_ENGINE_PROCESS_MODEL_DIAGONAL_H
