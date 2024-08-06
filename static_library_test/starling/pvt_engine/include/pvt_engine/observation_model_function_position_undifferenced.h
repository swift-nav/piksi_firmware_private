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

#ifndef LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_FUNCTION_POSITION_UNDIFFERENCED_H
#define LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_FUNCTION_POSITION_UNDIFFERENCED_H

#include <pvt_engine/line_of_sight.h>
#include <pvt_engine/observation_model_function_position.h>

namespace pvt_engine {

class ObservationModelFunctionPositionUndifferenced
    : public ObservationModelFunctionPosition {
 public:
  ObservationModelFunctionPositionUndifferenced();

  double operator()(const VectorMaxStateDimd_t &state) const override;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_FUNCTION_POSITION_UNDIFFERENCED_H
