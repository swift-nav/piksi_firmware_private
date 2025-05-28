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

#ifndef LIBSWIFTNAV_PVT_ENGINE_SINGLE_OBSERVATION_FILTER_DATA_H
#define LIBSWIFTNAV_PVT_ENGINE_SINGLE_OBSERVATION_FILTER_DATA_H

#include <pvt_common/optional.h>
#include <pvt_engine/eigen_types.h>

#include <pvt_engine/observation_model_function_row.h>

namespace pvt_engine {

class SingleObsUntaggedFilterData {
 public:
  SingleObsUntaggedFilterData(
      const double &y, const VectorMaxStateDimd_t &H_row_transpose,
      const optional<const ObservationModelFunctionRow &> &h_func,
      const double &R);

  // give const access to the members
  const double &get_y() const { return y_; }
  const VectorMaxStateDimd_t &get_H_row_transpose() const {
    return H_row_transpose_;
  }
  const optional<const ObservationModelFunctionRow &> get_H_func() const;
  const double &get_R() const { return R_; }

 private:
  double y_;
  VectorMaxStateDimd_t H_row_transpose_;
  optional<const ObservationModelFunctionRow &> h_func_;
  double R_;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_SINGLE_OBSERVATION_FILTER_DATA_H
