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

#ifndef LIBSWIFTNAV_PVT_ENGINE_FILTER_DATA_INTERFACE_H
#define LIBSWIFTNAV_PVT_ENGINE_FILTER_DATA_INTERFACE_H

#include <pvt_common/optional.h>
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/single_observation_filter_data.h>
#include <pvt_engine/sum_squared_normalized_residuals.h>

namespace pvt_engine {

class FilterDataInterface {
 public:
  FilterDataInterface() = default;
  virtual ~FilterDataInterface() = default;

  // give const access to the members
  virtual const VectorMaxObsd_t &get_y() const = 0;
  virtual const MatrixMaxObsByStatesd_t &get_H() const = 0;
  virtual const VectorMaxObsd_t &get_R() const = 0;

  virtual const optional<const ObservationModelFunctionRow &> get_h_func(
      s32 idx) const = 0;
  // give const access to the members
  virtual SingleObsUntaggedFilterData get_single_obs(
      const s32 &index) const = 0;
  virtual s32 get_num_obs() const = 0;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_FILTER_DATA_INTERFACE_H
