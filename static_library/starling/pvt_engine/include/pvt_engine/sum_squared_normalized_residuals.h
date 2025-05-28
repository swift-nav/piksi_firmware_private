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

#ifndef LIBSWIFTNAV_PVT_ENGINE_SUM_SQUARED_NORMALIZED_RESIDUALS_H
#define LIBSWIFTNAV_PVT_ENGINE_SUM_SQUARED_NORMALIZED_RESIDUALS_H

#include <pvt_common/containers/map.h>
#include <pvt_engine/pvt_types.h>

namespace pvt_engine {

class SumSquaredNormalizedResiduals {
 public:
  SumSquaredNormalizedResiduals();
  SumSquaredNormalizedResiduals &operator+=(
      const SumSquaredNormalizedResiduals &rhs);
  void add(const constellation_t &constel, const OBSERVATION_TYPE &obs_type,
           const double &additional_sum_squared_normalized_residuals);
  void clear();
  double get_code_sum_squared_normalized_residuals(
      const pvt_common::containers::Set<constellation_t, CONSTELLATION_COUNT>
          &constellations) const;

  double get_phase_sum_squared_normalized_residuals(
      const pvt_common::containers::Set<constellation_t, CONSTELLATION_COUNT>
          &constellations) const;

 private:
  pvt_common::containers::Map<constellation_t, double, CONSTELLATION_COUNT>
      code_sum_squared_normalized_residuals;
  pvt_common::containers::Map<constellation_t, double, CONSTELLATION_COUNT>
      phase_sum_squared_normalized_residuals;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_SUM_SQUARED_NORMALIZED_RESIDUALS_H
