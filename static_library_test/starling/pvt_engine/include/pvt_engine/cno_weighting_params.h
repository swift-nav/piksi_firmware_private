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

#ifndef STARLING_CNO_WEIGHTING_PARAMS_H
#define STARLING_CNO_WEIGHTING_PARAMS_H

#include <pvt_common/containers/map.h>
#include <pvt_engine/pvt_types.h>

namespace pvt_engine {
struct CNoWeightingParams {
  double maximum_stddev = 0.0;
  double inflection_point_x_axis = 0.0;
  double rate_of_decrease = 0.0;
};

class CNoWeightCollection final {
 public:
  CNoWeightCollection();
  explicit CNoWeightCollection(
      const pvt_common::containers::Map<OBSERVATION_TYPE, CNoWeightingParams,
                                        MAX_OBSERVATION_TYPE> &weight_params);

  void set_params(const OBSERVATION_TYPE obs_type,
                  const CNoWeightingParams &params);
  optional<CNoWeightingParams> get_params(
      const OBSERVATION_TYPE obs_type) const;

 private:
  pvt_common::containers::Map<OBSERVATION_TYPE, CNoWeightingParams,
                              MAX_OBSERVATION_TYPE>
      weights_map_;
};

}  // namespace pvt_engine

#endif  // STARLING_CNO_WEIGHTING_PARAMS_H
