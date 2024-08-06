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

#ifndef LIBSWIFTNAV_PVT_ENGINE_OUTLIER_INFO_H
#define LIBSWIFTNAV_PVT_ENGINE_OUTLIER_INFO_H

#include <pvt_common/optional.h>

#include <swiftnav/common.h>
#include <swiftnav/logging.h>

#include <pvt_engine/covariance_ref.h>
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/filter_data_interface.h>
#include <pvt_engine/finity.h>
#include <pvt_engine/gaussian.h>
#include <pvt_engine/observation_model_function_row.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/pvt_types.h>
#include <pvt_engine/single_observation_filter_data.h>

namespace pvt_engine {

struct OutlierInfo {
  Gaussian residual;
  optional<double> ratio;
  optional<double> scaled_ratio;
  optional<double> variance_factor;

  OutlierInfo(Gaussian res, double rat, double vf)
      : residual(res),
        ratio(rat),
        scaled_ratio(rat / vf),
        variance_factor(vf) {}

  OutlierInfo() : residual(), ratio(), scaled_ratio(), variance_factor() {}
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_OUTLIER_INFO_H
