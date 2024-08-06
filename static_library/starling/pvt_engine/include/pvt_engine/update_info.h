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

#ifndef LIBSWIFTNAV_PVT_ENGINE_UPDATE_INFO_H
#define LIBSWIFTNAV_PVT_ENGINE_UPDATE_INFO_H

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

struct UpdateInfo {
  optional<s32> index;
  optional<double> measurement;
  optional<double> measurement_variance;
  Gaussian residual;

  UpdateInfo() : index(), measurement(), measurement_variance(), residual() {}

  UpdateInfo(s32 idx, double meas, double meas_var, Gaussian res)
      : index(idx),
        measurement(meas),
        measurement_variance(meas_var),
        residual(res) {}
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_UPDATE_INFO_H
