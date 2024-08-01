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

#ifndef STARLING_VALIDATE_H
#define STARLING_VALIDATE_H

#include <inttypes.h>
#include <cmath>

#include <pvt_common/covariance_checks.h>
#include <pvt_engine/finity.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/pvt_types.h>
#include <starling/logging.h>
#include <starling/observation.h>

#include <swiftnav/gnss_time.h>

#include <Eigen/Core>

namespace pvt_engine {

namespace validation {
PRC validate_inputs(const gps_time_t &obs_time, u32 num_obs);

PRC validate_inputs(
    const Eigen::Vector3d &reference_station_position,
    REFERENCE_STATION_POSITION_SOURCE reference_station_position_source);

PRC validate_outputs(const Eigen::Vector3d &baseline,
                     const Eigen::Matrix3d &baseline_covariance, bool is_fixed);

}  // namespace validation
}  // namespace pvt_engine

#endif  // STARLING_VALIDATE_H
