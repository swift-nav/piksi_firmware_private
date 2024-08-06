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

#ifndef STARLING_VRS_UTILS_H
#define STARLING_VRS_UTILS_H

#include <pvt_common/eigen_custom.h>

namespace pvt_engine {
namespace vrs {

constexpr double DEFAULT_VRS_OFFSET_METERS = 100000.0;

Eigen::Vector3d offset_rover_latitude(const Eigen::Vector3d &rover_location,
                                      const double offset_meters);

Eigen::Vector3d reverse_offset_rover_latitude(
    const Eigen::Vector3d &rover_location, const double offset_meters);

}  // namespace vrs
}  // namespace pvt_engine

#endif  // STARLING_VRS_UTILS_H
