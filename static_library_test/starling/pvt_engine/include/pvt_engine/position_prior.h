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

#ifndef PVT_ENGINE_POSITION_PRIOR_H
#define PVT_ENGINE_POSITION_PRIOR_H

#include <pvt_common/eigen_custom.h>

namespace pvt_engine {

struct PositionPrior {
  gps_time_t time;
  Eigen::Vector3d position;
  Eigen::Matrix3d covariance;

  PositionPrior()
      : time(GPS_TIME_UNKNOWN),
        position(Eigen::Vector3d::Zero()),
        covariance(Eigen::Matrix3d::Zero()) {}
  PositionPrior(const gps_time_t &t, const Eigen::Vector3d &p,
                const Eigen::Matrix3d &c)
      : time(t), position(p), covariance(c) {}
};

}  // namespace pvt_engine

#endif  // PVT_ENGINE_POSITION_PRIOR_H
