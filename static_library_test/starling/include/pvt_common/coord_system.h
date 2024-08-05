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
#ifndef PVT_COMMON_COORD_SYSTEM_H
#define PVT_COMMON_COORD_SYSTEM_H

#include <swiftnav/constants.h>
#include <swiftnav/coord_system.h>
#include "eigen_custom.h"

namespace pvt_common {

/*
 * Contains simple convenience functions which deal with packing and
 * unpacking data from Eigen vectors when converting to and from LLH
 * and ECEF.
 */
Eigen::Vector3d wgsllh2ecef(const Eigen::Vector3d &receiver_llh);

Eigen::Vector3d wgsecef2llh(const Eigen::Vector3d &receiver_ecef);

/*
 * Note that this uses the latitude, longitude and height to create
 * a matrix that transforms a vector for north east down to x, y, z.
 */
Eigen::Matrix3d wgs_ecef2ned_matrix(const Eigen::Vector3d &llh);
Eigen::Matrix3d ned2ecef_covariance_transformation(
    const Eigen::Matrix3d &ned, const Eigen::Vector3d &pos_ecef);

}  // namespace pvt_common

#endif
