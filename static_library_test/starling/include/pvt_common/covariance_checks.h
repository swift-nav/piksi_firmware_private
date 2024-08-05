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

#ifndef PVT_COMMON_COVARIANCE_CHECKS_H
#define PVT_COMMON_COVARIANCE_CHECKS_H

#include "eigen_custom.h"

namespace pvt_common {

bool is_position_variance_acceptable(
    const Eigen::Vector3d &baseline,
    const Eigen::Matrix3d &baseline_covariance);
bool is_velocity_variance_acceptable(
    const Eigen::Vector3d &baseline,
    const Eigen::Matrix3d &velocity_covariance);

void covariance_to_accuracy(const Eigen::Matrix3d &covariance_ecef,
                            const Eigen::Vector3d &ref_ecef, double *accuracy,
                            double *h_accuracy, double *v_accuracy,
                            double *ecef_cov, double *ned_cov);

}  // namespace pvt_common

#endif  // PVT_COMMON_COVARIANCE_CHECKS_H
