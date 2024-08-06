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

#include <swiftnav/gnss_time.h>
#include <array>
#include "eigen_custom.h"

namespace pvt_common {

bool is_position_variance_acceptable(
    const gps_time_t &epoch_time, const Eigen::Matrix3d &position_covariance);

bool is_velocity_variance_acceptable(
    const gps_time_t &epoch_time, const Eigen::Matrix3d &velocity_covariance);

void covariance_to_array(const Eigen::Matrix3d &covariance,
                         std::array<double, 6> *cov);

void covariance_to_ecef_accuracy(const Eigen::Matrix3d &covariance_ecef,
                                 double *accuracy,
                                 std::array<double, 6> *ecef_cov);

void covariance_to_accuracy(const Eigen::Matrix3d &covariance_ecef,
                            const Eigen::Vector3d &ref_ecef, double *accuracy,
                            double *h_accuracy, double *v_accuracy,
                            std::array<double, 6> *ecef_cov,
                            std::array<double, 6> *ned_cov);

void covariance_ecef_to_3d_accuracy(const Eigen::Matrix3d &covariance_ecef,
                                    double *accuracy);

void covariance_ned_to_hv_accuracy(const Eigen::Matrix3d &covariance_ned,
                                   double *h_accuracy, double *v_accuracy);

void covariance_ned_to_hv_accuracy_at_confidence_level(
    const Eigen::Matrix3d &covariance_ned, const int confidence,
    double *h_accuracy, double *v_accuracy, double *ell_semi_major,
    double *ell_semi_minor, double *ell_orientation);

void covariance_ned_to_ca_accuracy_at_confidence_level(
    const Eigen::Matrix3d &covariance_ned, const int confidence,
    const double heading, double *ct_accuracy, double *at_accuracy);

void compute_2d_error_ellipse(const Eigen::Matrix2d &covariance_ne,
                              double *ell_semi_major, double *ell_semi_minor,
                              double *ell_orientation);

}  // namespace pvt_common

#endif  // PVT_COMMON_COVARIANCE_CHECKS_H
