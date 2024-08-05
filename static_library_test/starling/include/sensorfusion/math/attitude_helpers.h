///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2019 Swift Navigation Inc.
// Contact: Swift Navigation <dev@swiftnav.com>
//
// This source is subject to the license found in the file 'LICENSE' which must
// be distributed together with this source. All other rights reserved.
//
// THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
// EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
///////////////////////////////////////////////////////////////////////////////

#ifndef STARLING_MATH_HELPERS_H
#define STARLING_MATH_HELPERS_H

#include "numeric_helpers.h"
#include "pvt_common/eigen_custom.h"
#include "sensorfusion/core/error_types.h"
#include "swiftnav/common.h"
#include "swiftnav/constants.h"

#define ERROR_INVALID_ROTATION_MATRIX "ERROR INVALID ROTATION MATRIX"

namespace sensorfusion {
namespace math {
namespace attitude {

inline double signed_angle_difference_degrees(double a1, double a2) {
  double delta = a1 - a2;
  while (delta > 180.) {
    delta -= 360.;
  }
  while (delta < -180.) {
    delta += 360.;
  }
  return delta;
}

inline double signed_angle_difference_radians(double a1, double a2) {
  return D2R * signed_angle_difference_degrees(R2D * a1, R2D * a2);
}

Eigen::Vector3d unsafe_rotation_matrix_to_euler_zyx(
    const Eigen::Matrix3d &rot_matrix);

ValueOrError<Eigen::Vector3d> passive_rotation_matrix_to_euler_zyx(
    const Eigen::Matrix3d &ctm_matrix);

ValueOrError<Eigen::Vector3d> active_rotation_matrix_to_euler_zyx(
    const Eigen::Matrix3d &active_rot_matrix);

template <typename T, s32 N>
inline bool is_valid_rotation_matrix(const Eigen::Matrix<T, N, N> &a_matrix) {
  // Check singularity & norm
  return sensorfusion::math::is_within_floating_error_tolerance(
      (a_matrix.transpose() * a_matrix).norm(), 1.73205080756887729352, 100);
}

template <typename T>
inline bool is_unit_quaternion(const Eigen::Quaternion<T> &a_quaternion) {
  // Check quaternion unity
  return sensorfusion::math::is_within_floating_error_tolerance(
      std::abs(a_quaternion.norm()), 1.0, 100);
}

// Note: Considering that we use the conjugation v |--> qvq^-1 to rotate a
// vector v a quaternion-based rotation matrix is typically a left-multiplier,
// i.e. a CTM (that is, a passive rotation) in our common usage.
ValueOrError<Eigen::Matrix3d> quat_to_active_rotation_matrix(
    const Eigen::Quaternion<double> &a_quaternion);

ValueOrError<Eigen::Matrix3d> quat_to_passive_rotation_matrix(
    const Eigen::Quaternion<double> &a_quaternion);

ValueOrError<Eigen::Quaternion<double>> active_rotation_matrix_to_quat(
    const Eigen::Matrix3d &a_rotation_matrix);

ValueOrError<Eigen::Vector3d> quat_to_euler_zyx(
    const Eigen::Quaternion<double> &a_quaternion);

ValueOrError<Eigen::Quaternion<double>> safe_unit_quat_multiplicative_inverse(
    const Eigen::Quaternion<double> &a_unit_quaternion);

ValueOrError<double> safe_asin(const double &x);

ValueOrError<Eigen::Quaternion<double>> safe_unit_quat_multiplicative_inverse(
    const Eigen::Quaternion<double> &a_unit_quaternion);

Eigen::Quaternion<double> unsafe_quat_multiplicative_inverse(
    const Eigen::Quaternion<double> &a_quaternion);

Eigen::Matrix3d euler_zyx_to_rotation_matrix(const Eigen::Vector3d &zyx);

// Create cross-product matrix "Vx" out of a vector "v" such that
// "Vx.dot(w)" is equivalent to "v.cross(w)"
template <class T>
inline Eigen::Matrix3d cross_matrix(T &&v) {
  return (Eigen::Matrix3d() << 0., -v(2), v(1), v(2), 0., -v(0), -v(1), v(0),
          0.)
      .finished();
}

}  // namespace attitude
}  // namespace math
}  // namespace sensorfusion

#endif  // STARLING_MATH_HELPERS_H
