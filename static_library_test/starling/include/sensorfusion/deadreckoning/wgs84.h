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

#ifndef SENSORFUSION_DEADRECKONING_WGS84_H
#define SENSORFUSION_DEADRECKONING_WGS84_H

#include "pvt_common/eigen_custom.h"

//------------------------------------------------------------------------------
// About:
//
// Constants and functions related to the ellipsoidal model of the earth
//------------------------------------------------------------------------------

namespace sensorfusion {
namespace deadreckoning {
namespace wgs84 {

/// Ellipsoid flattening
constexpr double f = 1. / 298.257223563;

/// Ellipsoid semi-major axis
constexpr double a = 6378137.0;

/// Ellipsoid eccentricity
constexpr double e2 = 2.0 * f - f * f;

/// Ellipsoid semi-minor axis
constexpr double b = 6.356752314245179e+06;  // a*std::sqrt(1.0-e2);

/// Earth standard gravitational parameter
constexpr double kM = 3.986004418e14;

/// Earth rotation rate
constexpr double omega_e = 7.292115e-5;

/// J2 constant of geopotential model
constexpr double J2 = 0.001082629821313;

/// J4 constant of geopotential model
constexpr double J4 = 3.0 / 35.0 * e2 * e2 - 6.0 / 7.0 * J2 * e2;

/// \brief Function which implements the calculation of the gravity vector in
/// ECEF according to a spherical harmonic model truncated after the J4-term.
/// (https://en.wikipedia.org/wiki/Geopotential_model#The_deviations_of_Earth's_gravitational_field_from_that_of_a_homogeneous_sphere)
Eigen::Vector3d get_gravity_J4(const Eigen::Vector3d &pos_ecef);

/// \brief Function which calculates the gravity jacobian w.r.t. position
/// analytically
/// \returns the jacobian of the J4 gravity at the given position
Eigen::Matrix3d get_gravity_jacobian(const Eigen::Vector3d &pos_ecef);
}  // namespace wgs84
}  // namespace deadreckoning
}  // namespace sensorfusion

#endif  // SENSORFUSION_DEADRECKONING_WGS84_H
