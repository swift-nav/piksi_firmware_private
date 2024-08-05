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

#ifndef LIBSWIFTNAV_PVT_ENGINE_PROPAGATE_H
#define LIBSWIFTNAV_PVT_ENGINE_PROPAGATE_H

#ifdef __cplusplus
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/gnss_constants.h>
#endif

#ifdef __cplusplus
namespace pvt_engine {

namespace propagate {

/**
 * Compute a nominal doppler measurement which includes the geometry term and
 * satellite clock term.
 * @param sat_vel Vector of satellite velocity
 * @param sat_pos Vector of satellite position
 * @param receiver_pos Vector of receiver position
 * @param sat_clock_error_rate Rate of change of satellite clock error
 * @return computed nominal doppler value
 */
double nominal_doppler(const Eigen::Vector3d &sat_vel,
                       const Eigen::Vector3d &sat_pos,
                       const Eigen::Vector3d &receiver_pos,
                       const double sat_clock_error_rate);

/**
 * Compute a nominal distance (pseudorange) measurement which includes the
 * geometry term and satellite clock term.
 * @param sat_pos Vector of satellite position
 * @param receiver_pos Vector of receiver position
 * @param sat_clock_error Satellite clock error
 * @return computed nominal pseudorange value
 */
double nominal_pseudorange(const Eigen::Vector3d sat_pos,
                           const Eigen::Vector3d &receiver_pos,
                           const double sat_clock_error);

}  // namespace propagate

}  // namespace pvt_engine

extern "C" {
#endif  // __cplusplus

double nominal_doppler(const double sat_vel[3], const double sat_pos[3],
                       const double receiver_pos[3],
                       const double sat_clock_error_rate);

double nominal_pseudorange(const double sat_pos[3],
                           const double receiver_pos[3],
                           const double sat_clock_error);

#ifdef __cplusplus
}  // extern "C"
#endif  // __cplusplus

#endif  // LIBSWIFTNAV_PVT_ENGINE_PROPAGATE_H
