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

#ifndef LIBSWIFTNAV_PVT_ENGINE_LINE_OF_SIGHT_H
#define LIBSWIFTNAV_PVT_ENGINE_LINE_OF_SIGHT_H

#include <pvt_engine/eigen_types.h>
#include <swiftnav/coord_system.h>
#include <swiftnav/gnss_time.h>

namespace pvt_engine {

Eigen::Vector3d compute_los_vector(const Eigen::Vector3d &sat_pos,
                                   const Eigen::Vector3d &station_position);

Eigen::Vector3d sagnac_rotation(double tof, const Eigen::Vector3d &sat_pos);

double compute_range(const Eigen::Vector3d &sat_pos,
                     const Eigen::Vector3d &station_location);

/*
 * Computes a line of sight vector (ECEF) from coordinates in WGS84,
 * the latitude (radians), longitude (radians), height (meters), and
 * angles relative to the WGS84 ellipsoid,
 * elevation (radians) and azimuth (radians).
 */
Eigen::Vector3d line_of_sight_from_wgs84_elevation_azimuth(
    const Eigen::Vector3d &receiver_llh, double elevation_radians,
    double azimuth_radians);

void wgs84_elevation_azimuth_from_line_of_sight(
    const Eigen::Vector3d &receiver_llh, const Eigen::Vector3d &line_of_sight,
    double *azimuth, double *elevation);

gps_time_t compute_time_of_transmission(
    const gps_time_t &time_of_reception, const Eigen::Vector3d &sat_pos,
    const double &sat_clk_meters, const Eigen::Vector3d &station_location);

bool above_horizon(const Eigen::Vector3d &sat_pos,
                   const Eigen::Vector3d &station_location,
                   const double elevation_mask_rad = 0.0);

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_LINE_OF_SIGHT_H
