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

#ifndef LIBSWIFTNAV_PVT_ENGINE_INTERNAL_SBAS_IONO_PIERCE_POINT_H
#define LIBSWIFTNAV_PVT_ENGINE_INTERNAL_SBAS_IONO_PIERCE_POINT_H

#include <math.h>
#include <swiftnav/constants.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/signal.h>

#include <pvt_common/containers/map.h>
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/pvt_types.h>
#include <pvt_engine/sbas/internal/sbas_common.h>
#include <pvt_engine/sbas/sbas.h>

#include <pvt_common/optional.h>

namespace pvt_engine {

class PiercePoint {
 public:
  void calculate(const Eigen::Vector3d &user_pos_llh,
                 const AzimuthElevation &sat_az_el);
  optional<double> calculate_slant_delay(const IonoGrid &iono_grid,
                                         const gps_time_t &epoch_time,
                                         const double &sat_elevation_rad);
  PiercePointBand which_band() const;

  double get_latitude() const { return lat_deg; }
  double get_longitude() const { return lon_deg; }

  void set_latitude(double latitude_deg) {
    assert(-90 <= latitude_deg);
    assert(latitude_deg <= 90);
    lat_deg = latitude_deg;
  }
  void set_longitude(double longitude_deg) {
    assert(-180 <= longitude_deg);
    assert(longitude_deg < 180);
    lon_deg = longitude_deg;
  }

 private:
  void sanitize_calculate_params(const Eigen::Vector3d &user_pos_llh,
                                 const AzimuthElevation &sat_az_el) const;

  double lat_deg;
  double lon_deg;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_INTERNAL_SBAS_IONO_PIERCE_POINT_H
