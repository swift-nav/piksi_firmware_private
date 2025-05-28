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

#ifndef LIBSWIFTNAV_PVT_ENGINE_AZIMUTH_ELEVATION_H
#define LIBSWIFTNAV_PVT_ENGINE_AZIMUTH_ELEVATION_H

#include "pvt_engine/pvt_types.h"

namespace pvt_engine {

class AzimuthElevation {
 public:
  enum ANGLE_UNIT { DEGREES, RADIANS };
  explicit AzimuthElevation(const double &azimuth = 0.,
                            const double &elevation = 0.,
                            const ANGLE_UNIT &units = RADIANS);
  double get_azimuth_rad() const;
  double get_elevation_rad() const;
  double get_azimuth_deg() const;
  double get_elevation_deg() const;

  void set_azimuth_rad(const double azimuth) { azimuth_rad_ = azimuth; };
  void set_elevation_rad(const double elevation) {
    elevation_rad_ = elevation;
  };

 private:
  double azimuth_rad_;
  double elevation_rad_;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_AZIMUTH_ELEVATION_H
