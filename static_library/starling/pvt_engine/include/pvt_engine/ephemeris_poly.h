/*
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_PVT_ENGINE_EPHEMERIS_POLY_H
#define STARLING_PVT_ENGINE_EPHEMERIS_POLY_H

#include <swiftnav/ephemeris.h>
#include <swiftnav/gnss_time.h>
#include "pvt_common/optional.h"
#include "pvt_engine/configuration.h"
#include "pvt_engine/eigen_types.h"
#include "pvt_engine/satellite_pva.h"

namespace pvt_engine {

class EphemerisPoly {
 public:
  EphemerisPoly(const ephemeris_t &eph, const gps_time_t &t,
                const EphemerisConfiguration &config);
  EphemerisPoly(const ephemeris_t &eph, const gps_time_t &t_start,
                const gps_time_t &t_end, const EphemerisConfiguration &config);

  optional<SatPVA> get_sat_pva(const gps_time_t &t) const;
  bool valid_time(const gps_time_t &t) const;
  EphemerisKey get_eph_key() const;

  // degree of interpolating polynomial (only 3=Cubic implemented so far)
  static constexpr int POLY_DEGREE = 3;
  // minimum allowed time difference to ensure numerical stability
  static constexpr double POLY_MIN_INTERVAL_S = 0.125;

 private:
  // polynomial coefficients for satellite position
  Eigen::Matrix<double, 3, POLY_DEGREE + 1> pos_coeff_;
  // polynomial coefficients for satellite clock
  Eigen::Matrix<double, 1, POLY_DEGREE + 1> clk_coeff_;
  // start and end times of the interval
  gps_time_t t_start_;
  gps_time_t t_end_;
  // ephemeris key
  EphemerisKey eph_key_;

  gps_time_t make_start_time(const ephemeris_t &eph, const gps_time_t &t_ref,
                             const EphemerisConfiguration &config) const;
  gps_time_t make_end_time(const ephemeris_t &eph, const gps_time_t &t_ref,
                           const EphemerisConfiguration &config) const;

  void compute_polynomials(const Eigen::Vector3d &pos1,
                           const Eigen::Vector3d &vel1, const double clk1,
                           const double clk_rate1, const Eigen::Vector3d &pos2,
                           const Eigen::Vector3d &vel2, const double clk2,
                           const double clk_rate2);
};

}  // namespace pvt_engine

#endif  // STARLING_PVT_ENGINE_EPHEMERIS_POLY_H
