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

#ifndef STARLING_PVT_ENGINE_SATELLITE_PVA_H
#define STARLING_PVT_ENGINE_SATELLITE_PVA_H

#include <pvt_engine/eigen_types.h>

namespace pvt_engine {

using EphemerisKey = u16;

class SatPVA {
 public:
  SatPVA(const double pos[], const double vel[], const double acc[],
         const double clk, const double clk_rate, const EphemerisKey eph_key);

  SatPVA(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel,
         const Eigen::Vector3d &acc, const double clk, const double clk_rate,
         const EphemerisKey eph_key);

  Eigen::Vector3d get_pos() const;
  Eigen::Vector3d get_vel() const;
  Eigen::Vector3d get_acc() const;

  double get_clk() const;
  double get_clk_rate() const;

  EphemerisKey get_eph_key() const;

  bool operator==(const SatPVA &rhs) const;

  // helper to copy out the values to C arrays
  void to_arrays(double sat_pos[], double sat_vel[], double sat_acc[],
                 double *sat_clock_err, double *sat_clock_err_rate,
                 u16 *eph_key) const;

 private:
  Eigen::Vector3d pos_;
  Eigen::Vector3d vel_;
  Eigen::Vector3d acc_;
  // In metres and metres second^-1
  double clk_;
  double clk_rate_;
  EphemerisKey eph_key_;
};

}  // namespace pvt_engine

#endif  // STARLING_PVT_ENGINE_SATELLITE_PVA_H
