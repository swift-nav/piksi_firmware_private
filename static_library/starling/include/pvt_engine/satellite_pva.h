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

class SatPVA {
 public:
  SatPVA();

  SatPVA(const double pos[], const double vel[], const double acc[],
         const double clk, const double clk_rate, const u8 iode,
         const u16 iodc);

  SatPVA(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel,
         const Eigen::Vector3d &acc, const double clk, const double clk_rate,
         const u8 iode, const u16 iodc);

  Eigen::Vector3d get_pos() const;
  Eigen::Vector3d get_vel() const;
  Eigen::Vector3d get_acc() const;

  double get_clk() const;
  double get_clk_rate() const;

  u8 get_iode() const;
  u16 get_iodc() const;

  bool operator==(const SatPVA &rhs) const;

 private:
  Eigen::Vector3d pos_;
  Eigen::Vector3d vel_;
  Eigen::Vector3d acc_;
  // In metres and metres second^-1
  double clk_;
  double clk_rate_;
  u8 iode_;
  u16 iodc_;
};

}  // namespace pvt_engine

#endif  // STARLING_PVT_ENGINE_SATELLITE_PVA_H
