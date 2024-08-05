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

#ifndef LIBSWIFTNAV_PVT_ENGINE_INTERNAL_SBAS_LONG_TERM_CORRECTIONS_H
#define LIBSWIFTNAV_PVT_ENGINE_INTERNAL_SBAS_LONG_TERM_CORRECTIONS_H

#include <pvt_engine/eigen_types.h>
#include <swiftnav/constants.h>
#include <swiftnav/gnss_time.h>

namespace pvt_engine {

// WAAS specification TABLE A-25 En Route, Terminal, NPA Time-Out [s]
constexpr u16 cLongTermTimeout_s = 360;

// Factors
constexpr double cDeltaPositionFactor_m = 0.125;
constexpr double cDeltaAf0Factor_s = C_1_2P31;
constexpr double cDeltaVelocityFactor_m = C_1_2P11;
constexpr double cDeltaAf1Factor_s_s = C_1_2P39;
constexpr u8 cToaFactor_s = 16;

struct LongTermCorrection {
  LongTermCorrection()
      : time_of_applicability(GPS_TIME_UNKNOWN),
        IOD(0),
        delta_pos({0, 0, 0}),
        delta_vel({0, 0, 0}),
        delta_af0(0.0),
        delta_af1(0.0) {}

  gps_time_t time_of_applicability;
  u8 IOD;                     // issue of data, corresponds to GPS IODE
  Eigen::Vector3d delta_pos;  // position correction, from -128 to 128 m
  Eigen::Vector3d delta_vel;  // velocity correction, -0.00625 to 0.00625 m/s
  double delta_af0;           // clock correction, from -2^-21 to 2^-21 s
  double delta_af1;           // clock rate correction, from -2^-32 to 2^-32 s/s
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_INTERNAL_SBAS_LONG_TERM_CORRECTIONS_H
