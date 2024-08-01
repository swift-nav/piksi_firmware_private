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

#ifndef SENSORFUSION_DEADRECKONING_DR_DATA_H_
#define SENSORFUSION_DEADRECKONING_DR_DATA_H_

//------------------------------------------------------------------------------
// About:
//
// Raw data types representing "sensor" outputs. These are all POD structs
// that should generalize to all sensors for each given class.
//
// If there is a need for specialized data specific to a certain
// sensor hardware (or similar), it is probably better to find another
// mechanism for providing that information to the DR system.
//------------------------------------------------------------------------------

#include <swiftnav/gnss_time.h>
#include <swiftnav/pvt_result.h>
#include "pvt_common/eigen_custom.h"

namespace sensorfusion {
namespace deadreckoning {
namespace SensorData {

//------------------------------------------------------------------------------
struct imu_6axis_t {
  gps_time_t timestamp;
  // raw accelerometer reading with nominal units: m/s^2
  Eigen::Vector3d raw_acc_xyz;
  // raw gyroscope reading with nominal units: rad/s
  Eigen::Vector3d raw_gyr_xyz;
};

//------------------------------------------------------------------------------
struct gnss_t {
  gps_time_t timestamp;
  // gnss position estimate with units: m
  Eigen::Vector3d ecef_pos_xyz;
  // gnss velocity estimate with units: m/s
  Eigen::Vector3d ecef_vel_xyz;
  // gnss full position and velocity covariance upper three blocks
  Eigen::Matrix3d ecef_pos_xyz_cov;
  Eigen::Matrix3d ecef_vel_xyz_cov;
  Eigen::Matrix3d ecef_pos_vel_xyz_crosscov;

  double pdop;

  bool velocity_valid;
  // gnss solution type
  enum SolutionType {
    NONE = POSITION_MODE_NONE,
    SPP = POSITION_MODE_SPP,
    SBAS = POSITION_MODE_SBAS,
    DGNSS = POSITION_MODE_DGNSS,
    RTKFLOAT = POSITION_MODE_FLOAT,
    RTKFIX = POSITION_MODE_FIXED,
    DEADRECKONING = POSITION_MODE_DEAD_RECKONING,
  } solution_type;
};

}  // namespace SensorData
}  // namespace deadreckoning
}  // namespace sensorfusion

#endif
