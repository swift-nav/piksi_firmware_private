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

#ifndef SENSORFUSION_DEADRECKONING_DR_SOLUTION_H_
#define SENSORFUSION_DEADRECKONING_DR_SOLUTION_H_

//------------------------------------------------------------------------------
// About:
//
// Dead reckoning solution type. This is what you can expect to periodically
// recieve from any DR system. There is a lot of information packed into
// this type, but it is grouped logically, and each sub-element
// should be self-explanatory.
//
// It is preferred to use the provided "set_*()" methods to ensure that
// the validity flags are maintained consistently.
//------------------------------------------------------------------------------

#include <swiftnav/gnss_time.h>
#include <swiftnav/pvt_result.h>

#include <limits>
#include <optional.hpp>

#include "pvt_common/eigen_custom.h"

namespace sensorfusion {
namespace deadreckoning {

struct DRSolution {
 public:
  //----------------------------------------------------------------------------
  struct Position {
    // ECEF position vector in meters.
    Eigen::Vector3d ecef_xyz_m;
    // ECEF position covariance in meters^2.
    Eigen::Matrix3d ecef_xyz_m_cov;
    // Last GNSS solution type used to update the filter
    SensorData::gnss_t::SolutionType solution_type;
  };

  //----------------------------------------------------------------------------
  struct Velocity {
    // NED velocity vector in meters per second.
    Eigen::Vector3d ned_mps;
    // NED velocity covariance in meters^2 per second^2
    Eigen::Matrix3d ned_mps_cov;
  };

  //----------------------------------------------------------------------------
  struct Attitude {
    // Attitude Tait-Bryan angles in radians, relative to local NED frame.
    Eigen::Vector3d ned_ypr_rad;
    // Attitude Tait-Bryan covariance in radians^2.
    Eigen::Matrix3d ned_ypr_rad_cov;
  };

  //----------------------------------------------------------------------------
  struct LeverArm {
    // Lever arm from center of navigation to sensor in computation frame, in m.
    Eigen::Vector3d lever_arm_m;
    // Lever arm (from center of navigation to sensor in computation frame)
    // covariance, in m^2.
    Eigen::Matrix3d lever_arm_m_cov;
  };

  //----------------------------------------------------------------------------
  struct ProtectionLevel {
    // Position protection levels in meters.
    double hori_m;
    double vert_m;
    // General protection level error rate in dB.
    double error_rate_dB;
  };

  //----------------------------------------------------------------------------
  enum class ProcessingState {
    INITIALIZING,
    ALIGNING,
    CONVERGING,
    TRACKING,
    DEADRECKONING,
    SIMULATED_DROPOUT,
  };

  //----------------------------------------------------------------------------
  // Time of validity.
  gps_time_t time = GPS_TIME_UNKNOWN;

  ProcessingState processing_state = DRSolution::ProcessingState::INITIALIZING;
  bool gnss_meas_rejected = false;

  std::experimental::optional<Position> position = {};
  std::experimental::optional<Velocity> velocity = {};
  std::experimental::optional<Attitude> attitude = {};
  std::experimental::optional<ProtectionLevel> protection_level = {};
  std::experimental::optional<LeverArm> gnss_lever_arm = {};

  u32 get_sbp_ins_status() const {
    u32 ins_status_sbp_field = 0;
    switch (processing_state) {
      case DRSolution::ProcessingState::INITIALIZING:
        ins_status_sbp_field = INS_STATUS_AWAITING_INIT;  // 0
        break;
      case DRSolution::ProcessingState::ALIGNING:
        ins_status_sbp_field = INS_STATUS_ALIGNING;  // 1
        break;
      case DRSolution::ProcessingState::TRACKING:
        ins_status_sbp_field = INS_STATUS_READY;  // 2
        break;
      default:
        assert(0 && "Unexpected processing_state in get_sbp_ins_status()");
    }
    return ins_status_sbp_field;
  }
};

}  // namespace deadreckoning
}  // namespace sensorfusion

#endif
