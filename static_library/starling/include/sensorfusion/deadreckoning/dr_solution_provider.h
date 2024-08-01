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

#ifndef SENSORFUSION_DEADRECKONING_DR_SOLUTION_PROVIDER_H_
#define SENSORFUSION_DEADRECKONING_DR_SOLUTION_PROVIDER_H_

//------------------------------------------------------------------------------
// About:
//
// This file provides the highest-possible level interface to an automobile DR
// system. A factory method is provided for instantiating these objects. Any
// implementation of this interface should really be able to handle anything
// you can throw at it:
//  * out of order measurements
//  * invalid configurations
//  * bad measurements
// All of this is handled internally.
//
// Your job as a client is to simply instantiate an object, and begin piping
// in sensor readings in a relatively timely manner. It should be clear from
// the interface what sensors are expected, and which are optional.
//
// You may also wish to do some amount of runtime configuration of this object.
// There is a modest interface for this, but it is intentionally kept scarce
// as we do not want to facilitate any amount of "config bloat". In general
// you should be configuring an object for your needs on initialization, and
// then letting it do its thing.
//------------------------------------------------------------------------------

#include <limits>
#include <memory>

#include "pvt_common/eigen_custom.h"
#include "sensorfusion/core/error_types.h"
#include "sensorfusion/deadreckoning/dr_data.h"
#include "sensorfusion/deadreckoning/dr_solution.h"

namespace sensorfusion {
namespace deadreckoning {

//------------------------------------------------------------------------------
// Interface for comprehensive automobile dead-reckoning solution.
//------------------------------------------------------------------------------
class DRSolutionProvider {
 protected:
  DRSolutionProvider() = default;

 public:
  //--------------------------------------------------------------------------
  struct Configuration {
    // ***SYSTEM PHYSICAL SPECS***
    // leverarm of primary GNSS antenna and deviation in meters, specified in
    // IMU frame.
    Eigen::Vector3d primary_ant_leverarm_xyz_m;
    double primary_ant_leverarm_dev_m;
    // misalignment of primary IMU within vehicle and deviation in radians.
    // nominal vehicle frame is defined as X = forward, Y = right, Z = down.
    Eigen::Vector3d primary_imu_misalign_ypr_rad;
    double primary_imu_misalign_dev_rad;
    // leverarm of (optional) solutions reference point and deviation in meters,
    // specified in IMU frame.
    Eigen::Vector3d solution_ref_point_leverarm_xyz_m;
    bool solution_ref_point_leverarm_xyz_m_is_set;
    double solution_ref_point_leverarm_dev_m;
    bool solution_ref_point_leverarm_dev_m_is_set;

    // ***SYSTEM TIMING SPECS***
    // maximum sensor latency to allocate in seconds.
    double max_sensor_latency_s;
    // maximum sensor rates to allocate in Hz.
    double max_imu_rate_hz;
    double max_gnss_rate_hz;

    // ***SYSTEM INTEGRITY SPECS***
    // desired protection level error rate in dB.
    double protection_level_error_rate_dB;
  };

 public:
  //--------------------------------------------------------------------------
  virtual ~DRSolutionProvider() = default;

  //--------------------------------------------------------------------------
  // This is an all-or-nothing operation. In general, piecemeal reconfiguration
  // is discouraged. For most use-cases, you should be able to configure once,
  // and then leave the DR system to go about its business.
  virtual MaybeError UpdateConfiguration(const Configuration &config) = 0;

  //--------------------------------------------------------------------------
  // Full internal reset of all estimation state. Stored settings are left
  // unchanged. This method is suitable for effectively "flushing" the
  // DR system of all data, but it is not a reliable way to recover from an
  // unexpected failure.
  virtual void Reset() = 0;

  //--------------------------------------------------------------------------
  // Heavyweight processing function. This will perform as much processing as
  // is required to form a "current solution" at the given time, "now".
  // The workload is a deterministic function of the number of sensor
  // inputs you have provided since the last call to this function.
  //
  // You should be careful to pass in strictly chronological times, as
  // this method will immediately error if you try to run it backwards
  // in time.
  virtual MaybeError RunUntil(const gps_time_t &now) = 0;

  //--------------------------------------------------------------------------
  // Returns whatever solution is currently stored in the system. To get
  // solutions at specific times, use the "run" method defined above.
  virtual DRSolution GetCurrentSolution() const = 0;

  //--------------------------------------------------------------------------
  /// \returns the timestamp associated with the current solution.
  virtual gps_time_t GetCurrentTime() const = 0;

  //--------------------------------------------------------------------------
  // Lightweight sample input methods which are guaranteed to be "fast".
  // The implementation assumes that all timestamps are valid.
  virtual MaybeError AddSamplePrimaryIMU(
      const SensorData::imu_6axis_t &data) = 0;
  virtual MaybeError AddSamplePrimaryGNSS(const SensorData::gnss_t &data) = 0;

 public:
  //--------------------------------------------------------------------------
  // Implementations currently supported by the factory.
  enum class Implementation {
    MOCK,
    GPS_INS,
  };

  //--------------------------------------------------------------------------
  // Factory method for generating solution instances.
  //
  // This is the only part of the public API that allows you
  // to instantiate objects with this interface.
  static std::unique_ptr<DRSolutionProvider> Create(
      const Implementation implementation);
};

}  // namespace deadreckoning
}  // namespace sensorfusion

#endif
