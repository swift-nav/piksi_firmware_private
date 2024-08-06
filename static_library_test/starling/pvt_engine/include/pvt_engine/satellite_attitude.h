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

#ifndef LIBSWIFTNAV_PVT_ENGINE_SATELLITE_ATTITUDE_H
#define LIBSWIFTNAV_PVT_ENGINE_SATELLITE_ATTITUDE_H

#include <pvt_common/containers/map.h>
#include <pvt_common/optional.h>
#include <pvt_engine/RTKLib_apriori_models/rtklib_common_antenna.h>
#include <pvt_engine/RTKLib_apriori_models/rtklib_common_tides.h>
#include <pvt_engine/sat_identifier.h>
#include <starling/build/config.h>
#include <swiftnav/constants.h>
#include <swiftnav/linear_algebra.h>

namespace pvt_engine {

constexpr double GPS_ORBITAL_RATE = 0.00836;
constexpr double GLONASS_ORBITAL_RATE = 0.00888;

constexpr double GPS_IIR_MAX_YAW_RATE = 0.20;
constexpr double GPS_IIF_MAX_YAW_RATE = 0.11;
constexpr double GLONASS_MAX_YAW_RATE = 0.25;
constexpr double BEIDOU_2G_MAX_YAW_RATE = 0.085;
constexpr double BEIDOU_2M_MAX_YAW_RATE = 0.158;

constexpr double GPS_IIA_YAW_BIAS = 0.5;
constexpr double GPS_IIF_YAW_BIAS = -0.7;

// These values match those specified in SBP. Only append new values,
// do not remove values or insert new values in the middle.
enum class SatelliteType {
  UNKNOWN = 0,
  GPS_I,
  GPS_II,
  GPS_IIA,
  GPS_IIR,
  GPS_IIF,
  GPS_III,
  GLONASS,
  GLONASS_M,
  GLONASS_K1,
  GALILEO,
  BEIDOU_2G,
  BEIDOU_2I,
  BEIDOU_2M,
  BEIDOU_3M_SECM,
  BEIDOU_3G_SECM,
  BEIDOU_3M_CAST,
  BEIDOU_3G_CAST,
  BEIDOU_3I_CAST,
  QZSS,
  COUNT,
};

SatelliteType get_satellite_type(uint16_t int_value);

double wrap_angle_degrees(double angle);

SatelliteType satellite_type_from_str(const std::string &type);
SatelliteType satellite_type_from_str(const char *type);
const char *satellite_type_to_str(SatelliteType type);

struct SatelliteBodyVectors {
  Eigen::Vector3d x;
  Eigen::Vector3d y;
  Eigen::Vector3d z;
  SatelliteBodyVectors() : x(), y(), z() {}
};

using SVN = u16;
struct SatelliteAttitudeData {
  SatelliteType satellite_type;
  optional<double> beta_initial;
  optional<double> yaw_initial;
  optional<gps_time_t> time_initial;
  SVN svn;

  SatelliteAttitudeData()
      : satellite_type(SatelliteType::UNKNOWN),
        beta_initial(),
        yaw_initial(),
        time_initial(),
        svn(0) {}
};

using AttitudeData =
    pvt_common::containers::Map<SatIdentifier, SatelliteAttitudeData, cNumSat>;

class SatelliteAttitude {
 public:
  SatelliteAttitude();
  void initialize(const std::string &external_data_path,
                  const std::string &atx_filename);

  bool update_config(const std::string &external_data_path,
                     const std::string &atx_filename);

  optional<double> compute_expected_yaw_angle(
      const gps_time_t &time, const pvt_engine::SatIdentifier &sat_id,
      const Eigen::Vector3d &sat_pos, const Eigen::Vector3d &sat_vel,
      const Eigen::Vector3d &sun_pos);

  optional<SatelliteBodyVectors> compute_satellite_orientation(
      const gps_time_t &time, const gnss_signal_t &sid,
      const Eigen::Vector3d &sat_pos, const Eigen::Vector3d &sat_vel,
      const Eigen::Vector3d &sun_pos,
      const optional<double> &precomputed_yaw_angle);

  void update_satellite_info(const SatellitePCVMap &sat_apc_map);

 private:
  struct YawAngles {
    double nominal_yaw_angle;
    double expected_yaw_angle;
  };

  // non-const because this may cache some computations for future maneuvers
  optional<YawAngles> compute_yaw_angles(
      const gps_time_t &time, const pvt_engine::SatIdentifier &sat_id,
      const Eigen::Vector3d &sat_pos, const Eigen::Vector3d &sat_vel,
      const Eigen::Vector3d &sun_pos,
      const optional<double> &precomputed_yaw_angle);

  optional<double> get_maximal_yaw_rate(const SatIdentifier &sat_id) const;

  double get_yaw_bias(const SatIdentifier &sat_id) const;

  double compute_expected_yaw_angle(const gps_time_t &time,
                                    const pvt_engine::SatIdentifier &sat_id,
                                    const double beta,
                                    const double orbital_angle,
                                    const double nominal_yaw) const;

  PRC read_satellite_type(const gps_time_t &time);

  AttitudeData attitude_data_;
  pvt_common::containers::Map<SVN, double, NUM_SATS_GPS> gps_IIA_yaw_rates_;
  std::string external_data_path_;
  std::string atx_filename_;
  bool attempted_to_read_pcvs_;
};

// Utility functions
double adjust_yaw_interval(double yaw);

double clamp(const double value, const double low_bound,
             const double high_bound);

double compute_beta_angle(const Eigen::Vector3d &sun_pos,
                          const Eigen::Vector3d &orbit_normal);

double compute_eclipse_angle(const Eigen::Vector3d &sat_pos,
                             const Eigen::Vector3d &sun_pos);

double compute_nominal_yaw_angle(const double beta, const double orbital_angle);

double compute_nominal_yaw_rate(double beta, double orbital_angle,
                                const double orbital_rate);

double compute_orbital_angle(const Eigen::Vector3d &sun_pos,
                             const Eigen::Vector3d &sat_ini_vel,
                             const double beta, const double eclipse_angle);

Eigen::Vector3d compute_satellite_inertial_velocity(
    const Eigen::Vector3d &sat_pos, const Eigen::Vector3d &sat_vel);

void rotate_satellite(const double angle, Eigen::Vector3d *sat_body_x,
                      Eigen::Vector3d *sat_body_y);

// Eclipsing functions
double gps_IIA_midnight_turn(const double beta, const double orbital_angle,
                             const double max_yaw_rate);
double gps_IIF_midnight_turn(const double beta, const double orbital_angle);
double gps_noon_turn(const double beta, const double orbital_angle,
                     const double max_yaw_rate, const double yaw_bias,
                     const double orbital_angle_offset);
double glonass_midnight_turn(const double beta, const double orbital_angle);
double glonass_noon_turn(const double beta, const double orbital_angle);
double yaw_bds_cast(const double dt, const double beta,
                    const double orbital_angle, const double yaw0,
                    const double yaw_rate);
double yaw_bds_secm(const double beta, const double orbital_angle);

}  // namespace pvt_engine
#endif  // LIBSWIFTNAV_PVT_ENGINE_SATELLITE_ATTITUDE_H
