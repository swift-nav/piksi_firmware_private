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
#include <pvt_engine/RTKLib_apriori_models/rtklib_common_antenna.h>
#include <pvt_engine/RTKLib_apriori_models/rtklib_common_tides.h>
#include <pvt_engine/optional.h>
#include <pvt_engine/sat_identifier.h>
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

constexpr u8 MAX_SATELLITE_TYPES = 14;
enum class SatelliteType {
  SATELLITE_TYPE_UNKNOWN,
  GPS_I,
  GPS_II,
  GPS_IIA,
  GPS_IIR,
  GPS_IIF,
  GLONASS,
  GLONASS_M,
  GLONASS_K1,
  GALILEO,
  BEIDOU_2G,
  BEIDOU_2I,
  BEIDOU_2M,
  QZSS
};

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
  SVN svn;

  SatelliteAttitudeData()
      : satellite_type(SatelliteType::SATELLITE_TYPE_UNKNOWN),
        beta_initial(),
        svn(0) {}
};

using AttitudeData =
    pvt_common::containers::Map<SatIdentifier, SatelliteAttitudeData, NUM_SATS>;

class SatelliteAttitude {
 public:
  SatelliteAttitude();
  void initialize(const std::string &external_data_path,
                  const std::string &atx_filename);

  double compute_expected_yaw_angle(const gnss_signal_t &sid, const double beta,
                                    const double orbital_angle) const;

  optional<SatelliteBodyVectors> compute_satellite_orientation(
      const gps_time_t &time, const gnss_signal_t &sid,
      const Eigen::Vector3d &sat_pos, const Eigen::Vector3d &sat_vel,
      const Eigen::Vector3d &sun_pos, const optional<double> &yaw_angle);

 private:
  SatelliteType define_satellite_type(const std::string &type) const;

  optional<double> get_maximal_yaw_rate(const SatIdentifier &sat_id) const;

  double get_yaw_bias(const SatIdentifier &sat_id) const;

  PRC read_satellite_type(const gps_time_t &time);

  AttitudeData attitude_data_;
  pvt_common::containers::Map<SVN, double, NUM_SATS_GPS> gps_IIA_yaw_rates_;
  std::string external_data_path_;
  std::string atx_filename_;
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

};      // namespace pvt_engine
#endif  // LIBSWIFTNAV_PVT_ENGINE_SATELLITE_ATTITUDE_H
