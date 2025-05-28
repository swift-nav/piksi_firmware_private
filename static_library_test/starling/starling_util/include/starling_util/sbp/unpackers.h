/**
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_UTIL_SBP_UNPACKERS_H
#define STARLING_UTIL_SBP_UNPACKERS_H

#include <pvt_common/observations.h>
#include <swiftnav/ephemeris.h>
#include <swiftnav/glonass_phase_biases.h>
#include <swiftnav/ionosphere.h>
#include <swiftnav/nav_meas.h>
#include <swiftnav/sbas_raw_data.h>

#include <libsbp/sbp_msg_type.h>
#include <libsbp/v4/sbp_msg.h>

namespace starling {
namespace util {
namespace sbp {

constexpr int cImuAuxErrorInvalidRange = -1;
constexpr int cImuAuxErrorUnknownImu = -2;
constexpr int cImuAuxErrorInvalidTemperature = -3;
constexpr int cImuAuxExitSuccess = 0;

enum class ImuTimeQuality {
  cTowAlignmentComplete = 0,  // TOW is fully synchronized with GPS clock.
  cTimeSinceStartup,          // TOW is time since startup
  cTowAlignmentUnknown,       // TOW has no known alignment.
  cTowAlignmentRelative,      // TOW fractional part is synchronized to GPS
                              // second.
};

struct ImuData {
  /* Sample time. */
  gps_time_t t;
  /* 3-axis acceleration in m/s^2. */
  double acc_xyz[3];
  /* 3-axis angular velocity in rad/s. */
  double gyr_xyz[3];
};

struct PosEcefData {
  double tow_secs;            // GPS TOW [s]
  double pos_xyz[3];          // ECEF coordinations [m]
  double accuracy_in_meters;  // Position estimated standard deviation [m]
  u8 n_sats;                  // Number of satellites used in solution
  u8 flags;                   // flags
};

nav_meas_flags_t nm_flags_from_sbp(u8 from);

void unpack_obs_header(const sbp_observation_header_t &msg, gps_time_t *t,
                       u8 *total, u8 *count);
/**
 * Convert an SBP observation into the format accepted
 * by the Starling engine.
 */
void unpack_obs_content(const sbp_packed_obs_content_t &msg, double *P,
                        double *L, double *D, double *cn0, double *lock_time,
                        nav_meas_flags_t *flags, gnss_signal_t *sid);
void unpack_obs_content_into_starling_obs(const sbp_packed_obs_content_t &msg,
                                          starling_obs_t *dst);
void unpack_osr_content(const sbp_packed_osr_content_t &msg,
                        starling_obs_t *dst);

void unpack_ephemeris(sbp_msg_type_t msg_type, const sbp_msg_t &msg,
                      ephemeris_t *e);
void unpack_ephemeris(const sbp_msg_ephemeris_gps_t &msg, ephemeris_t *e);
void unpack_ephemeris(const sbp_msg_ephemeris_glo_t &msg, ephemeris_t *e);
void unpack_ephemeris(const sbp_msg_ephemeris_gal_t &msg, ephemeris_t *e);
void unpack_ephemeris(const sbp_msg_ephemeris_bds_t &msg, ephemeris_t *e);
void unpack_ephemeris(const sbp_msg_ephemeris_qzss_t &msg, ephemeris_t *e);
void unpack_ephemeris(const sbp_msg_ephemeris_sbas_t &msg, ephemeris_t *e);

void unpack_sbas_raw_data(const sbp_msg_sbas_raw_t &m, sbas_raw_data_t *d);

void unpack_glonass_biases_content(const sbp_msg_glo_biases_t &msg,
                                   glo_biases_t *glonass_biases);

void unpack_imu_raw(const sbp_msg_imu_raw_t &msg, double accl_sf,
                    double gyro_sf, ImuData *starling_imu_data,
                    ImuTimeQuality *time_quality);

int unpack_imu_aux_accl_sf(const sbp_msg_imu_aux_t &msg, double *accl_sf);

int unpack_imu_aux_gyro_sf(const sbp_msg_imu_aux_t &msg, double *gyro_sf);

int unpack_imu_aux_temp(const sbp_msg_imu_aux_t &msg, double *imu_temp);

void unpack_ephemeris_gal_dep_a(const sbp_msg_ephemeris_gal_dep_a_t &msg,
                                ephemeris_t *e);

void unpack_sbp_gps_time(const sbp_v4_gps_time_t &msg, gps_time_t *output);

void unpack_gps_time(const sbp_msg_gps_time_t &msg, gps_time_t *output);

void unpack_gps_time_gnss(const sbp_msg_gps_time_gnss_t &msg,
                          gps_time_t *output);

void unpack_tow_from_pos_ecef(const sbp_msg_pos_ecef_t &msg,
                              gps_time_t *output);

void unpack_pos_ecef_gnss(const sbp_msg_pos_ecef_gnss_t &msg,
                          PosEcefData *output);

void unpack_tow_from_pos_ecef_cov(const sbp_msg_pos_ecef_cov_t &msg,
                                  gps_time_t *output);

void unpack_tow_from_pos_llh(const sbp_msg_pos_llh_t &msg, gps_time_t *output);

void unpack_tow_from_vel_ned(const sbp_msg_vel_ned_t &msg, gps_time_t *output);

void unpack_tow_from_vel_ned_cov(const sbp_msg_vel_ned_cov_t &msg,
                                 gps_time_t *output);

void unpack_tow_from_orient_euler(const sbp_msg_orient_euler_t &msg,
                                  gps_time_t *output);

void unpack_iono(const sbp_msg_iono_t &msg, ionosphere_t *iono);

}  // namespace sbp
}  // namespace util
}  // namespace starling

#endif
