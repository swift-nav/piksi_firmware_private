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

#ifndef LEGACY_STARLING_UTIL_SBP_UNPACKERS_H
#define LEGACY_STARLING_UTIL_SBP_UNPACKERS_H

#include <legacy_starling_util/sbp/messages.h>
#include <pvt_common/observations.h>
#include <swiftnav/ephemeris.h>
#include <swiftnav/glonass_phase_biases.h>
#include <swiftnav/ionosphere.h>
#include <swiftnav/nav_meas.h>
#include <swiftnav/sbas_raw_data.h>

#define IMU_AUX_ERROR_INVALID_RANGE (-1)
#define IMU_AUX_ERROR_UNKNOWN_IMU (-2)
#define IMU_AUX_ERROR_INVALID_TEMPERATURE (-3)
#define IMU_AUX_EXIT_SUCCESS (0)

#ifdef __cplusplus
extern "C" {
#endif

enum {
  IMU_TOW_ALIGNMENT_COMPLETE,  // TOW is fully synchronized with GPS clock.
  IMU_TIME_SINCE_STARTUP,      // TOW is time since startup
  IMU_TOW_ALIGNMENT_UNKNOWN,   // TOW has no known alignment.
  IMU_TOW_ALIGNMENT_RELATIVE,  // TOW fractional part is synchronized to GPS
                               // second.
};
typedef int imu_time_quality_t;

typedef struct {
  /* Sample time. */
  gps_time_t t;
  /* 3-axis acceleration in m/s^2. */
  double acc_xyz[3];
  /* 3-axis angular velocity in rad/s. */
  double gyr_xyz[3];
} imu_data_t;

typedef struct {
  double tow_secs;            // GPS TOW [s]
  double pos_xyz[3];          // ECEF coordinations [m]
  double accuracy_in_meters;  // Position estimated standard deviation [m]
  u8 n_sats;                  // Number of satellites used in solution
  u8 flags;                   // flags
} pos_ecef_data_t;

nav_meas_flags_t nm_flags_from_sbp(u8 from);

void unpack_obs_header(const observation_header_t *msg, gps_time_t *t,
                       u8 *total, u8 *count);
/**
 * Convert an SBP observation into the format accepted
 * by the Starling engine.
 */
void unpack_obs_content(const packed_obs_content_t *msg, double *P, double *L,
                        double *D, double *cn0, double *lock_time,
                        nav_meas_flags_t *flags, gnss_signal_t *sid);
void unpack_obs_content_into_starling_obs(const packed_obs_content_t *msg,
                                          starling_obs_t *dst);
void unpack_osr_content(const packed_osr_content_t *msg, starling_obs_t *dst);

void unpack_ephemeris(const msg_ephemeris_t *msg, ephemeris_t *e);

void unpack_sbas_raw_data(const msg_sbas_raw_t *m, sbas_raw_data_t *d);

void sbp_unpack_glonass_biases_content(msg_glo_biases_t msg,
                                       glo_biases_t *glonass_biases);

void sbp_unpack_imu_raw(const u8 *msg, double accl_sf, double gyro_sf,
                        imu_data_t *starling_imu_data,
                        imu_time_quality_t *time_quality);

int sbp_unpack_imu_aux_accl_sf(const u8 *msg, double *accl_sf);

int sbp_unpack_imu_aux_gyro_sf(const u8 *msg, double *gyro_sf);

int sbp_unpack_imu_aux_temp(const u8 *msg, double *imu_temp);

void unpack_ephemeris_gal_dep_a(const msg_ephemeris_gal_dep_a_t *msg,
                                ephemeris_t *e);

void sbp_unpack_sbp_gps_time(const sbp_gps_time_t *msg, gps_time_t *output);

void sbp_unpack_gps_time(const msg_gps_time_t *msg, gps_time_t *output);

void sbp_unpack_gps_time_gnss(const msg_gps_time_gnss_t *msg,
                              gps_time_t *output);

void sbp_unpack_tow_from_pos_ecef(const msg_pos_ecef_t *msg,
                                  gps_time_t *output);

void sbp_unpack_pos_ecef_gnss(const msg_pos_ecef_gnss_t *msg,
                              pos_ecef_data_t *output);

void sbp_unpack_tow_from_pos_ecef_cov(const msg_pos_ecef_cov_t *msg,
                                      gps_time_t *output);

void sbp_unpack_tow_from_pos_llh(const msg_pos_llh_t *msg, gps_time_t *output);

void sbp_unpack_tow_from_vel_ned(const msg_vel_ned_t *msg, gps_time_t *output);

void sbp_unpack_tow_from_vel_ned_cov(const msg_vel_ned_cov_t *msg,
                                     gps_time_t *output);

void sbp_unpack_tow_from_orient_euler(const msg_orient_euler_t *msg,
                                      gps_time_t *output);

void sbp_unpack_iono(const msg_iono_t *msg, ionosphere_t *iono);

#ifdef __cplusplus
}
#endif

#endif  // LEGACY_STARLING_UTIL_SBP_UNPACKERS_H
