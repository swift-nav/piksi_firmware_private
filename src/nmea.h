/*
 * Copyright (C) 2013-2017 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_NMEA_H
#define SWIFTNAV_NMEA_H

#include <libswiftnav/common.h>
#include <libswiftnav/pvt.h>
#include <libswiftnav/time.h>
#include <libsbp/navigation.h>

#include "track.h"

#define NMEA_GGA_FIX_INVALID 0
#define NMEA_GGA_FIX_GPS     1
#define NMEA_GGA_FIX_DGPS    2
#define NMEA_GGA_FIX_PPS     3
#define NMEA_GGA_FIX_RTK     4
#define NMEA_GGA_FIX_FLOAT   5
#define NMEA_GGA_FIX_EST     6
#define NMEA_GGA_FIX_MANUAL  7
#define NMEA_GGA_FIX_SIM     8

#define MS2KNOTS(x, y, z) (sqrt((x) * (x) + (y) * (y) + (z) * (z)) * 1.94385)
#define MS2KMHR(x, y, z)  (sqrt((x) * (x) + (y) * (y) + (z) * (z)) * \
                          (3600.0 / 1000.0))

void nmea_setup(void);
void nmea_gpgga(const msg_pos_llh_t *sbp_pos_llh, const msg_gps_time_t *sbp_msg_time,
                const msg_dops_t *sbp_dops, double propagation_time, u8 station_id);
void nmea_gpgsa(const u8 *prns, u8 num_prns, const msg_dops_t *sbp_dops);
void nmea_gpgsv(u8 n_used, const navigation_measurement_t *nav_meas,
                const msg_pos_ecef_t *sbp_pos_ecef);
void nmea_gprmc(const msg_pos_llh_t *sbp_pos_llh, const msg_vel_ned_t *sbp_vel_ned,
                const msg_gps_time_t *sbp_gps_time);
void nmea_gpvtg(const msg_vel_ned_t *sbp_vel_ned);
void nmea_gpgll(const msg_pos_llh_t *sbp_pos_llh, const msg_gps_time_t *sbp_msg_time);
void nmea_gpzda(const msg_gps_time_t *sbp_msg_time);
void nmea_send_msgs(const msg_pos_llh_t *sbp_pos_llh, const msg_pos_ecef_t *sbp_pos_ecef,
                    const msg_vel_ned_t *sbp_vel_ned, const msg_dops_t *sbp_dops,
                    const msg_gps_time_t *sbp_msg_time, u8 n_used, const navigation_measurement_t *nav_meas,
                    double propagation_time, u8 sender_id);
char get_nmea_status(u8 flags);
char get_nmea_vel_mode_indicator(u8 flags);
char get_nmea_mode_indicator(u8 flags);

#endif  /* SWIFTNAV_NMEA_H */
