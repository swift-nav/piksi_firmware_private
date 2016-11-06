/*
 * Copyright (C) 2013-2016 Swift Navigation Inc.
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

#include "track.h"

/** \addtogroup nmea
 * \{ */

#define NMEA_GGA_FIX_INVALID 0
#define NMEA_GGA_FIX_GPS     1
#define NMEA_GGA_FIX_DGPS    2
#define NMEA_GGA_FIX_PPS     3
#define NMEA_GGA_FIX_RTK     4
#define NMEA_GGA_FIX_FLOAT   5
#define NMEA_GGA_FIX_EST     6
#define NMEA_GGA_FIX_MANUAL  7
#define NMEA_GGA_FIX_SIM     8

#define MS2KNOTTS(x,y,z)     sqrt((x)*(x) + (y)*(y) + (z)*(z)) * 1.94385
#define MS2KMHR(x,y,z)       sqrt((x)*(x)+(y)*(y)+(z)*(z)) * (3600.0/1000.0)

extern u32 gpgsv_msg_rate;
extern u32 gprmc_msg_rate;
extern u32 gpvtg_msg_rate;
extern u32 gpgll_msg_rate;
extern u32 gpzda_msg_rate;
extern u32 gpgsa_msg_rate;

void nmea_setup(void);
void nmea_gpgga(const double pos_llh[3], const gps_time_t *gps_t, u8 n_used,
                u8 fix_type, double hdop, double diff_age, u16 station_id);
void nmea_assemble_gpgsa(const dops_t *dops);
void nmea_gpgsa(const u8 *prns, u8 num_prns, const dops_t *dops);
void nmea_gpgsv(const double pos_ecef[3], const gps_time_t *gps_t);
void nmea_gprmc(const double pos_llh[3], const double vel_ned[3],
                const gps_time_t *gps_t);
void nmea_gpvtg(const double vel_ned[3]);
void nmea_gpgll(const double pos_llh[3], const gps_time_t *gps_t);
void nmea_gpzda(const gps_time_t *gps_t);

/** Register a new dispatcher for NMEA messages
 *
 * \param send_func Pointer to dispatcher function.
 */
#define nmea_dispatcher_register(send_func) do {         \
  static struct nmea_dispatcher dispatcher = \
    { .send = send_func }; \
  _nmea_dispatcher_register(&dispatcher); \
} while(0)

/** \cond */
struct nmea_dispatcher {
  void (*send)(const char *msg, size_t msg_size);
  struct nmea_dispatcher *next;
};

void _nmea_dispatcher_register(struct nmea_dispatcher *);
/** \endcond */

/** \} */

#endif  /* SWIFTNAV_NMEA_H */
