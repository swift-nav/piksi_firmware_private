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

#include <libsbp/navigation.h>
#include <libsbp/orientation.h>
#include <libswiftnav/common.h>
#include <libswiftnav/gnss_time.h>
#include <libswiftnav/single_epoch_solver.h>

#define FULL_CIRCLE_DEG 360.0f

/* Convert millimeters to meters */
#define MM2M(x) (x / 1000.0)

#define MS2KNOTS(x, y, z) (sqrt((x) * (x) + (y) * (y) + (z) * (z)) * 1.94385)
#define MS2KMHR(x, y, z) \
  (sqrt((x) * (x) + (y) * (y) + (z) * (z)) * (3600.0 / 1000.0))

void nmea_setup(void);
void nmea_send_gsv(u8 n_used, const channel_measurement_t *ch_meas);
char get_nmea_status(u8 flags);
char get_nmea_mode_indicator(u8 flags);
u8 get_nmea_quality_indicator(u8 flags);
#endif /* SWIFTNAV_NMEA_H */
