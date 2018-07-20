/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Measurement Engine team <michele@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef ME_MSG_H
#define ME_MSG_H

#include <ch.h>

#include <libswiftnav/config.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/nav_meas.h>

typedef struct _me_msg_obs_t {
  size_t size;
  navigation_measurement_t obs[MAX_CHANNELS];
  ephemeris_t ephem[MAX_CHANNELS];
  gps_time_t obs_time;
} me_msg_obs_t;

#endif /* #ifndef ME_MSG_H */
