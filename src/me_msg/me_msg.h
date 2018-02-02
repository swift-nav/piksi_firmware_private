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

#include <libsbp/sbas.h>
#include <libswiftnav/config.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/nav_meas.h>

enum {
  ME_MSG_OBS = 1,
  ME_MSG_SBAS_RAW = 2
};

typedef struct _me_msg_obs_t {
  size_t size;
  navigation_measurement_t obs[MAX_CHANNELS];
  ephemeris_t ephem[MAX_CHANNELS];
  gps_time_t obs_time;
} me_msg_obs_t;

typedef struct _me_msg_t {
  u32 id;
  union {
    me_msg_obs_t obs;
    msg_sbas_raw_t sbas;
  } msg;
} me_msg_t;

extern mailbox_t me_msg_mailbox;
extern memory_pool_t me_msg_buff_pool;

void me_msg_setup(void);

#endif /* #ifndef ME_MSG_H */
