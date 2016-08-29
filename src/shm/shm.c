/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Roman Gezikov <rgezikov@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <ch.h>
#include <libswiftnav/shm.h>
#include <libswiftnav/signal.h>
#include "shm.h"

static MUTEX_DECL(shm_data_access);

gps_sat_health_indicators_t gps_shis[NUM_SATS_GPS];

void shm_gps_set_shi1(u16 sat, u8 new_value)
{
  (void)(sat);
  (void)(new_value);
}

void shm_gps_set_shi4(u16 sat, bool new_value)
{
  (void)(sat);
  (void)(new_value);
}

void shm_gps_set_shi6(u16 sat, bool new_value)
{
  (void)(sat);
  (void)(new_value);
}

code_nav_state_t shm_get_sat_state(gnss_signal_t sid)
{
  (void)(sid);
  return CODE_NAV_STATE_UNKNOWN;
}

bool shm_gps_l1ca_tracking_allowed(u16 sat)
{
  (void)(sat);
  return false;
}

bool shm_gps_l2cm_tracking_allowed(u16 sat)
{
  (void)(sat);
  return false;
}

bool shm_gps_navigation_suitable(gnss_signal_t sid)
{
  (void)(sid);
  return false;
}

