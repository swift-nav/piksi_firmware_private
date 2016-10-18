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

#ifndef SWIFTNAV_SHM_H
#define SWIFTNAV_SHM_H

#include <libswiftnav/common.h>

/* Possible satellite code health states */
typedef enum {
  CODE_NAV_STATE_UNKNOWN,
  CODE_NAV_STATE_INVALID,
  CODE_NAV_STATE_VALID,
} code_nav_state_t;

void shm_gps_set_shi1(u16 sat, u8 new_value);
void shm_gps_set_shi4(u16 sat, bool new_value);
void shm_gps_set_shi6(u16 sat, bool new_value);

code_nav_state_t shm_get_sat_state(gnss_signal_t sid);
void shm_log_sat_state(const char* shi_name, u16 sat);

bool shm_tracking_allowed(gnss_signal_t sid);
bool shm_navigation_suitable(gnss_signal_t sid);
bool shm_navigation_unusable(gnss_signal_t sid);

#endif /* SWIFTNAV_SHM_H */
