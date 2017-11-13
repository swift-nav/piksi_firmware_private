/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Michele Bavaro <michele@swift-nav.com>
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
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/signal.h>

/* Possible satellite code health states */
typedef enum {
  CODE_NAV_STATE_UNKNOWN,
  CODE_NAV_STATE_INVALID,
  CODE_NAV_STATE_VALID,
} code_nav_state_t;

/* GLO satellite health states */
typedef enum { GLO_SV_HEALTHY, GLO_SV_UNHEALTHY } glo_health_t;

void shm_gps_set_shi1(u16 sat, u8 new_value);
void shm_gps_set_shi4(u16 sat, bool new_value);
void shm_gps_set_shi6(u16 sat, bool new_value);
void shm_glo_set_shi(u16 sat, u8 new_value);

void shm_log_sat_state(const char* shi_name, u16 sat);

bool shm_signal_healthy(gnss_signal_t sid);
bool shm_signal_unhealthy(gnss_signal_t sid);
bool shm_navigation_suitable(gnss_signal_t sid);
bool shm_navigation_unusable(gnss_signal_t sid);
bool shm_health_unknown(gnss_signal_t sid);
bool shm_ephe_healthy(const ephemeris_t *ephe, const code_t code);

#endif /* SWIFTNAV_SHM_H */
