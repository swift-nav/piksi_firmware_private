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

#include <stdbool.h>
#include <swiftnav/common.h>
#include <swiftnav/ephemeris.h>
#include <swiftnav/signal.h>

/* Possible satellite code health states */
typedef enum {
  CODE_NAV_STATE_UNKNOWN,
  CODE_NAV_STATE_INVALID,
  CODE_NAV_STATE_VALID,
} code_nav_state_t;

/* Maximum age for almanac page25 health status.
 * Age of full almanac 12.5 min + 1 second to cover timetag inaccuracy. */
#define SHM_ALMA_PAGE25_HEALTH_AGE_S (12.5 * MINUTE_SECS + 1)

/* Satellite health states */
typedef enum { SV_HEALTHY, SV_UNHEALTHY } health_t;

#ifdef __cplusplus
extern "C" {
#endif

void shm_gps_set_shi_ephemeris(u16 sat, u8 new_value);
void shm_gps_set_shi_page25(u16 sat, u8 new_value);
void shm_gps_set_shi_lnav_how_alert(u16 sat, bool new_value);
void shm_gps_set_shi_cnav_alert(u16 sat, bool new_value);

void shm_qzss_set_shi_ephemeris(u16 sat, u8 new_value);
void shm_qzss_set_shi_page25(u16 sat, u8 new_value);
void shm_qzss_set_shi_lnav_how_alert(u16 sat, bool new_value);
void shm_qzss_set_shi_cnav_alert(u16 sat, bool new_value);

void shm_glo_set_shi(u16 sat, u8 new_value);
void shm_bds_set_shi(u16 sat, u8 new_value);
void shm_gal_set_shi(u16 sat, u8 new_value);

void shm_log_sat_state(const char* shi_name, u16 sat);

bool shm_signal_healthy(gnss_signal_t sid);
bool shm_signal_unhealthy(gnss_signal_t sid);
bool shm_navigation_suitable(gnss_signal_t sid);
bool shm_navigation_unusable(gnss_signal_t sid);
bool shm_health_unknown(gnss_signal_t sid);
bool shm_ephe_healthy(const ephemeris_t* ephe, code_t code);

bool shm_alma_page25_health_aged(u32 timetag_s);

#ifdef __cplusplus
}
#endif

#endif /* SWIFTNAV_SHM_H */
