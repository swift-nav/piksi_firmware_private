/*
 * Copyright (C) 2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_CALC_STARLING_PLATFORM_SHIM_H
#define STARLING_CALC_STARLING_PLATFORM_SHIM_H

#include <libswiftnav/gnss_time.h>
#include <ch.h>

/**
 * This is the dumping ground header for everything required
 * to get the starling threads to compile correctly from 
 * outside of the firmware.
 *
 * Here you will find the definitions of types and functions
 * which are not accessible by includes in 'common' or 'libswiftnav'.
 *
 * Ultimately, the contents of this file should be reduced to 0.
 */

extern dgnss_solution_mode_t dgnss_soln_mode;
extern dgnss_filter_t dgnss_filter;

extern FilterManager *time_matched_filter_manager;
extern FilterManager *low_latency_filter_manager;
extern FilterManager *spp_filter_manager;

extern MUTEX_DECL(time_matched_filter_manager_lock);
extern MUTEX_DECL(low_latency_filter_manager_lock);
extern MUTEX_DECL(spp_filter_manager_lock);

extern MUTEX_DECL(time_matched_iono_params_lock);
extern bool has_time_matched_iono_params;
extern ionosphere_t time_matched_iono_params;

extern MUTEX_DECL(last_sbp_lock);
extern gps_time_t last_dgnss;
extern gps_time_t last_spp;
extern gps_time_t last_time_matched_rover_obs_post;

#endif
