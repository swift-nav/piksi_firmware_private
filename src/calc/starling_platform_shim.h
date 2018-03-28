/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Kevin Dade <kevin@swiftnav.com>
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

#include <libsbp/navigation.h>
#include <libsbp/orientation.h>
#include <libsbp/system.h>
#include <libswiftnav/gnss_time.h>

// All of these need to go.
#include "calc_base_obs.h"
#include "manage.h"
#include "me_msg/me_msg.h"
#include "sbp_utils.h"
#include "simulator.h"

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

////////////////////////////////////////////////////////////////////////////////
// Constants
////////////////////////////////////////////////////////////////////////////////
#define SPP_ECEF_SIZE 3
/** number of milliseconds before SPP resumes in pseudo-absolute mode */
#define DGNSS_TIMEOUT_MS 5000
/* Warn on 15 second base station observation latency */
#define BASE_LATENCY_TIMEOUT 15

////////////////////////////////////////////////////////////////////////////////
// Types
////////////////////////////////////////////////////////////////////////////////
typedef enum {
  SOLN_MODE_LOW_LATENCY,
  SOLN_MODE_TIME_MATCHED,
  SOLN_MODE_NO_DGNSS
} dgnss_solution_mode_t;

typedef enum {
  FILTER_FLOAT,
  FILTER_FIXED,
} dgnss_filter_t;

typedef struct {
  piksi_systime_t systime;
  dgnss_filter_t mode;
} soln_dgnss_stats_t;

typedef struct {
  msg_gps_time_t gps_time;
  msg_utc_time_t utc_time;
  msg_pos_llh_t pos_llh;
  msg_pos_ecef_t pos_ecef;
  msg_vel_ned_t vel_ned;
  msg_vel_ecef_t vel_ecef;
  msg_dops_t sbp_dops;
  msg_age_corrections_t age_corrections;
  msg_dgnss_status_t dgnss_status;
  msg_baseline_ecef_t baseline_ecef;
  msg_baseline_ned_t baseline_ned;
  msg_baseline_heading_t baseline_heading;
  msg_pos_ecef_cov_t pos_ecef_cov;
  msg_vel_ecef_cov_t vel_ecef_cov;
  msg_pos_llh_cov_t pos_llh_cov;
  msg_vel_ned_cov_t vel_ned_cov;
} sbp_messages_t;

////////////////////////////////////////////////////////////////////////////////
// Globals
////////////////////////////////////////////////////////////////////////////////
extern memory_pool_t time_matched_obs_buff_pool;
extern mailbox_t time_matched_obs_mailbox;

extern dgnss_solution_mode_t dgnss_soln_mode;
extern dgnss_filter_t dgnss_filter;

extern mutex_t last_sbp_lock;
extern gps_time_t last_dgnss;
extern gps_time_t last_spp;
extern gps_time_t last_time_matched_rover_obs_post;

extern double starling_frequency;

extern bool disable_klobuchar;
extern bool enable_glonass;
extern float glonass_downweight_factor;

extern u8 current_base_sender_id;

extern sbas_system_t current_sbas_system;

// This regrettable hack is necessary because of the header spaghetti elsewhere.
#ifndef ME_CALC_PVT_H
extern double soln_freq_setting;
#endif

////////////////////////////////////////////////////////////////////////////////
// Function Dependencies
////////////////////////////////////////////////////////////////////////////////
void post_observations(u8 n,
                       const navigation_measurement_t m[],
                       const gps_time_t *t,
                       const pvt_engine_result_t *soln);

void solution_send_low_latency_output(
    u8 base_sender_id,
    const sbp_messages_t *sbp_messages,
    u8 n_meas,
    const navigation_measurement_t nav_meas[]);

void solution_simulation(sbp_messages_t *sbp_messages);

void sbp_messages_init(sbp_messages_t *sbp_messages, gps_time_t *t);

void solution_send_pos_messages(u8 base_sender_id,
                                const sbp_messages_t *sbp_messages,
                                u8 n_meas,
                                const navigation_measurement_t nav_meas[]);

void solution_make_baseline_sbp(const pvt_engine_result_t *result,
                                const double spp_ecef[SPP_ECEF_SIZE],
                                const dops_t *dops,
                                sbp_messages_t *sbp_messages);

void solution_make_sbp(const pvt_engine_result_t *soln,
                       dops_t *dops,
                       sbp_messages_t *sbp_messages);

////////////////////////////////////////////////////////////////////////////////
// Platform Shim Functions
////////////////////////////////////////////////////////////////////////////////
void platform_initialize_settings(void);
void platform_initialize_memory_pools(void);
void platform_mutex_lock(mutex_t *mtx);
void platform_mutex_unlock(mutex_t *mtx);
void platform_pool_free(void *pool, void *buf);
void platform_thread_create_static(
    void *wa, size_t wa_size, int prio, void (*fn)(void *), void *user);
void platform_thread_set_name(const char *name);
// Return true on success.
bool platform_try_read_ephemeris(const gnss_signal_t sid, ephemeris_t *eph);
// Return true on success.
bool platform_try_read_iono_corr(ionosphere_t *params);
void platform_watchdog_notify_starling_main_thread(void);

#define PLATFORM_THD_WORKING_AREA(s, n) THD_WORKING_AREA(s, n)

#define PLATFORM_MUTEX_DECL(m) MUTEX_DECL(m)

#endif
