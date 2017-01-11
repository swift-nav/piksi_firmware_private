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

#ifndef SWIFTNAV_SOLUTION_H
#define SWIFTNAV_SOLUTION_H

#include <ch.h>
#include <libsbp/navigation.h>
#include <libswiftnav/common.h>
#include <libswiftnav/pvt.h>
#include <libswiftnav/track.h>
#include <libswiftnav/time.h>
#include <libswiftnav/observation.h>

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
  u8 signals_tracked;
  u8 signals_useable;
} soln_stats_t;

typedef struct {
  systime_t systime;
  u8 signals_used;
} soln_pvt_stats_t;

typedef struct {
  systime_t systime;
  dgnss_filter_t mode;
} soln_dgnss_stats_t;

/** Maximum time that an observation will be propagated for to align it with a
 * solution epoch before it is discarded.  */
#define OBS_PROPAGATION_LIMIT 10e-3

#define OBS_N_BUFF 60

#define OBS_BUFF_SIZE (OBS_N_BUFF * sizeof(obss_t))

extern double soln_freq;
extern u32 obs_output_divisor;
extern u32 max_age_of_differential;

void solution_send_pos_messages(double propagation_time, u8 sender_id, u8 n_used,
                                const navigation_measurement_t *nav_meas,
                                const msg_gps_time_t *gps_time, const msg_pos_llh_t *pos_llh,
                                const msg_pos_ecef_t *pos_ecef, const msg_vel_ned_t *vel_ned,
                                const msg_vel_ecef_t * vel_ecef, const msg_dops_t *sbp_dops,
                                const msg_baseline_ned_t *baseline_ned, const msg_baseline_ecef_t *baseline_ecef,
                                const msg_baseline_heading_t *baseline_heading);
void solution_make_sbp(const gnss_solution *soln, dops_t *dops, bool clock_jump, msg_gps_time_t *gps_time,
                       msg_pos_llh_t *pos_llh, msg_pos_ecef_t *pos_ecef,
                       msg_vel_ned_t *vel_ned, msg_vel_ecef_t *vel_ecef,
                       msg_dops_t *sbp_dops);
void extract_covariance(double full_covariance[9], const gnss_solution *soln);
double calc_heading(const double b_ned[3]);
void solution_make_baseline_sbp(const gps_time_t *t, u8 n_sats, double b_ecef[3],
                                double covariance_ecef[9], double ref_ecef[3], u8 flags, dops_t *dops,
                                msg_pos_llh_t *pos_llh, msg_pos_ecef_t *pos_ecef,
                                msg_baseline_ned_t *baseline_ned, msg_baseline_ecef_t *baseline_ecef,
                                msg_baseline_heading_t *baseline_heading, msg_dops_t *sbp_dops);
soln_stats_t solution_last_stats_get(void);
soln_pvt_stats_t solution_last_pvt_stats_get(void);
soln_dgnss_stats_t solution_last_dgnss_stats_get(void);
void solution_setup(void);
void reset_rtk_filter(void);

#endif
