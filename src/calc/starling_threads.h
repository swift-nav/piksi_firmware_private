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

#ifndef STARLING_CALC_PVT_H
#define STARLING_CALC_PVT_H

#include <libsbp/navigation.h>
#include <libsbp/orientation.h>
#include <libsbp/system.h>
#include <libswiftnav/common.h>
#include <libswiftnav/gnss_time.h>
#include <libswiftnav/observation.h>
#include <libswiftnav/pvt_engine/firmware_binding.h>
#include <libswiftnav/single_epoch_solver.h>

#include "piksi_systime.h"

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

typedef struct {
  piksi_systime_t systime;
  u8 signals_used;
} soln_pvt_stats_t;

/** Maximum time that an observation will be propagated for to align it with a
 * solution epoch before it is discarded.  */
#define OBS_PROPAGATION_LIMIT 10e-3

/* Warn on 15 second base station observation latency */
#define BASE_LATENCY_TIMEOUT 15

/* Make the buffer large enough to handle 15 second latency at 10Hz */
#define STARLING_OBS_N_BUFF BASE_LATENCY_TIMEOUT * 10

extern u32 max_age_of_differential;
extern bool enable_glonass;
extern bool send_heading;

void solution_make_sbp(const pvt_engine_result_t *soln,
                       dops_t *dops,
                       sbp_messages_t *sbp_messages);

double calc_heading(const double b_ned[3]);

soln_dgnss_stats_t solution_last_dgnss_stats_get(void);
void starling_calc_pvt_setup(void);
void reset_rtk_filter(void);
void set_known_ref_pos(const double base_pos[3]);
void set_known_glonass_biases(const glo_biases_t biases);

soln_pvt_stats_t solution_last_pvt_stats_get(void);

// Enable fixed RTK mode in the Starling engine.
void starling_set_enable_fix_mode(bool is_fix_enabled);
// Indicate for how long corrections should persist.
void starling_set_max_correction_age(int max_age);

#endif
