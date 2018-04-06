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

/** number of milliseconds before SPP resumes in pseudo-absolute mode */
#define DGNSS_TIMEOUT_MS 5000

typedef enum {
  SOLN_MODE_LOW_LATENCY,
  SOLN_MODE_TIME_MATCHED,
  SOLN_MODE_NO_DGNSS
} dgnss_solution_mode_t;

typedef enum {
  FILTER_FLOAT,
  FILTER_FIXED,
} dgnss_filter_t;

typedef struct soln_info_pvt_t {
  u8 num_signals_used;
} soln_info_pvt_t;

typedef struct soln_info_dgnss_t {
  dgnss_filter_t mode;
} soln_info_dgnss_t;

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

/** Maximum time that an observation will be propagated for to align it with a
 * solution epoch before it is discarded.  */
#define OBS_PROPAGATION_LIMIT 10e-3

/* Warn on 15 second base station observation latency */
#define BASE_LATENCY_TIMEOUT 15

/* Make the buffer large enough to handle 15 second latency at 10Hz */
#define STARLING_OBS_N_BUFF BASE_LATENCY_TIMEOUT * 10

/* Totally non-thread-safe access to settings.
 * TODO(kevin) Come up with a better solution.*/
extern dgnss_solution_mode_t dgnss_soln_mode;
extern bool enable_glonass;
extern bool send_heading;
extern double heading_offset;
extern bool disable_klobuchar;
extern float glonass_downweight_factor;
extern double soln_freq_setting;
extern u32 obs_output_divisor;

void reset_rtk_filter(void);
void set_known_ref_pos(const double base_pos[3]);
void set_known_glonass_biases(const glo_biases_t biases);

/*******************************************************************************
 * Formal Starling API
 ******************************************************************************/

/* Initialize starling threads and begin computing a pvt solution. */
void starling_setup(void);
/* Enable fixed RTK mode in the Starling engine. */
void starling_set_enable_fix_mode(bool is_fix_enabled);
/* Indicate for how long corrections should persist. */
void starling_set_max_correction_age(int max_age);

/* Callback type for receiving the position solution messages. */
typedef void (*pos_messages_cb_t)(u8 base_sender_id,
                                  const sbp_messages_t *sbp_messages,
                                  u8 n_meas,
                                  const navigation_measurement_t nav_meas[]);

/* Callback type for receiving PVT solution statistics. */
typedef void (*soln_info_pvt_cb_t)(const soln_info_pvt_t *info);

/* Callback type for receiving DGNSS solution statistics. */
typedef void (*soln_info_dgnss_cb_t)(const soln_info_dgnss_t *info);

/* Callback for receiving the low-latency results. */
typedef void (*filter_output_cb_t)(u8 base_sender_id,
                                   const sbp_messages_t *sbp_messages,
                                   u8 n_meas,
                                   const navigation_measurement_t nav_meas[]);

/* Callback for receiving updated baselines. */
#define SPP_ECEF_SIZE 3
typedef void (*update_baseline_cb_t)(const pvt_engine_result_t *result,
                                     const double spp_ecef[SPP_ECEF_SIZE],
                                     const dops_t *dops,
                                     sbp_messages_t *sbp_messages);

/* NOT THREAD SAFE, JUST DO THESE ONCE AT THE START
 * TODO(kevin) Formalize this part of the API. */
void starling_set_pos_messages_callback(pos_messages_cb_t cb);
void starling_set_soln_info_pvt_callback(soln_info_pvt_cb_t cb);
void starling_set_soln_info_dgnss_callback(soln_info_dgnss_cb_t cb);
void starling_set_low_latency_output_callback(filter_output_cb_t cb);
void starling_set_time_matched_output_callback(filter_output_cb_t cb);
void starling_set_update_baseline_callback(update_baseline_cb_t cb);

#endif
