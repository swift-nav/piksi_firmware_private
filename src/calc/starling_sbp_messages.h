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

#ifndef STARLING_SBP_MESSAGES_H_
#define STARLING_SBP_MESSAGES_H_

#include <libsbp/common.h>
#include <libsbp/gnss.h>
#include <libsbp/navigation.h>
#include <libsbp/ndb.h>
#include <libsbp/observation.h>
#include <libsbp/orientation.h>
#include <libsbp/system.h>
#include <starling/starling.h>
#include <swiftnav/almanac.h>
#include <swiftnav/ephemeris.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/sbas_raw_data.h>
#include <swiftnav/signal.h>
#include <swiftnav/single_epoch_solver.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Set of messages sent by the Piksi Multi integration of Starling. */
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

void sbp_init_gps_time(msg_gps_time_t *gps_time, gps_time_t *t, u8 time_qual);
void sbp_init_utc_time(msg_utc_time_t *utc_time, gps_time_t *t, u8 time_qual);
void sbp_init_pos_llh(msg_pos_llh_t *pos_llh, gps_time_t *t);
void sbp_init_pos_ecef(msg_pos_ecef_t *pos_ecef, gps_time_t *t);
void sbp_init_vel_ned(msg_vel_ned_t *vel_ned, gps_time_t *t);
void sbp_init_vel_ecef(msg_vel_ecef_t *vel_ecef, gps_time_t *t);
void sbp_init_pos_ecef_cov(msg_pos_ecef_cov_t *pos_ecef_cov, gps_time_t *t);
void sbp_init_vel_ecef_cov(msg_vel_ecef_cov_t *vel_ecef_cov, gps_time_t *t);
void sbp_init_pos_llh_cov(msg_pos_llh_cov_t *pos_llh_cov, gps_time_t *t);
void sbp_init_vel_ned_cov(msg_vel_ned_cov_t *vel_ned_cov, gps_time_t *t);
void sbp_init_baseline_ecef(msg_baseline_ecef_t *baseline_ecef, gps_time_t *t);
void sbp_init_baseline_ned(msg_baseline_ned_t *baseline_ned, gps_time_t *t);
void sbp_init_baseline_heading(msg_baseline_heading_t *baseline_heading,
                               gps_time_t *t);
void sbp_init_sbp_dops(msg_dops_t *sbp_dops, gps_time_t *t);
void sbp_init_age_corrections(msg_age_corrections_t *age_corrections,
                              gps_time_t *t);
void sbp_init_dgnss_status(msg_dgnss_status_t *dgnss_status);

void sbp_make_pos_llh_vect(msg_pos_llh_t *pos_llh,
                           const double llh[3],
                           double h_accuracy,
                           double v_accuracy,
                           const gps_time_t *gps_t,
                           u8 n_sats_used,
                           u8 flags);
void sbp_make_pos_llh_cov(msg_pos_llh_cov_t *pos_llh_cov,
                          const double llh[3],
                          const double llh_cov[6],
                          const gps_time_t *gps_t,
                          u8 n_sats_used,
                          u8 flags);
void sbp_make_pos_ecef_vect(msg_pos_ecef_t *pos_ecef,
                            const double ecef[3],
                            double accuracy,
                            const gps_time_t *gps_t,
                            u8 n_sats_used,
                            u8 flags);
void sbp_make_pos_ecef_cov(msg_pos_ecef_cov_t *pos_ecef_cov,
                           const double ecef[3],
                           const double ecef_cov[6],
                           const gps_time_t *gps_t,
                           u8 n_sats_used,
                           u8 flags);
void sbp_make_vel_ned(msg_vel_ned_t *vel_ned,
                      const double v_ned[3],
                      double h_accuracy,
                      double v_accuracy,
                      const gps_time_t *gps_t,
                      u8 n_sats_used,
                      u8 flags);
void sbp_make_vel_ned_cov(msg_vel_ned_cov_t *vel_ned_cov,
                          const double v_ned[3],
                          const double ned_cov[6],
                          const gps_time_t *gps_t,
                          u8 n_sats_used,
                          u8 flags);
void sbp_make_vel_ecef(msg_vel_ecef_t *vel_ecef,
                       const double v_ecef[3],
                       double accuracy,
                       const gps_time_t *gps_t,
                       u8 n_sats_used,
                       u8 flags);
void sbp_make_vel_ecef_cov(msg_vel_ecef_cov_t *vel_ecef_cov,
                           const double v_ecef[3],
                           const double ecef_cov[6],
                           const gps_time_t *gps_t,
                           u8 n_sats_used,
                           u8 flags);
void sbp_make_baseline_ecef(msg_baseline_ecef_t *baseline_ecef,
                            const gps_time_t *t,
                            u8 n_sats,
                            const double b_ecef[3],
                            double accuracy,
                            u8 flags);
void sbp_make_baseline_ned(msg_baseline_ned_t *baseline_ned,
                           const gps_time_t *t,
                           u8 n_sats,
                           const double b_ned[3],
                           double h_accuracy,
                           double v_accuracy,
                           u8 flags);
void sbp_make_heading(msg_baseline_heading_t *baseline_heading,
                      const gps_time_t *t,
                      const double heading,
                      u8 n_sats_used,
                      u8 flags);

void sbp_make_dops(msg_dops_t *dops_out,
                   const dops_t *dops_in,
                   u32 tow,
                   u8 flags);

void sbp_make_age_corrections(msg_age_corrections_t *age_corrections,
                              const gps_time_t *t,
                              double propagation_time);

void sbp_make_dgnss_status(msg_dgnss_status_t *dgnss_status,
u8 num_sats,
double obs_latency,
u8 flags);

void sbp_make_gps_time(msg_gps_time_t *t_out,
                       const gps_time_t *t_in,
                       time_quality_t time_qual);
void sbp_make_utc_time(msg_utc_time_t *t_out,
                       const gps_time_t *t_in,
                       time_quality_t time_qual);

/*******************************************************************************/

void sbp_messages_init(sbp_messages_t *sbp_messages,
                       const gps_time_t *epoch_time,
                       time_quality_t time_qual); 


#ifdef __cplusplus
}
#endif

#endif
