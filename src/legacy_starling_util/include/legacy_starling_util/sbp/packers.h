/**
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LEGACY_STARLING_UTIL_SBP_PACKERS_H
#define LEGACY_STARLING_UTIL_SBP_PACKERS_H

#include <legacy_starling_util/sbp/messages.h>
#include <swiftnav/almanac.h>
#include <swiftnav/ephemeris.h>
#include <swiftnav/geoid_model.h>
#include <swiftnav/glonass_phase_biases.h>
#include <swiftnav/nav_meas.h>
#include <swiftnav/single_epoch_solver.h>

#ifdef __cplusplus
extern "C" {
#endif

void sbp_init_gps_time(msg_gps_time_t *gps_time,
                       const gps_time_t *t,
                       u8 time_qual);
void sbp_init_utc_time(msg_utc_time_t *utc_time,
                       const gps_time_t *t,
                       const utc_params_t *utc_params,
                       u8 flags);
void sbp_init_gps_time_gnss(msg_gps_time_gnss_t *gps_time,
                            const gps_time_t *t,
                            u8 time_qual);
void sbp_init_utc_time_gnss(msg_utc_time_gnss_t *utc_time,
                            const gps_time_t *t,
                            const utc_params_t *utc_params,
                            u8 flags);
void sbp_init_pos_llh(msg_pos_llh_t *pos_llh, const gps_time_t *t);
void sbp_init_pos_ecef(msg_pos_ecef_t *pos_ecef, const gps_time_t *t);
void sbp_init_vel_ned(msg_vel_ned_t *vel_ned, const gps_time_t *t);
void sbp_init_vel_ecef(msg_vel_ecef_t *vel_ecef, const gps_time_t *t);
void sbp_init_sbp_dops(msg_dops_t *sbp_dops, const gps_time_t *t);
void sbp_init_age_corrections(msg_age_corrections_t *age_corrections,
                              const gps_time_t *t);
void sbp_init_dgnss_status(msg_dgnss_status_t *dgnss_status);
void sbp_init_baseline_ecef(msg_baseline_ecef_t *baseline_ecef,
                            const gps_time_t *t);
void sbp_init_baseline_ned(msg_baseline_ned_t *baseline_ned,
                           const gps_time_t *t);
void sbp_init_baseline_heading(msg_baseline_heading_t *baseline_heading,
                               const gps_time_t *t);
void sbp_init_pos_ecef_cov(msg_pos_ecef_cov_t *pos_ecef_cov,
                           const gps_time_t *t);
void sbp_init_vel_ecef_cov(msg_vel_ecef_cov_t *vel_ecef_cov,
                           const gps_time_t *t);
void sbp_init_pos_llh_cov(msg_pos_llh_cov_t *pos_llh_cov, const gps_time_t *t);
void sbp_init_pos_llh_acc(msg_pos_llh_acc_t *pos_llh_acc, const gps_time_t *t);
void sbp_init_vel_ned_cov(msg_vel_ned_cov_t *vel_ned_cov, const gps_time_t *t);
void sbp_init_pl(msg_protection_level_t *pl_msg, const gps_time_t *t);

void sbp_init_orient_euler(msg_orient_euler_t *msg, const gps_time_t *t);

void sbp_make_gps_time(msg_gps_time_t *t_out,
                       const gps_time_t *t_in,
                       u8 time_qual);
void sbp_make_gps_time_gnss(msg_gps_time_gnss_t *t_out,
                            const gps_time_t *t_in,
                            u8 time_qual);
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
void sbp_make_pos_llh_acc(msg_pos_llh_acc_t *pos_llh_acc,
                          const double llh[3],
                          double orthometric_height,
                          double h_accuracy,
                          double v_accuracy,
                          double ct_accuracy,
                          double at_accuracy,
                          double h_ellipse_major,
                          double h_ellipse_minor,
                          double h_ellipse_orientation,
                          u8 confidence,
                          const gps_time_t *gps_t,
                          u8 n_sats_used,
                          u8 flags,
                          geoid_model_t geoid_model);
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
void sbp_make_dops(msg_dops_t *dops_out,
                   const dops_t *dops_in,
                   u32 tow,
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

void sbp_make_pl(msg_protection_level_t *pl_msg,
                 const gps_time_t *t,
                 double vpl,
                 double hpl,
                 const double llh[3],
                 u8 flags);

void sbp_make_heading(msg_baseline_heading_t *baseline_heading,
                      const gps_time_t *t,
                      double heading,
                      u8 n_sats,
                      u8 flags);
void sbp_make_age_corrections(msg_age_corrections_t *age_corrections,
                              const gps_time_t *t,
                              double propagation_time);
void sbp_make_ins_status(msg_ins_status_t *msg_ins_status, u32 flags);
void sbp_make_dgnss_status(msg_dgnss_status_t *dgnss_status,
                           u8 num_sats,
                           double obs_latency,
                           u8 flags);
void sbp_make_utc_time(msg_utc_time_t *t_out,
                       const gps_time_t *t_in,
                       const utc_params_t *utc_params,
                       u8 flags);
void sbp_make_utc_time_gnss(msg_utc_time_gnss_t *t_out,
                            const gps_time_t *t_in,
                            const utc_params_t *utc_params,
                            u8 flags);

void sbp_make_orient_euler(msg_orient_euler_t *msg,
                           const gps_time_t *t,
                           double yaw,
                           double pit,
                           double rol,
                           double yaw_dev,
                           double pit_dev,
                           double rol_dev);

void pack_obs_header(const gps_time_t *t,
                     u8 total,
                     u8 count,
                     observation_header_t *msg);

u8 nm_flags_to_sbp(nav_meas_flags_t from);

s8 pack_obs_content(const navigation_measurement_t *meas,
                    packed_obs_content_t *msg);

s8 pack_osr_content(const navigation_measurement_t *meas,
                    const measurement_std_t *meas_std,
                    packed_osr_content_t *msg);

msg_info_t pack_ephemeris(const ephemeris_t *e, msg_ephemeris_t *msg);

msg_info_t pack_almanac(const almanac_t *a, msg_almanac_t *msg);

void sbp_pack_sbas_raw_data(gnss_signal_t sid,
                            u32 tow_ms,
                            u8 msg,
                            const u8 *decoded,
                            msg_sbas_raw_t *sbas_raw_msg);

void sbp_pack_glonass_biases_content(glo_biases_t glonass_biases,
                                     msg_glo_biases_t *msg);

#ifdef __cplusplus
}
#endif

#endif  // LEGACY_STARLING_UTIL_SBP_PACKERS_H
