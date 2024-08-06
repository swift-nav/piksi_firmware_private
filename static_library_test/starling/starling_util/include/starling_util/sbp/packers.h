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

#ifndef STARLING_UTIL_SBP_PACKERS_H
#define STARLING_UTIL_SBP_PACKERS_H

#include <swiftnav/almanac.h>
#include <swiftnav/ephemeris.h>
#include <swiftnav/geoid_model.h>
#include <swiftnav/glonass_phase_biases.h>
#include <swiftnav/nav_meas.h>
#include <swiftnav/single_epoch_solver.h>
#include <optional.hpp>

#include <libsbp/sbp_msg_type.h>
#include <libsbp/v4/gnss.h>
#include <libsbp/v4/navigation.h>
#include <libsbp/v4/observation.h>
#include <libsbp/v4/orientation.h>
#include <libsbp/v4/sbas.h>
#include <libsbp/v4/sbp_msg.h>
#include <libsbp/v4/system.h>

namespace starling {
namespace util {
namespace sbp {

void init_gps_time(sbp_msg_gps_time_t *gps_time, const gps_time_t &t,
                   u8 time_qual);
void init_utc_time(sbp_msg_utc_time_t *utc_time, const gps_time_t &t,
                   const utc_params_t *utc_params, u8 flags);
void init_gps_time_gnss(sbp_msg_gps_time_gnss_t *gps_time, const gps_time_t &t,
                        u8 time_qual);
void init_utc_time_gnss(sbp_msg_utc_time_gnss_t *utc_time, const gps_time_t &t,
                        const utc_params_t *utc_params, u8 flags);
void init_pos_llh(sbp_msg_pos_llh_t *pos_llh, const gps_time_t &t);
void init_pos_ecef(sbp_msg_pos_ecef_t *pos_ecef, const gps_time_t &t);
void init_vel_ned(sbp_msg_vel_ned_t *vel_ned, const gps_time_t &t);
void init_vel_ecef(sbp_msg_vel_ecef_t *vel_ecef, const gps_time_t &t);
void init_sbp_dops(sbp_msg_dops_t *sbp_dops, const gps_time_t &t);
void init_age_corrections(sbp_msg_age_corrections_t *age_corrections,
                          const gps_time_t &t);
void init_dgnss_status(sbp_msg_dgnss_status_t *dgnss_status);
void init_baseline_ecef(sbp_msg_baseline_ecef_t *baseline_ecef,
                        const gps_time_t &t);
void init_baseline_ned(sbp_msg_baseline_ned_t *baseline_ned,
                       const gps_time_t &t);
void init_baseline_heading(sbp_msg_baseline_heading_t *baseline_heading,
                           const gps_time_t &t);
void init_pos_ecef_cov(sbp_msg_pos_ecef_cov_t *pos_ecef_cov,
                       const gps_time_t &t);
void init_vel_ecef_cov(sbp_msg_vel_ecef_cov_t *vel_ecef_cov,
                       const gps_time_t &t);
void init_vel_cog(sbp_msg_vel_cog_t *vel_cog, const gps_time_t &t);
void init_pos_llh_cov(sbp_msg_pos_llh_cov_t *pos_llh_cov, const gps_time_t &t);
void init_pos_llh_acc(sbp_msg_pos_llh_acc_t *pos_llh_acc, const gps_time_t &t);
void init_vel_ned_cov(sbp_msg_vel_ned_cov_t *vel_ned_cov, const gps_time_t &t);
void init_pl(sbp_msg_protection_level_t *pl_msg, const gps_time_t &t);

void init_orient_euler(sbp_msg_orient_euler_t *msg, const gps_time_t &t);

void make_gps_time(sbp_msg_gps_time_t *t_out, const gps_time_t &t_in,
                   u8 time_qual);
void make_gps_time_gnss(sbp_msg_gps_time_gnss_t *t_out, const gps_time_t &t_in,
                        u8 time_qual);
void make_pos_llh_vect(sbp_msg_pos_llh_t *pos_llh, const double llh[3],
                       double h_accuracy, double v_accuracy,
                       const gps_time_t &gps_t, u8 n_sats_used, u8 flags);
void make_pos_llh_cov(sbp_msg_pos_llh_cov_t *pos_llh_cov, const double llh[3],
                      const double llh_cov[6], const gps_time_t &gps_t,
                      u8 n_sats_used, u8 flags);
void make_pos_llh_acc(sbp_msg_pos_llh_acc_t *pos_llh_acc, const double llh[3],
                      double orthometric_height, double h_accuracy,
                      double v_accuracy, double ct_accuracy, double at_accuracy,
                      double h_ellipse_major, double h_ellipse_minor,
                      double h_ellipse_orientation, u8 confidence,
                      const gps_time_t &gps_t, u8 n_sats_used, u8 flags,
                      geoid_model_t geoid_model);
void make_pos_ecef_vect(sbp_msg_pos_ecef_t *pos_ecef, const double ecef[3],
                        double accuracy, const gps_time_t &gps_t,
                        u8 n_sats_used, u8 flags);
void make_pos_ecef_cov(sbp_msg_pos_ecef_cov_t *pos_ecef_cov,
                       const double ecef[3], const double ecef_cov[6],
                       const gps_time_t &gps_t, u8 n_sats_used, u8 flags);
void make_vel_ned(sbp_msg_vel_ned_t *vel_ned, const double v_ned[3],
                  double h_accuracy, double v_accuracy, const gps_time_t &gps_t,
                  u8 n_sats_used, u8 flags);
void make_vel_ned_cov(sbp_msg_vel_ned_cov_t *vel_ned_cov, const double v_ned[3],
                      const double ned_cov[6], const gps_time_t &gps_t,
                      u8 n_sats_used, u8 flags);
void make_vel_ecef(sbp_msg_vel_ecef_t *vel_ecef, const double v_ecef[3],
                   double accuracy, const gps_time_t &gps_t, u8 n_sats_used,
                   u8 flags);
void make_vel_ecef_cov(sbp_msg_vel_ecef_cov_t *vel_ecef_cov,
                       const double v_ecef[3], const double ecef_cov[6],
                       const gps_time_t &gps_t, u8 n_sats_used, u8 flags);

/// \brief Pack a sbp_msg_vel_cog message
///
/// \param msg SBP message (output parameter)
/// \param cog_rad course over ground [rad]
/// \param sog_m_s speed over ground [m/s]
/// \param vel_d_m_s down component of velocity [m/s]
/// \param cog_dev_rad course over ground accuracy [rad]
/// \param sog_dev_m_s speed over ground accuracy [m/s]
/// \param vel_d_dev_m_s down velocity accuracy [m/s]
/// \param flags message flags
void make_vel_cog(sbp_msg_vel_cog_t *msg, const gps_time_t &gps_t,
                  const double cog_rad, const double sog_m_s,
                  const double vel_d_m_s, const double cog_dev_rad,
                  const double sog_dev_m_s, const double vel_d_dev_m_s,
                  const u8 flags);

void make_dops(sbp_msg_dops_t *dops_out, const dops_t &dops_in, u32 tow,
               u8 flags);
void make_baseline_ecef(sbp_msg_baseline_ecef_t *baseline_ecef,
                        const gps_time_t &t, u8 n_sats, const double b_ecef[3],
                        double accuracy, u8 flags);
void make_baseline_ned(sbp_msg_baseline_ned_t *baseline_ned,
                       const gps_time_t &t, u8 n_sats, const double b_ned[3],
                       double h_accuracy, double v_accuracy, u8 flags);

void make_pl(sbp_msg_protection_level_t *pl_msg, const gps_time_t &t,
             double vpl, double hpl, const double llh[3], u8 flags);

void make_heading(sbp_msg_baseline_heading_t *baseline_heading,
                  const gps_time_t &t, const double heading, u8 n_sats,
                  u8 flags);
void make_age_corrections(sbp_msg_age_corrections_t *age_corrections,
                          const gps_time_t &t, double propagation_time);
void make_ins_status(sbp_msg_ins_status_t *msg_ins_status, const u32 flags);
void make_dgnss_status(sbp_msg_dgnss_status_t *dgnss_status, u8 num_sats,
                       double obs_latency, u8 flags);
void make_utc_time(sbp_msg_utc_time_t *t_out, const gps_time_t &t_in,
                   const utc_params_t *utc_params, u8 flags);
void make_utc_time_gnss(sbp_msg_utc_time_gnss_t *t_out, const gps_time_t &t_in,
                        const utc_params_t *utc_params, u8 flags);

void make_orient_euler(sbp_msg_orient_euler_t *msg, const gps_time_t &t,
                       double yaw, double pit, double rol, double yaw_dev,
                       double pit_dev, double rol_dev);

void pack_obs_header(const gps_time_t &t, u8 total, u8 count,
                     sbp_observation_header_t *msg);

u8 nm_flags_to_sbp(nav_meas_flags_t from);

s8 pack_obs_content(const navigation_measurement_t &meas,
                    sbp_packed_obs_content_t *msg);

s8 pack_osr_content(const navigation_measurement_t &meas,
                    const measurement_std_t &meas_std,
                    sbp_packed_osr_content_t *msg);

std::experimental::optional<sbp_msg_type_t> pack_ephemeris(const ephemeris_t &e,
                                                           sbp_msg_t *msg);

std::experimental::optional<sbp_msg_type_t> pack_almanac(const almanac_t &a,
                                                         sbp_msg_t *msg);

void pack_sbas_raw_data(const gnss_signal_t sid, u32 tow_ms, u8 msg,
                        const u8 *decoded, sbp_msg_sbas_raw_t *sbas_raw_msg);

void pack_glonass_biases_content(glo_biases_t glonass_biases,
                                 sbp_msg_glo_biases_t *msg);

}  // namespace sbp
}  // namespace util
}  // namespace starling

#endif
