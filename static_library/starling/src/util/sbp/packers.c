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

#include "starling/util/sbp/packers.h"

#include <assert.h>
#include <starling/util/sbp/misc.h>
#include <string.h>
#include <swiftnav/bits.h>
#include <swiftnav/pvt_result.h>

void sbp_init_gps_time(msg_gps_time_t *gps_time, const gps_time_t *t,
                       u8 time_qual) {
  memset(gps_time, 0, sizeof(msg_gps_time_t));
  sbp_make_gps_time(gps_time, t, time_qual);
}

void sbp_init_utc_time(msg_utc_time_t *utc_time, const gps_time_t *t,
                       const utc_params_t *utc_params, u8 flags) {
  memset(utc_time, 0, sizeof(msg_utc_time_t));
  sbp_make_utc_time(utc_time, t, utc_params, flags);
}

void sbp_init_pos_llh(msg_pos_llh_t *pos_llh, const gps_time_t *t) {
  memset(pos_llh, 0, sizeof(msg_pos_llh_t));
  if (gps_time_valid(t)) {
    pos_llh->tow = round_tow_ms(t->tow);
  }
}

void sbp_init_pos_ecef(msg_pos_ecef_t *pos_ecef, const gps_time_t *t) {
  memset(pos_ecef, 0, sizeof(msg_pos_ecef_t));
  if (gps_time_valid(t)) {
    pos_ecef->tow = round_tow_ms(t->tow);
  }
}

void sbp_init_vel_ned(msg_vel_ned_t *vel_ned, const gps_time_t *t) {
  memset(vel_ned, 0, sizeof(msg_vel_ned_t));
  if (gps_time_valid(t)) {
    vel_ned->tow = round_tow_ms(t->tow);
  }
}

void sbp_init_vel_ecef(msg_vel_ecef_t *vel_ecef, const gps_time_t *t) {
  memset(vel_ecef, 0, sizeof(msg_vel_ecef_t));
  if (gps_time_valid(t)) {
    vel_ecef->tow = round_tow_ms(t->tow);
  }
}

void sbp_init_sbp_dops(msg_dops_t *sbp_dops, const gps_time_t *t) {
  memset(sbp_dops, 0, sizeof(msg_dops_t));
  if (gps_time_valid(t)) {
    sbp_dops->tow = round_tow_ms(t->tow);
  }
}

void sbp_init_age_corrections(msg_age_corrections_t *age_corrections,
                              const gps_time_t *t) {
  memset(age_corrections, 0, sizeof(msg_age_corrections_t));
  age_corrections->age = 0xFFFF;
  if (gps_time_valid(t)) {
    age_corrections->tow = round_tow_ms(t->tow);
  }
}

void sbp_init_dgnss_status(msg_dgnss_status_t *dgnss_status) {
  memset(dgnss_status, 0, sizeof(msg_dgnss_status_t));
}

void sbp_init_baseline_ecef(msg_baseline_ecef_t *baseline_ecef,
                            const gps_time_t *t) {
  memset(baseline_ecef, 0, sizeof(msg_baseline_ecef_t));
  if (gps_time_valid(t)) {
    baseline_ecef->tow = round_tow_ms(t->tow);
  }
}

void sbp_init_baseline_ned(msg_baseline_ned_t *baseline_ned,
                           const gps_time_t *t) {
  memset(baseline_ned, 0, sizeof(msg_baseline_ned_t));
  if (gps_time_valid(t)) {
    baseline_ned->tow = round_tow_ms(t->tow);
  }
}

void sbp_init_baseline_heading(msg_baseline_heading_t *baseline_heading,
                               const gps_time_t *t) {
  memset(baseline_heading, 0, sizeof(msg_baseline_heading_t));
  if (gps_time_valid(t)) {
    baseline_heading->tow = round_tow_ms(t->tow);
  }
}

void sbp_init_pos_ecef_cov(msg_pos_ecef_cov_t *pos_ecef_cov,
                           const gps_time_t *t) {
  memset(pos_ecef_cov, 0, sizeof(msg_pos_ecef_cov_t));
  if (gps_time_valid(t)) {
    pos_ecef_cov->tow = round_tow_ms(t->tow);
  }
}

void sbp_init_vel_ecef_cov(msg_vel_ecef_cov_t *vel_ecef_cov,
                           const gps_time_t *t) {
  memset(vel_ecef_cov, 0, sizeof(msg_vel_ecef_cov_t));
  if (gps_time_valid(t)) {
    vel_ecef_cov->tow = round_tow_ms(t->tow);
  }
}

void sbp_init_pos_llh_cov(msg_pos_llh_cov_t *pos_llh_cov, const gps_time_t *t) {
  memset(pos_llh_cov, 0, sizeof(msg_pos_llh_cov_t));
  if (gps_time_valid(t)) {
    pos_llh_cov->tow = round_tow_ms(t->tow);
  }
}

void sbp_init_vel_ned_cov(msg_vel_ned_cov_t *vel_ned_cov, const gps_time_t *t) {
  memset(vel_ned_cov, 0, sizeof(msg_vel_ned_cov_t));
  if (gps_time_valid(t)) {
    vel_ned_cov->tow = round_tow_ms(t->tow);
  }
}

void sbp_init_pl(msg_protection_level_t *pl_msg, const gps_time_t *t) {
  memset(pl_msg, 0, sizeof(msg_protection_level_t));
  if (gps_time_valid(t)) {
    pl_msg->tow = round_tow_ms(t->tow);
  }
}

void sbp_make_gps_time(msg_gps_time_t *t_out, const gps_time_t *t_in,
                       u8 time_qual) {
  if (!gps_time_valid(t_in)) {
    memset(t_out, 0, sizeof(msg_gps_time_t));
    return;
  }
  /* TODO(Leith): SBP message should reuse the GPSTimeNano struct */
  sbp_gps_time_t t_nano;
  round_time_nano(t_in, &t_nano);
  t_out->wn = t_nano.wn;
  t_out->tow = t_nano.tow;
  t_out->ns_residual = t_nano.ns_residual;
  t_out->flags = (u8)(sbp_get_time_quality_flags(time_qual) & 0x7);
}

void sbp_make_pos_llh_vect(msg_pos_llh_t *pos_llh, const double llh[3],
                           double h_accuracy, double v_accuracy,
                           const gps_time_t *gps_t, u8 n_sats_used, u8 flags) {
  pos_llh->tow = round_tow_ms(gps_t->tow);
  pos_llh->lat = llh[0] * R2D;
  pos_llh->lon = llh[1] * R2D;
  pos_llh->height = llh[2];
  pos_llh->h_accuracy = MIN(round(1e3 * h_accuracy), UINT16_MAX);
  pos_llh->v_accuracy = MIN(round(1e3 * v_accuracy), UINT16_MAX);
  pos_llh->n_sats = n_sats_used;
  pos_llh->flags = flags;
}

void sbp_make_pos_llh_cov(msg_pos_llh_cov_t *pos_llh_cov, const double llh[3],
                          const double llh_cov[6], const gps_time_t *gps_t,
                          u8 n_sats_used, u8 flags) {
  pos_llh_cov->tow = round_tow_ms(gps_t->tow);
  pos_llh_cov->lat = llh[0] * R2D;
  pos_llh_cov->lon = llh[1] * R2D;
  pos_llh_cov->height = llh[2];
  pos_llh_cov->cov_n_n = llh_cov[0];
  pos_llh_cov->cov_n_e = llh_cov[1];
  pos_llh_cov->cov_n_d = llh_cov[3];
  pos_llh_cov->cov_e_e = llh_cov[2];
  pos_llh_cov->cov_e_d = llh_cov[4];
  pos_llh_cov->cov_d_d = llh_cov[5];
  pos_llh_cov->n_sats = n_sats_used;
  pos_llh_cov->flags = flags;
}

void sbp_make_pos_ecef_vect(msg_pos_ecef_t *pos_ecef, const double ecef[3],
                            double accuracy, const gps_time_t *gps_t,
                            u8 n_sats_used, u8 flags) {
  pos_ecef->tow = round_tow_ms(gps_t->tow);
  pos_ecef->x = ecef[0];
  pos_ecef->y = ecef[1];
  pos_ecef->z = ecef[2];
  pos_ecef->accuracy = MIN(round(1e3 * accuracy), UINT16_MAX);
  pos_ecef->n_sats = n_sats_used;
  pos_ecef->flags = flags;
}

void sbp_make_pos_ecef_cov(msg_pos_ecef_cov_t *pos_ecef_cov,
                           const double ecef[3], const double ecef_cov[6],
                           const gps_time_t *gps_t, u8 n_sats_used, u8 flags) {
  pos_ecef_cov->tow = round_tow_ms(gps_t->tow);
  pos_ecef_cov->x = ecef[0];
  pos_ecef_cov->y = ecef[1];
  pos_ecef_cov->z = ecef[2];
  pos_ecef_cov->cov_x_x = ecef_cov[0];
  pos_ecef_cov->cov_x_y = ecef_cov[1];
  pos_ecef_cov->cov_x_z = ecef_cov[3];
  pos_ecef_cov->cov_y_y = ecef_cov[2];
  pos_ecef_cov->cov_y_z = ecef_cov[4];
  pos_ecef_cov->cov_z_z = ecef_cov[5];
  pos_ecef_cov->n_sats = n_sats_used;
  pos_ecef_cov->flags = flags;
}

void sbp_make_vel_ned(msg_vel_ned_t *vel_ned, const double v_ned[3],
                      double h_accuracy, double v_accuracy,
                      const gps_time_t *gps_t, u8 n_sats_used, u8 flags) {
  vel_ned->tow = round_tow_ms(gps_t->tow);
  vel_ned->n = round(v_ned[0] * 1e3);
  vel_ned->e = round(v_ned[1] * 1e3);
  vel_ned->d = round(v_ned[2] * 1e3);
  vel_ned->h_accuracy = MIN(round(1e3 * h_accuracy), UINT16_MAX);
  vel_ned->v_accuracy = MIN(round(1e3 * v_accuracy), UINT16_MAX);
  vel_ned->n_sats = n_sats_used;
  vel_ned->flags = flags;
}

void sbp_make_vel_ned_cov(msg_vel_ned_cov_t *vel_ned_cov, const double v_ned[3],
                          const double ned_cov[6], const gps_time_t *gps_t,
                          u8 n_sats_used, u8 flags) {
  vel_ned_cov->tow = round_tow_ms(gps_t->tow);
  vel_ned_cov->n = round(v_ned[0] * 1e3);
  vel_ned_cov->e = round(v_ned[1] * 1e3);
  vel_ned_cov->d = round(v_ned[2] * 1e3);
  vel_ned_cov->cov_n_n = ned_cov[0];
  vel_ned_cov->cov_n_e = ned_cov[1];
  vel_ned_cov->cov_n_d = ned_cov[3];
  vel_ned_cov->cov_e_e = ned_cov[2];
  vel_ned_cov->cov_e_d = ned_cov[4];
  vel_ned_cov->cov_d_d = ned_cov[5];
  vel_ned_cov->n_sats = n_sats_used;
  vel_ned_cov->flags = flags;
}

void sbp_make_vel_ecef(msg_vel_ecef_t *vel_ecef, const double v_ecef[3],
                       double accuracy, const gps_time_t *gps_t, u8 n_sats_used,
                       u8 flags) {
  vel_ecef->tow = round_tow_ms(gps_t->tow);
  vel_ecef->x = round(v_ecef[0] * 1e3);
  vel_ecef->y = round(v_ecef[1] * 1e3);
  vel_ecef->z = round(v_ecef[2] * 1e3);
  vel_ecef->accuracy = MIN(round(1e3 * accuracy), UINT16_MAX);
  vel_ecef->n_sats = n_sats_used;
  vel_ecef->flags = flags;
}

void sbp_make_vel_ecef_cov(msg_vel_ecef_cov_t *vel_ecef_cov,
                           const double v_ecef[3], const double ecef_cov[6],
                           const gps_time_t *gps_t, u8 n_sats_used, u8 flags) {
  vel_ecef_cov->tow = round_tow_ms(gps_t->tow);
  vel_ecef_cov->x = round(v_ecef[0] * 1e3);
  vel_ecef_cov->y = round(v_ecef[1] * 1e3);
  vel_ecef_cov->z = round(v_ecef[2] * 1e3);
  vel_ecef_cov->cov_x_x = ecef_cov[0];
  vel_ecef_cov->cov_x_y = ecef_cov[1];
  vel_ecef_cov->cov_x_z = ecef_cov[3];
  vel_ecef_cov->cov_y_y = ecef_cov[2];
  vel_ecef_cov->cov_y_z = ecef_cov[4];
  vel_ecef_cov->cov_z_z = ecef_cov[5];
  vel_ecef_cov->n_sats = n_sats_used;
  vel_ecef_cov->flags = flags;
}

void sbp_make_baseline_ecef(msg_baseline_ecef_t *baseline_ecef,
                            const gps_time_t *t, u8 n_sats,
                            const double b_ecef[3], double accuracy, u8 flags) {
  baseline_ecef->tow = round_tow_ms(t->tow);
  baseline_ecef->x = round(1e3 * b_ecef[0]);
  baseline_ecef->y = round(1e3 * b_ecef[1]);
  baseline_ecef->z = round(1e3 * b_ecef[2]);
  baseline_ecef->accuracy = MIN(round(1e3 * accuracy), UINT16_MAX);
  baseline_ecef->n_sats = n_sats;
  baseline_ecef->flags = flags;
}

void sbp_make_baseline_ned(msg_baseline_ned_t *baseline_ned,
                           const gps_time_t *t, u8 n_sats,
                           const double b_ned[3], double h_accuracy,
                           double v_accuracy, u8 flags) {
  baseline_ned->tow = round_tow_ms(t->tow);
  baseline_ned->n = round(1e3 * b_ned[0]);
  baseline_ned->e = round(1e3 * b_ned[1]);
  baseline_ned->d = round(1e3 * b_ned[2]);
  baseline_ned->h_accuracy = MIN(round(1e3 * h_accuracy), UINT16_MAX);
  baseline_ned->v_accuracy = MIN(round(1e3 * v_accuracy), UINT16_MAX);
  baseline_ned->n_sats = n_sats;
  baseline_ned->flags = flags;
}

void sbp_make_pl(msg_protection_level_t *pl_msg, const gps_time_t *t,
                 double vpl, double hpl, const double llh[3], u8 flags) {
  pl_msg->tow = round_tow_ms(t->tow);
  pl_msg->vpl = MIN(round(1e2 * vpl), UINT16_MAX);
  pl_msg->hpl = MIN(round(1e2 * hpl), UINT16_MAX);
  pl_msg->lat = llh[0] * R2D;
  pl_msg->lon = llh[1] * R2D;
  pl_msg->height = llh[2];
  pl_msg->flags = flags;
}

static double constrain_angle(const double heading) {
  double constrained_heading = fmod(heading, 360.0);
  if (constrained_heading < 0) {
    constrained_heading += 360.0;
  }
  return constrained_heading;
}

void sbp_make_heading(msg_baseline_heading_t *baseline_heading,
                      const gps_time_t *t, const double heading, u8 n_sats,
                      u8 flags) {
  baseline_heading->tow = round_tow_ms(t->tow);
  baseline_heading->heading =
      round(constrain_angle(heading) * MSG_HEADING_SCALE_FACTOR);
  baseline_heading->n_sats = n_sats;
  baseline_heading->flags = flags;
}

void sbp_make_dops(msg_dops_t *dops_out, const dops_t *dops_in, const u32 tow,
                   u8 flags) {
  dops_out->tow = tow;
  dops_out->pdop = round(dops_in->pdop * 100);
  dops_out->gdop = round(dops_in->gdop * 100);
  dops_out->tdop = round(dops_in->tdop * 100);
  dops_out->hdop = round(dops_in->hdop * 100);
  dops_out->vdop = round(dops_in->vdop * 100);
  dops_out->flags = flags;
}

void sbp_make_age_corrections(msg_age_corrections_t *age_corrections,
                              const gps_time_t *t, double propagation_time) {
  age_corrections->tow = round_tow_ms(t->tow);
  age_corrections->age = MIN(round(10 * propagation_time), UINT16_MAX);
}

static utc_tm gps2utc_nano(const gps_time_t *t_in,
                           const utc_params_t *p_utc_params) {
  /* convert to UTC */
  utc_tm utc_time;
  gps2utc(t_in, &utc_time, p_utc_params);

  /* If the nanosecond part of the UTC timestamp rounds up to the next second,
   * recompute the UTC time structure to roll over all fields properly, also
   * accounting for possible leap second event */
  if (round(utc_time.second_frac * SECS_NS) == SECS_NS) {
    gps_time_t t_tmp = *t_in;
    double dt = 1.0 - utc_time.second_frac;
    /* round up to the next representable floating point number */
    t_tmp.tow = nextafter(t_tmp.tow + dt, INFINITY);
    normalize_gps_time(&t_tmp);
    /* recompute UTC time for proper rounding */
    gps2utc(&t_tmp, &utc_time, p_utc_params);
  }
  return utc_time;
}

void sbp_make_utc_time(msg_utc_time_t *t_out, const gps_time_t *t_in,
                       const utc_params_t *utc_params, u8 flags) {
  if (!gps_time_valid(t_in)) {
    memset(t_out, 0, sizeof(msg_utc_time_t));
    return;
  }

  /* convert to UTC with nanosecond precision (falls back to a hard-coded table
   * if the pointer is null) */
  utc_tm utc_time = gps2utc_nano(t_in, utc_params);

  t_out->tow = round_tow_ms(t_in->tow);
  t_out->year = utc_time.year;
  t_out->month = utc_time.month;
  t_out->day = utc_time.month_day;
  t_out->hours = utc_time.hour;
  t_out->minutes = utc_time.minute;
  t_out->seconds = utc_time.second_int;
  /* note: utc_time has been rounded above to guarantee that this does not roll
   * over to the next second */
  t_out->ns = round(utc_time.second_frac * SECS_NS);
  assert(t_out->ns < SECS_NS);
  t_out->flags = flags;
}

void sbp_make_ins_status(msg_ins_status_t *msg_ins_status, const u32 flags) {
  u32 flags_mode = (flags & INS_MODE_MASK);
  assert(flags_mode == INS_STATUS_AWAITING_INIT ||
         flags_mode == INS_STATUS_ALIGNING || flags_mode == INS_STATUS_READY ||
         flags_mode == INS_STATUS_OUTAGE_EXCEEDS_MAX);
  msg_ins_status->flags = flags;
}

void sbp_make_dgnss_status(msg_dgnss_status_t *dgnss_status, u8 num_sats,
                           double obs_latency, u8 flags) {
  if (flags > POSITION_MODE_DGNSS) {
    dgnss_status->flags = 2;
  } else {
    dgnss_status->flags = 1;
  }
  dgnss_status->latency = MIN(lround(10 * obs_latency), UINT16_MAX);
  dgnss_status->num_signals = num_sats;
}

void pack_obs_header(const gps_time_t *t, u8 total, u8 count,
                     observation_header_t *msg) {
  if (gps_time_valid(t)) {
    round_time_nano(t, &msg->t);
  } else {
    memset(&msg->t, 0, sizeof(sbp_gps_time_t));
  }
  msg->n_obs =
      ((total << MSG_OBS_HEADER_SEQ_SHIFT) | (count & MSG_OBS_HEADER_SEQ_MASK));
}

u8 nm_flags_to_sbp(nav_meas_flags_t from) {
  u8 to = 0;
  if (0 != (from & NAV_MEAS_FLAG_CODE_VALID)) {
    to |= MSG_OBS_FLAGS_CODE_VALID;
  }
  if (0 != (from & NAV_MEAS_FLAG_PHASE_VALID)) {
    to |= MSG_OBS_FLAGS_PHASE_VALID;
  }
  if (0 != (from & NAV_MEAS_FLAG_HALF_CYCLE_KNOWN)) {
    to |= MSG_OBS_FLAGS_HALF_CYCLE_KNOWN;
  }
  if (0 != (from & NAV_MEAS_FLAG_MEAS_DOPPLER_VALID)) {
    to |= MSG_OBS_FLAGS_MEAS_DOPPLER_VALID;
  }
  if (0 != (from & NAV_MEAS_FLAG_RAIM_EXCLUSION)) {
    to |= MSG_OBS_FLAGS_RAIM_EXCLUSION;
  }
  return to;
}

static bool pack_pseudorange(double P_in, nav_meas_flags_t flags, u32 *P_out) {
  if (0 != (flags & NAV_MEAS_FLAG_CODE_VALID)) {
    s64 P_fp = llround(P_in * MSG_OBS_P_MULTIPLIER);
    if (P_in < 0 || P_fp > UINT32_MAX) {
      return false;
    }
    *P_out = P_fp;
  } else {
    *P_out = 0;
  }

  return true;
}

static bool pack_carrier_phase(double L_in, nav_meas_flags_t flags,
                               carrier_phase_t *L_out) {
  if (0 != (flags & NAV_MEAS_FLAG_PHASE_VALID)) {
    double Li = floor(L_in);
    if (Li < INT32_MIN || Li > INT32_MAX) {
      return false;
    }

    double Lf = L_in - Li;

    L_out->i = Li;
    u16 frac_part_cp = round(Lf * MSG_OBS_LF_MULTIPLIER);
    if (frac_part_cp >= MSG_OBS_LF_OVERFLOW) {
      frac_part_cp = 0;
      L_out->i += 1;
    }
    L_out->f = frac_part_cp;
  } else {
    L_out->i = 0;
    L_out->f = 0;
  }

  return true;
}

static bool pack_doppler(double D_in, nav_meas_flags_t flags,
                         doppler_t *D_out) {
  if (0 != (flags & NAV_MEAS_FLAG_MEAS_DOPPLER_VALID)) {
    double Di = floor(D_in);
    if (Di < INT16_MIN || Di > INT16_MAX) {
      return false;
    }

    double Df = D_in - Di;

    D_out->i = (s16)Di;
    u16 frac_part_d = round(Df * MSG_OBS_DF_MULTIPLIER);
    if (frac_part_d >= MSG_OBS_DF_OVERFLOW) {
      frac_part_d = 0;
      D_out->i += 1;
    }
    D_out->f = frac_part_d;
  } else {
    D_out->i = 0;
    D_out->f = 0;
  }

  return true;
}

static bool pack_cn0(double cn0_in, nav_meas_flags_t flags, u8 *cn0_out) {
  if (0 != (flags & NAV_MEAS_FLAG_CN0_VALID)) {
    s32 cn0_fp = lround(cn0_in * MSG_OBS_CN0_MULTIPLIER);
    if (cn0_in < 0 || cn0_fp > UINT8_MAX) {
      return false;
    }

    *cn0_out = cn0_fp;
  } else {
    *cn0_out = 0;
  }

  return true;
}

/** Pack GPS observables into a `msg_obs_content_t` struct.
 * For use in constructing a `MSG_NEW_OBS` SBP message.
 *
 * \param P Pseudorange in meters
 * \param L Carrier phase in cycles
 * \param D Measured doppler in Hz
 * \param cn0 Signal-to-noise ratio
 * \param lock_time Lock time gives an indication of the time
                    for which a signal has maintained continuous phase lock in
                    second
 * \param flags Observation flags from nav_meas_flags_t
 * \param sid Signal ID
 * \param msg Pointer to a `msg_obs_content_t` struct to fill out
 * \return `0` on success or `-1` on an overflow error
 */
s8 pack_obs_content(const navigation_measurement_t *meas,
                    packed_obs_content_t *msg) {
  u32 P_tmp;  // We have to have an intermediate since msg->P is packed and
              // could be unaligned
  if (!pack_pseudorange(meas->raw_pseudorange, meas->flags, &P_tmp)) {
    log_error_sid(meas->sid,
                  "observation message packing: P integer overflow "
                  "(%.1f,%.1f,%.1f,%.1f,%.1f,0x%X)",
                  meas->raw_pseudorange, meas->raw_carrier_phase,
                  meas->raw_measured_doppler, meas->cn0, meas->lock_time,
                  meas->flags);
    return -1;
  }
  msg->P = P_tmp;

  if (!pack_carrier_phase(meas->raw_carrier_phase, meas->flags, &msg->L)) {
    log_error_sid(meas->sid,
                  "observation message packing: L integer overflow "
                  "(%.1f,%.1f,%.1f,%.1f,%.1f,0x%X)",
                  meas->raw_pseudorange, meas->raw_carrier_phase,
                  meas->raw_measured_doppler, meas->cn0, meas->lock_time,
                  meas->flags);
    return -1;
  }

  if (!pack_doppler(meas->raw_measured_doppler, meas->flags, &msg->D)) {
    log_error_sid(meas->sid,
                  "observation message packing: D integer overflow "
                  "(%.1f,%.1f,%.1f,%.1f,%.1f,0x%X)",
                  meas->raw_pseudorange, meas->raw_carrier_phase,
                  meas->raw_measured_doppler, meas->cn0, meas->lock_time,
                  meas->flags);
    return -1;
  }

  if (!pack_cn0(meas->cn0, meas->flags, &msg->cn0)) {
    log_error_sid(meas->sid,
                  "observation message packing: C/N0 integer overflow "
                  "(%.1f,%.1f,%.1f,%.1f,%.1f,0x%X)",
                  meas->raw_pseudorange, meas->raw_carrier_phase,
                  meas->raw_measured_doppler, meas->cn0, meas->lock_time,
                  meas->flags);
    return -1;
  }

  msg->lock = encode_lock_time(meas->lock_time);

  msg->flags = nm_flags_to_sbp(meas->flags);

  msg->sid = sid_to_sbp(meas->sid);

  return 0;
}

/** Pack GPS observables into a `msg_obs_content_t` struct.
 * For use in constructing a `MSG_NEW_OBS` SBP message.
 *
 * \param P Pseudorange in meters
 * \param L Carrier phase in cycles
 * \param D Measured doppler in Hz
 * \param cn0 Signal-to-noise ratio
 * \param lock_time Lock time gives an indication of the time
                    for which a signal has maintained continuous phase lock in
                    second
 * \param flags Observation flags from nav_meas_flags_t
 * \param sid Signal ID
 * \param msg Pointer to a `msg_obs_content_t` struct to fill out
 * \return `0` on success or `-1` on an overflow error
 */
s8 pack_osr_content(const navigation_measurement_t *meas,
                    const measurement_std_t *meas_std,
                    packed_osr_content_t *msg) {
  u32 P_tmp;  // We have to have an intermediate since msg->P is packed and
              // could be unaligned
  if (!pack_pseudorange(meas->raw_pseudorange, NAV_MEAS_FLAG_CODE_VALID,
                        &P_tmp)) {
    log_error_sid(meas->sid,
                  "observation message packing: P integer overflow "
                  "(%.1f,%.1f,%.1f)",
                  meas->raw_pseudorange, meas->raw_carrier_phase,
                  meas->lock_time);
    return -1;
  }
  msg->P = P_tmp;

  if (!pack_carrier_phase(meas->raw_carrier_phase, NAV_MEAS_FLAG_PHASE_VALID,
                          &msg->L)) {
    log_error_sid(meas->sid,
                  "observation message packing: L integer overflow "
                  "(%.1f,%.1f,%.1f)",
                  meas->raw_pseudorange, meas->raw_carrier_phase,
                  meas->lock_time);
    return -1;
  }

  msg->lock = encode_lock_time(meas->lock_time);

  msg->flags = meas_std->flags;

  msg->sid = sid_to_sbp(meas->sid);

  // Assume the std values are valid?
  {
    s32 iono_std_fp = lround(meas_std->iono_std * MSG_OSR_IONO_STD_MULTIPLIER);
    if (iono_std_fp < 0 || iono_std_fp > UINT16_MAX) {
      log_error_sid(meas_std->sid,
                    "OSR message packing: Iono std integer overflow "
                    "(%.1f,%.1f,%.1f,0x%X)",
                    meas_std->iono_std, meas_std->tropo_std,
                    meas_std->range_std, meas_std->flags);
      return -1;
    }
    msg->iono_std = iono_std_fp;
  }

  {
    s32 tropo_std_fp =
        lround(meas_std->tropo_std * MSG_OSR_TROPO_STD_MULTIPLIER);
    if (tropo_std_fp < 0 || tropo_std_fp > UINT16_MAX) {
      log_error_sid(meas_std->sid,
                    "OSR message packing: Tropo std integer overflow "
                    "(%.1f,%.1f,%.1f,0x%X)",
                    meas_std->iono_std, meas_std->tropo_std,
                    meas_std->range_std, meas_std->flags);
      return -1;
    }
    msg->tropo_std = tropo_std_fp;
  }

  {
    s32 range_std_fp =
        lround(meas_std->tropo_std * MSG_OSR_RANGE_STD_MULTIPLIER);
    if (range_std_fp < 0 || range_std_fp > UINT16_MAX) {
      log_error_sid(meas_std->sid,
                    "OSR message packing: Range std integer overflow "
                    "(%.1f,%.1f,%.1f,0x%X)",
                    meas_std->iono_std, meas_std->tropo_std,
                    meas_std->range_std, meas_std->flags);
      return -1;
    }
    msg->range_std = range_std_fp;
  }

  return 0;
}

static void pack_ephemeris_common(const ephemeris_t *e,
                                  ephemeris_common_content_t *common) {
  common->toe.tow = round(e->toe.tow);
  common->toe.wn = e->toe.wn;
  common->valid = e->valid;
  common->health_bits = e->health_bits;
  common->sid = sid_to_sbp(e->sid);
  common->fit_interval = e->fit_interval;
  common->ura = e->ura;
}

static void pack_ephemeris_gps(const ephemeris_t *e, msg_ephemeris_t *m) {
  const ephemeris_kepler_t *k = &e->kepler;
  msg_ephemeris_gps_t *msg = &m->gps;
  pack_ephemeris_common(e, &msg->common);
  msg->tgd = k->tgd.gps_s[0];
  msg->c_rs = (float)k->crs;
  msg->c_rc = (float)k->crc;
  msg->c_uc = (float)k->cuc;
  msg->c_us = (float)k->cus;
  msg->c_ic = (float)k->cic;
  msg->c_is = (float)k->cis;
  msg->dn = k->dn;
  msg->m0 = k->m0;
  msg->ecc = k->ecc;
  msg->sqrta = k->sqrta;
  msg->omega0 = k->omega0;
  msg->omegadot = k->omegadot;
  msg->w = k->w;
  msg->inc = k->inc;
  msg->inc_dot = k->inc_dot;
  msg->af0 = (float)k->af0;
  msg->af1 = (float)k->af1;
  msg->af2 = (float)k->af2;
  msg->toc.tow = round(k->toc.tow);
  msg->toc.wn = k->toc.wn;
  msg->iode = k->iode;
  msg->iodc = k->iodc;
}

static void pack_ephemeris_qzss(const ephemeris_t *e, msg_ephemeris_t *m) {
  pack_ephemeris_gps(e, m);
}

static void pack_ephemeris_bds(const ephemeris_t *e, msg_ephemeris_t *m) {
  const ephemeris_kepler_t *k = &e->kepler;
  msg_ephemeris_bds_t *msg = &m->bds;
  pack_ephemeris_common(e, &msg->common);
  msg->tgd1 = k->tgd.bds_s[0];
  msg->tgd2 = k->tgd.bds_s[1];
  msg->c_rs = (float)k->crs;
  msg->c_rc = (float)k->crc;
  msg->c_uc = (float)k->cuc;
  msg->c_us = (float)k->cus;
  msg->c_ic = (float)k->cic;
  msg->c_is = (float)k->cis;
  msg->dn = k->dn;
  msg->m0 = k->m0;
  msg->ecc = k->ecc;
  msg->sqrta = k->sqrta;
  msg->omega0 = k->omega0;
  msg->omegadot = k->omegadot;
  msg->w = k->w;
  msg->inc = k->inc;
  msg->inc_dot = k->inc_dot;
  msg->af0 = k->af0;
  msg->af1 = (float)k->af1;
  msg->af2 = (float)k->af2;
  msg->toc.tow = round(k->toc.tow);
  msg->toc.wn = k->toc.wn;
  msg->iode = k->iode;
  msg->iodc = k->iodc;
}

static void pack_ephemeris_gal(const ephemeris_t *e, msg_ephemeris_t *m) {
  const ephemeris_kepler_t *k = &e->kepler;
  msg_ephemeris_gal_t *msg = &m->gal;
  pack_ephemeris_common(e, &msg->common);
  msg->bgd_e1e5a = k->tgd.gal_s[0];
  msg->bgd_e1e5b = k->tgd.gal_s[1];
  msg->c_rs = (float)k->crs;
  msg->c_rc = (float)k->crc;
  msg->c_uc = (float)k->cuc;
  msg->c_us = (float)k->cus;
  msg->c_ic = (float)k->cic;
  msg->c_is = (float)k->cis;
  msg->dn = k->dn;
  msg->m0 = k->m0;
  msg->ecc = k->ecc;
  msg->sqrta = k->sqrta;
  msg->omega0 = k->omega0;
  msg->omegadot = k->omegadot;
  msg->w = k->w;
  msg->inc = k->inc;
  msg->inc_dot = k->inc_dot;
  msg->af0 = k->af0;
  msg->af1 = k->af1;
  msg->af2 = (float)k->af2;
  msg->toc.tow = round(k->toc.tow);
  msg->toc.wn = k->toc.wn;
  msg->iode = k->iode;
  msg->iodc = k->iodc;
  msg->source = e->source;
}

static void pack_ephemeris_sbas(const ephemeris_t *e, msg_ephemeris_t *m) {
  const ephemeris_xyz_t *k = &e->xyz;
  msg_ephemeris_sbas_t *msg = &m->sbas;
  pack_ephemeris_common(e, &msg->common);
  msg->pos[0] = k->pos[0];
  msg->pos[1] = k->pos[1];
  msg->pos[2] = k->pos[2];
  msg->vel[0] = (float)k->vel[0];
  msg->vel[1] = (float)k->vel[1];
  msg->vel[2] = (float)k->vel[2];
  msg->acc[0] = (float)k->acc[0];
  msg->acc[1] = (float)k->acc[1];
  msg->acc[2] = (float)k->acc[2];
  msg->a_gf0 = (float)k->a_gf0;
  msg->a_gf1 = (float)k->a_gf1;
}

static void pack_ephemeris_glo(const ephemeris_t *e, msg_ephemeris_t *m) {
  const ephemeris_glo_t *k = &e->glo;
  msg_ephemeris_glo_t *msg = &m->glo;
  pack_ephemeris_common(e, &msg->common);
  msg->pos[0] = k->pos[0];
  msg->pos[1] = k->pos[1];
  msg->pos[2] = k->pos[2];
  msg->vel[0] = k->vel[0];
  msg->vel[1] = k->vel[1];
  msg->vel[2] = k->vel[2];
  msg->acc[0] = (float)k->acc[0];
  msg->acc[1] = (float)k->acc[1];
  msg->acc[2] = (float)k->acc[2];
  msg->gamma = (float)k->gamma;
  msg->tau = (float)k->tau;
  msg->d_tau = (float)k->d_tau;
  msg->iod = k->iod;
  msg->fcn = k->fcn;
}

typedef void (*pack_ephe_func)(const ephemeris_t *, msg_ephemeris_t *);

typedef struct {
  const msg_info_t msg_info;
  const pack_ephe_func pack;
} ephe_packer_table_element_t;

static ephe_packer_table_element_t ephe_packer_table[CONSTELLATION_COUNT] = {

    /* GPS */
    [CONSTELLATION_GPS] = {{SBP_MSG_EPHEMERIS_GPS, sizeof(msg_ephemeris_gps_t)},
                           pack_ephemeris_gps},

    /* SBAS */
    [CONSTELLATION_SBAS] = {{SBP_MSG_EPHEMERIS_SBAS,
                             sizeof(msg_ephemeris_sbas_t)},
                            pack_ephemeris_sbas},

    /* GLO */
    [CONSTELLATION_GLO] = {{SBP_MSG_EPHEMERIS_GLO, sizeof(msg_ephemeris_glo_t)},
                           pack_ephemeris_glo},

    /* BDS */
    [CONSTELLATION_BDS] = {{SBP_MSG_EPHEMERIS_BDS, sizeof(msg_ephemeris_bds_t)},
                           pack_ephemeris_bds},

    /* GAL */
    [CONSTELLATION_GAL] = {{SBP_MSG_EPHEMERIS_GAL, sizeof(msg_ephemeris_gal_t)},
                           pack_ephemeris_gal},

    /* QZSS */
    [CONSTELLATION_QZS] = {{SBP_MSG_EPHEMERIS_GPS, sizeof(msg_ephemeris_gps_t)},
                           pack_ephemeris_qzss},
};

msg_info_t pack_ephemeris(const ephemeris_t *e, msg_ephemeris_t *msg) {
  constellation_t c = sid_to_constellation(e->sid);

  assert(NULL != ephe_packer_table[c].pack);

  ephe_packer_table[c].pack(e, msg);

  return ephe_packer_table[c].msg_info;
}

/** This is helper function packs SBAS raw data structure.
 *
 * \param sid          GNSS signal identifier
 * \param tow_ms       GPS TOW [ms]
 * \param msg          SBAS message type
 * \param decoded      Unpacked decoded data
 * \param sbas_raw_msg The destination structure
 */
void sbp_pack_sbas_raw_data(const gnss_signal_t sid, u32 tow_ms, u8 msg,
                            const u8 *decoded, msg_sbas_raw_t *sbas_raw_msg) {
  memset(sbas_raw_msg->data, 0, sizeof(sbas_raw_msg->data));
  sbas_raw_msg->sid = sid_to_sbp(sid);
  sbas_raw_msg->tow = tow_ms;
  sbas_raw_msg->message_type = msg;
  /* Copy data ignoring preamble + msg_id (8 + 6) bits. */
  bitcopy(sbas_raw_msg->data, 0, decoded, 14, 212);
  /* Pad with zeros. */
  sbas_raw_msg->data[26] &= 0xF0;
}

static void pack_almanac_common(const almanac_t *a,
                                almanac_common_content_t *common) {
  common->toa.tow = round(a->toa.tow);
  common->toa.wn = a->toa.wn;
  common->valid = a->valid;
  common->health_bits = a->health_bits;
  common->sid = sid_to_sbp(a->sid);
  common->fit_interval = a->fit_interval;
  common->ura = a->ura;
}

static void pack_almanac_gps(const almanac_t *a, msg_almanac_t *m) {
  msg_almanac_gps_t *msg = &m->gps;
  pack_almanac_common(a, &msg->common);
  msg->m0 = a->kepler.m0;
  msg->ecc = a->kepler.ecc;
  msg->sqrta = a->kepler.sqrta;
  msg->omega0 = a->kepler.omega0;
  msg->omegadot = a->kepler.omegadot;
  msg->w = a->kepler.w;
  msg->inc = a->kepler.inc;
  msg->af0 = a->kepler.af0;
  msg->af1 = a->kepler.af1;
}

static void pack_almanac_glo(const almanac_t *a, msg_almanac_t *m) {
  msg_almanac_glo_t *msg = &m->glo;
  pack_almanac_common(a, &msg->common);
  msg->lambda_na = a->glo.lambda;
  msg->t_lambda_na = a->glo.t_lambda;
  msg->i = a->glo.i;
  msg->t = a->glo.t;
  msg->t_dot = a->glo.t_dot;
  msg->epsilon = a->glo.epsilon;
  msg->omega = a->glo.omega;
}

#define TYPE_TABLE_INVALID_MSG_ID 0
typedef void (*pack_alma_func)(const almanac_t *, msg_almanac_t *);

typedef struct {
  const msg_info_t msg_info;
  const pack_alma_func pack;
} alma_type_table_element_t;

static alma_type_table_element_t alma_type_table[CONSTELLATION_COUNT] = {

    /* GPS */
    {{SBP_MSG_ALMANAC_GPS, sizeof(msg_almanac_gps_t)}, pack_almanac_gps},

    /* SBAS almanac not supported at the moment */
    {{TYPE_TABLE_INVALID_MSG_ID, 0}, NULL},

    /* GLO */
    {{SBP_MSG_ALMANAC_GLO, sizeof(msg_almanac_glo_t)}, pack_almanac_glo}};

msg_info_t pack_almanac(const almanac_t *a, msg_almanac_t *msg) {
  constellation_t c = sid_to_constellation(a->sid);

  assert(NULL != alma_type_table[c].pack);

  alma_type_table[c].pack(a, msg);

  return alma_type_table[c].msg_info;
}

void sbp_pack_glonass_biases_content(const glo_biases_t glonass_biases,
                                     msg_glo_biases_t *msg) {
  msg->mask = (glonass_biases.mask);
  msg->l1ca_bias =
      round(glonass_biases.l1of_bias_m * MSG_GLO_BIASES_MULTIPLIER);
  msg->l1p_bias = round(glonass_biases.l1p_bias_m * MSG_GLO_BIASES_MULTIPLIER);
  msg->l2ca_bias =
      round(glonass_biases.l2of_bias_m * MSG_GLO_BIASES_MULTIPLIER);
  msg->l2p_bias = round(glonass_biases.l2p_bias_m * MSG_GLO_BIASES_MULTIPLIER);
}

// pack from YPR in NED frame (in radians)
#define SBP_ORIENT_INVALID 0
#define SBP_ORIENT_VALID 1
void sbp_init_orient_euler(msg_orient_euler_t *msg, const gps_time_t *t) {
  msg->tow = round_tow_ms(t->tow);
  msg->flags = SBP_ORIENT_INVALID;
}

// all angle values in radians
void sbp_make_orient_euler(msg_orient_euler_t *msg, const gps_time_t *t,
                           double yaw, double pit, double rol, double yaw_dev,
                           double pit_dev, double rol_dev) {
  sbp_init_orient_euler(msg, t);
  // all angles are in microdegrees
  msg->roll = (int32_t)(rol * R2D * 1e6);
  msg->pitch = (int32_t)(pit * R2D * 1e6);
  msg->yaw = (int32_t)(yaw * R2D * 1e6);
  // uncertainties are in degrees
  msg->roll_accuracy = (float)rol_dev * R2D;
  msg->pitch_accuracy = (float)pit_dev * R2D;
  msg->yaw_accuracy = (float)yaw_dev * R2D;
  msg->flags = SBP_ORIENT_VALID;
}
