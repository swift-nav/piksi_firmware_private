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

#include "starling_sbp_messages.h"

#include <math.h>
#include <string.h>

/*********************************************************************
 * External Dependencies -- TODO(kevin) remove these.
 *********************************************************************/

/*********************************************************************
 * Local Helpers
 *********************************************************************/
/**
 * Helper function for rounding tow to integer milliseconds, taking care of
 * week roll-over
 * @param[in] tow Time-of-week in seconds
 * @return Time-of-week in milliseconds
 */
static u32 round_tow_ms(double tow) {
  /* week roll-over */
  u32 tow_ms = round(tow * 1e3);
  while (tow_ms >= WEEK_MS) {
    tow_ms -= WEEK_MS;
  }
  return tow_ms;
}

/**
 * Helper function for converting GPS time to integer milliseconds with
 * nanosecond remainder, taking care of week roll-over
 * @param[in] t_in GPS time
 * @param[out] t_out SBP time
 */
static void round_time_nano(const gps_time_t *t_in, sbp_gps_time_t *t_out) {
  t_out->wn = t_in->wn;
  t_out->tow = round(t_in->tow * 1e3);
  t_out->ns_residual = round((t_in->tow - t_out->tow / 1e3) * 1e9);
  /* week roll-over */
  if (t_out->tow >= WEEK_MS) {
    t_out->wn++;
    t_out->tow -= WEEK_MS;
  }
}

static double constrain_angle(const double heading) {
  double constrained_heading = fmod(heading, 360.0);
  if (constrained_heading < 0) {
    constrained_heading += 360.0;
  }
  return constrained_heading;
}

static u8 sbp_get_time_quality_flags(time_quality_t time_qual) {
  u8 flags = 0;
  switch (time_qual) {
    case TIME_FINE: /* Intentionally FALLTHROUGH */
    case TIME_FINEST:
      /* Time comes from the measurement engine and if it is
       * FINE or FINEST then we call it GNSS derived (flags=1) */
      flags = 1;
      break;
    case TIME_PROPAGATED:
      /* Time Propagated (flags=2) */
      flags = 2;
      break;
    case TIME_COARSE:  /* Intentionally FALLTHROUGH  */
    case TIME_UNKNOWN: /* Intentionally FALLTHROUGH  */
    default:
      /* Time COARSE or UNKNOWN ->  mark time as invalid    */
      flags = 0;
  }
  return flags;
}

/*********************************************************************
 * SBP Message-Packing Functions
 *********************************************************************/
#define MSG_HEADING_SCALE_FACTOR 1000.0

void sbp_init_gps_time(msg_gps_time_t *gps_time, gps_time_t *t, time_quality_t time_qual) {
  memset(gps_time, 0, sizeof(msg_gps_time_t));
  sbp_make_gps_time(gps_time, t, time_qual);
}

void sbp_init_utc_time(msg_utc_time_t *utc_time, gps_time_t *t, time_quality_t time_qual) {
  memset(utc_time, 0, sizeof(msg_utc_time_t));
  sbp_make_utc_time(utc_time, t, time_qual);
}

void sbp_init_pos_llh(msg_pos_llh_t *pos_llh, gps_time_t *t) {
  memset(pos_llh, 0, sizeof(msg_pos_llh_t));
  if (gps_time_valid(t)) {
    pos_llh->tow = round_tow_ms(t->tow);
  }
}

void sbp_init_pos_ecef(msg_pos_ecef_t *pos_ecef, gps_time_t *t) {
  memset(pos_ecef, 0, sizeof(msg_pos_ecef_t));
  if (gps_time_valid(t)) {
    pos_ecef->tow = round_tow_ms(t->tow);
  }
}

void sbp_init_vel_ned(msg_vel_ned_t *vel_ned, gps_time_t *t) {
  memset(vel_ned, 0, sizeof(msg_vel_ned_t));
  if (gps_time_valid(t)) {
    vel_ned->tow = round_tow_ms(t->tow);
  }
}

void sbp_init_vel_ecef(msg_vel_ecef_t *vel_ecef, gps_time_t *t) {
  memset(vel_ecef, 0, sizeof(msg_vel_ecef_t));
  if (gps_time_valid(t)) {
    vel_ecef->tow = round_tow_ms(t->tow);
  }
}

void sbp_init_baseline_ecef(msg_baseline_ecef_t *baseline_ecef, gps_time_t *t) {
  memset(baseline_ecef, 0, sizeof(msg_baseline_ecef_t));
  if (gps_time_valid(t)) {
    baseline_ecef->tow = round_tow_ms(t->tow);
  }
}

void sbp_init_baseline_ned(msg_baseline_ned_t *baseline_ned, gps_time_t *t) {
  memset(baseline_ned, 0, sizeof(msg_baseline_ned_t));
  if (gps_time_valid(t)) {
    baseline_ned->tow = round_tow_ms(t->tow);
  }
}

void sbp_init_baseline_heading(msg_baseline_heading_t *baseline_heading,
                               gps_time_t *t) {
  memset(baseline_heading, 0, sizeof(msg_baseline_heading_t));
  if (gps_time_valid(t)) {
    baseline_heading->tow = round_tow_ms(t->tow);
  }
}

void sbp_init_pos_ecef_cov(msg_pos_ecef_cov_t *pos_ecef_cov, gps_time_t *t) {
  memset(pos_ecef_cov, 0, sizeof(msg_pos_ecef_cov_t));
  if (gps_time_valid(t)) {
    pos_ecef_cov->tow = round_tow_ms(t->tow);
  }
}

void sbp_init_vel_ecef_cov(msg_vel_ecef_cov_t *vel_ecef_cov, gps_time_t *t) {
  memset(vel_ecef_cov, 0, sizeof(msg_vel_ecef_cov_t));
  if (gps_time_valid(t)) {
    vel_ecef_cov->tow = round_tow_ms(t->tow);
  }
}

void sbp_init_pos_llh_cov(msg_pos_llh_cov_t *pos_llh_cov, gps_time_t *t) {
  memset(pos_llh_cov, 0, sizeof(msg_pos_llh_cov_t));
  if (gps_time_valid(t)) {
    pos_llh_cov->tow = round_tow_ms(t->tow);
  }
}

void sbp_init_vel_ned_cov(msg_vel_ned_cov_t *vel_ned_cov, gps_time_t *t) {
  memset(vel_ned_cov, 0, sizeof(msg_vel_ned_cov_t));
  if (gps_time_valid(t)) {
    vel_ned_cov->tow = round_tow_ms(t->tow);
  }
}

void sbp_init_sbp_dops(msg_dops_t *sbp_dops, gps_time_t *t) {
  memset(sbp_dops, 0, sizeof(msg_dops_t));
  if (gps_time_valid(t)) {
    sbp_dops->tow = round_tow_ms(t->tow);
  }
}

void sbp_init_age_corrections(msg_age_corrections_t *age_corrections,
                              gps_time_t *t) {
  memset(age_corrections, 0, sizeof(msg_age_corrections_t));
  age_corrections->age = 0xFFFF;
  if (gps_time_valid(t)) {
    age_corrections->tow = round_tow_ms(t->tow);
  }
}

void sbp_init_dgnss_status(msg_dgnss_status_t *dgnss_status) {
  memset(dgnss_status, 0, sizeof(msg_dgnss_status_t));
}

void sbp_make_pos_llh_vect(msg_pos_llh_t *pos_llh,
                           const double llh[3],
                           double h_accuracy,
                           double v_accuracy,
                           const gps_time_t *gps_t,
                           u8 n_sats_used,
                           u8 flags) {
  pos_llh->tow = round_tow_ms(gps_t->tow);
  pos_llh->lat = llh[0] * R2D;
  pos_llh->lon = llh[1] * R2D;
  pos_llh->height = llh[2];
  pos_llh->h_accuracy = MIN(round(1e3 * h_accuracy), UINT16_MAX);
  pos_llh->v_accuracy = MIN(round(1e3 * v_accuracy), UINT16_MAX);
  pos_llh->n_sats = n_sats_used;
  pos_llh->flags = flags;
}

void sbp_make_pos_llh_cov(msg_pos_llh_cov_t *pos_llh_cov,
                          const double llh[3],
                          const double llh_cov[6],
                          const gps_time_t *gps_t,
                          u8 n_sats_used,
                          u8 flags) {
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

void sbp_make_pos_ecef_vect(msg_pos_ecef_t *pos_ecef,
                            const double ecef[3],
                            double accuracy,
                            const gps_time_t *gps_t,
                            u8 n_sats_used,
                            u8 flags) {
  pos_ecef->tow = round_tow_ms(gps_t->tow);
  pos_ecef->x = ecef[0];
  pos_ecef->y = ecef[1];
  pos_ecef->z = ecef[2];
  pos_ecef->accuracy = MIN(round(1e3 * accuracy), UINT16_MAX);
  pos_ecef->n_sats = n_sats_used;
  pos_ecef->flags = flags;
}

void sbp_make_pos_ecef_cov(msg_pos_ecef_cov_t *pos_ecef_cov,
                           const double ecef[3],
                           const double ecef_cov[6],
                           const gps_time_t *gps_t,
                           u8 n_sats_used,
                           u8 flags) {
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

void sbp_make_vel_ned(msg_vel_ned_t *vel_ned,
                      const double v_ned[3],
                      double h_accuracy,
                      double v_accuracy,
                      const gps_time_t *gps_t,
                      u8 n_sats_used,
                      u8 flags) {
  vel_ned->tow = round_tow_ms(gps_t->tow);
  vel_ned->n = round(v_ned[0] * 1e3);
  vel_ned->e = round(v_ned[1] * 1e3);
  vel_ned->d = round(v_ned[2] * 1e3);
  vel_ned->h_accuracy = MIN(round(1e3 * h_accuracy), UINT16_MAX);
  vel_ned->v_accuracy = MIN(round(1e3 * v_accuracy), UINT16_MAX);
  vel_ned->n_sats = n_sats_used;
  vel_ned->flags = flags;
}

void sbp_make_vel_ned_cov(msg_vel_ned_cov_t *vel_ned_cov,
                          const double v_ned[3],
                          const double ned_cov[6],
                          const gps_time_t *gps_t,
                          u8 n_sats_used,
                          u8 flags) {
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

void sbp_make_vel_ecef(msg_vel_ecef_t *vel_ecef,
                       const double v_ecef[3],
                       double accuracy,
                       const gps_time_t *gps_t,
                       u8 n_sats_used,
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
                           const double v_ecef[3],
                           const double ecef_cov[6],
                           const gps_time_t *gps_t,
                           u8 n_sats_used,
                           u8 flags) {
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
                            const gps_time_t *t,
                            u8 n_sats,
                            const double b_ecef[3],
                            double accuracy,
                            u8 flags) {
  baseline_ecef->tow = round_tow_ms(t->tow);
  baseline_ecef->x = round(1e3 * b_ecef[0]);
  baseline_ecef->y = round(1e3 * b_ecef[1]);
  baseline_ecef->z = round(1e3 * b_ecef[2]);
  baseline_ecef->accuracy = MIN(round(1e3 * accuracy), UINT16_MAX);
  baseline_ecef->n_sats = n_sats;
  baseline_ecef->flags = flags;
}

void sbp_make_baseline_ned(msg_baseline_ned_t *baseline_ned,
                           const gps_time_t *t,
                           u8 n_sats,
                           const double b_ned[3],
                           double h_accuracy,
                           double v_accuracy,
                           u8 flags) {
  baseline_ned->tow = round_tow_ms(t->tow);
  baseline_ned->n = round(1e3 * b_ned[0]);
  baseline_ned->e = round(1e3 * b_ned[1]);
  baseline_ned->d = round(1e3 * b_ned[2]);
  baseline_ned->h_accuracy = MIN(round(1e3 * h_accuracy), UINT16_MAX);
  baseline_ned->v_accuracy = MIN(round(1e3 * v_accuracy), UINT16_MAX);
  baseline_ned->n_sats = n_sats;
  baseline_ned->flags = flags;
}

void sbp_make_heading(msg_baseline_heading_t *baseline_heading,
                      const gps_time_t *t,
                      const double heading,
                      u8 n_sats,
                      u8 flags) {
  baseline_heading->tow = round_tow_ms(t->tow);
  baseline_heading->heading =
      round(constrain_angle(heading) * MSG_HEADING_SCALE_FACTOR);
  baseline_heading->n_sats = n_sats;
  baseline_heading->flags = flags;
}

void sbp_make_dops(msg_dops_t *dops_out,
                   const dops_t *dops_in,
                   const u32 tow,
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
                              const gps_time_t *t,
                              double propagation_time) {
  age_corrections->tow = round_tow_ms(t->tow);
  age_corrections->age = MIN(round(10 * propagation_time), UINT16_MAX);
}

void sbp_make_dgnss_status(msg_dgnss_status_t *dgnss_status,
    u8 num_sats,
    double obs_latency,
    u8 flags) {
  if (flags > POSITION_MODE_DGNSS) {
    dgnss_status->flags = 2;
  } else {
    dgnss_status->flags = 1;
  }
  dgnss_status->latency = MIN(round(10 * obs_latency), UINT16_MAX);
  dgnss_status->num_signals = num_sats;
}

void sbp_make_gps_time(msg_gps_time_t *t_out,
                       const gps_time_t *t_in,
                       time_quality_t time_qual) {
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
  t_out->flags = sbp_get_time_quality_flags(time_qual) & 0x7;
}

void sbp_make_utc_time(msg_utc_time_t *t_out,
                       const gps_time_t *t_in,
                       time_quality_t time_qual) {
  if (!gps_time_valid(t_in)) {
    memset(t_out, 0, sizeof(msg_utc_time_t));
    return;
  }
  u8 flags = 0;
  utc_params_t utc_params;
  utc_params_t *p_utc_params = &utc_params;
  bool is_nv;
  /* try to read UTC parameters from NDB */
  if (NDB_ERR_NONE == ndb_utc_params_read(&utc_params, &is_nv)) {
    if (is_nv) {
      flags |= (NVM_UTC << 3);
    } else {
      flags |= (DECODED_UTC << 3);
    }
  } else {
    p_utc_params = NULL;
    flags |= (DEFAULT_UTC << 3);
  }

  flags |= (sbp_get_time_quality_flags(time_qual) & 0x7);

  /* convert to UTC (falls back to a hard-coded table if the pointer is null) */
  utc_tm utc_time;
  gps2utc(t_in, &utc_time, p_utc_params);

  t_out->tow = round_tow_ms(t_in->tow);
  t_out->year = utc_time.year;
  t_out->month = utc_time.month;
  t_out->day = utc_time.month_day;
  t_out->hours = utc_time.hour;
  t_out->minutes = utc_time.minute;
  t_out->seconds = utc_time.second_int;
  assert(utc_time.second_frac >= 0.0);
  assert(utc_time.second_frac < 1.0);
  /* round the nanosecond part down to stop it rounding up to the next second */
  t_out->ns = floor(utc_time.second_frac * 1e9);
  t_out->flags = flags;
}

/*********************************************************************
 * High Level SBP Messages API 
 *********************************************************************/

void sbp_messages_init(sbp_messages_t *sbp_messages,
                       const gps_time_t *epoch_time,
                       time_quality_t time_qual) {
  /* Necessary because some of these functions strip the const qualifier. */
  gps_time_t *t = (gps_time_t *)epoch_time;
  /* if there is ANY time known here better than propagated,
   * initialize time_qual as time_propagated for SBP output.
   * If we have a GNSS solution, we will override with the sbp GNSS Solution
   * time quality */
  if (TIME_PROPAGATED <= time_qual) {
    time_qual = TIME_PROPAGATED;
  }
  sbp_init_gps_time(&sbp_messages->gps_time, t, time_qual);
  sbp_init_utc_time(&sbp_messages->utc_time, t, time_qual);
  sbp_init_pos_llh(&sbp_messages->pos_llh, t);
  sbp_init_pos_ecef(&sbp_messages->pos_ecef, t);
  sbp_init_vel_ned(&sbp_messages->vel_ned, t);
  sbp_init_vel_ecef(&sbp_messages->vel_ecef, t);
  sbp_init_sbp_dops(&sbp_messages->sbp_dops, t);
  sbp_init_age_corrections(&sbp_messages->age_corrections, t);
  sbp_init_dgnss_status(&sbp_messages->dgnss_status);
  sbp_init_baseline_ecef(&sbp_messages->baseline_ecef, t);
  sbp_init_baseline_ned(&sbp_messages->baseline_ned, t);
  sbp_init_baseline_heading(&sbp_messages->baseline_heading, t);
  sbp_init_pos_ecef_cov(&sbp_messages->pos_ecef_cov, t);
  sbp_init_vel_ecef_cov(&sbp_messages->vel_ecef_cov, t);
  sbp_init_pos_llh_cov(&sbp_messages->pos_llh_cov, t);
  sbp_init_vel_ned_cov(&sbp_messages->vel_ned_cov, t);
}

