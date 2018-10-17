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

#include <string.h>

/*********************************************************************
 * External Dependencies -- TODO(kevin) remove these.
 ********************************************************************/

extern u32 round_tow_ms(double tow);

/*********************************************************************
 * SBP Helper Functions
 ********************************************************************/

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




