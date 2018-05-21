/*
 * Copyright (C) 2014, 2016 - 2017 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *          Pasi Miettinen <pasi.miettinen@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <libswiftnav/constants.h>
#include <libswiftnav/glo_map.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/memcpy_s.h>
#include <libswiftnav/observation.h>

#include "ndb/ndb.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "timing/timing.h"

/** \addtogroup sbp
 * \{ */

/** \defgroup sbp_utils SBP Utils
 * Convert to and from SBP message types and other useful functions.
 * \{ */

u32 round_tow_ms(double tow);
void round_time_nano(const gps_time_t *t_in, sbp_gps_time_t *t_out);

sbp_gnss_signal_t sid_to_sbp(const gnss_signal_t from) {
  sbp_gnss_signal_t sbp_sid = {
      .code = from.code, .sat = from.sat,
  };

  return sbp_sid;
}

gnss_signal_t sid_from_sbp(const sbp_gnss_signal_t from) {
  gnss_signal_t sid = {
      .code = from.code, .sat = from.sat,
  };

  return sid;
}

void sbp_make_gps_time(msg_gps_time_t *t_out,
                       const gps_time_t *t_in,
                       u8 flags) {
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
  t_out->flags = flags;
}

void sbp_make_utc_time(msg_utc_time_t *t_out,
                       const gps_time_t *t_in,
                       u8 flags) {
  if (!gps_time_valid(t_in)) {
    memset(t_out, 0, sizeof(msg_utc_time_t));
    return;
  }

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

void sbp_make_dgnss_status(msg_dgnss_status_t *dgnss_status,
                           u8 num_sats,
                           double obs_latency,
                           u8 flags) {
  if (flags > DGNSS_POSITION) {
    dgnss_status->flags = 2;
  } else {
    dgnss_status->flags = 1;
  }
  dgnss_status->latency = MIN(round(10 * obs_latency), UINT16_MAX);
  dgnss_status->num_signals = num_sats;
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

double constrain_angle(const double heading) {
  double constrained_heading = fmod(heading, 360.0);
  if (constrained_heading < 0) {
    constrained_heading += 360.0;
  }
  return constrained_heading;
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

void sbp_make_age_corrections(msg_age_corrections_t *age_corrections,
                              const gps_time_t *t,
                              double propagation_time) {
  age_corrections->tow = round_tow_ms(t->tow);
  age_corrections->age = MIN(round(10 * propagation_time), UINT16_MAX);
}

void sbp_send_ndb_event(u8 event,
                        u8 obj_type,
                        u8 result,
                        u8 data_source,
                        const gnss_signal_t *object_sid,
                        const gnss_signal_t *src_sid,
                        u16 sender) {
  msg_ndb_event_t msg;
  memset(&msg, 0, sizeof(msg));

  msg.recv_time = timing_getms();
  msg.event = event;
  msg.object_type = obj_type;
  msg.result = result;
  msg.data_source = data_source;

  if (NULL != object_sid) {
    msg.object_sid = sid_to_sbp(*object_sid);
  }

  if (NULL != src_sid) {
    msg.src_sid = sid_to_sbp(*src_sid);
  }

  msg.original_sender = sender;

  sbp_send_msg(SBP_MSG_NDB_EVENT, sizeof(msg), (u8 *)&msg);
}

void unpack_obs_header(const observation_header_t *msg,
                       gps_time_t *t,
                       u8 *total,
                       u8 *count) {
  t->wn = msg->t.wn;
  t->tow = ((double)msg->t.tow) / 1e3 + ((double)msg->t.ns_residual) / 1e9;
  normalize_gps_time(t);
  *total = (msg->n_obs >> MSG_OBS_HEADER_SEQ_SHIFT);
  *count = (msg->n_obs & MSG_OBS_HEADER_SEQ_MASK);
}

void pack_obs_header(const gps_time_t *t,
                     u8 total,
                     u8 count,
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

nav_meas_flags_t nm_flags_from_sbp(u8 from) {
  nav_meas_flags_t to = 0;
  if (0 != (from & MSG_OBS_FLAGS_CODE_VALID)) {
    to |= NAV_MEAS_FLAG_CODE_VALID;
  }
  if (0 != (from & MSG_OBS_FLAGS_PHASE_VALID)) {
    to |= NAV_MEAS_FLAG_PHASE_VALID;
  }
  if (0 != (from & MSG_OBS_FLAGS_HALF_CYCLE_KNOWN)) {
    to |= NAV_MEAS_FLAG_HALF_CYCLE_KNOWN;
  }
  if (0 != (from & MSG_OBS_FLAGS_MEAS_DOPPLER_VALID)) {
    to |= NAV_MEAS_FLAG_MEAS_DOPPLER_VALID;
  }
  if (0 != (from & MSG_OBS_FLAGS_RAIM_EXCLUSION)) {
    to |= NAV_MEAS_FLAG_RAIM_EXCLUSION;
  }
  return to;
}

void unpack_obs_content(const packed_obs_content_t *msg,
                        double *P,
                        double *L,
                        double *D,
                        double *cn0,
                        double *lock_time,
                        nav_meas_flags_t *flags,
                        gnss_signal_t *sid) {
  *P = ((double)msg->P) / MSG_OBS_P_MULTIPLIER;
  *L = (((double)msg->L.i) + (((double)msg->L.f) / MSG_OBS_LF_MULTIPLIER));
  *D = (((double)msg->D.i) + (((double)msg->D.f) / MSG_OBS_DF_MULTIPLIER));
  *cn0 = ((double)msg->cn0) / MSG_OBS_CN0_MULTIPLIER;
  *lock_time = decode_lock_time(msg->lock);
  *flags = nm_flags_from_sbp(msg->flags);
  if (msg->cn0 != 0) {
    *flags |= NAV_MEAS_FLAG_CN0_VALID;
  }
  *sid = sid_from_sbp(msg->sid);
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
s8 pack_obs_content(double P,
                    double L,
                    double D,
                    double cn0,
                    double lock_time,
                    nav_meas_flags_t flags,
                    gnss_signal_t sid,
                    packed_obs_content_t *msg) {
  s64 P_fp = llround(P * MSG_OBS_P_MULTIPLIER);
  if (P < 0 || P_fp > UINT32_MAX) {
    log_error_sid(sid,
                  "observation message packing: P integer overflow "
                  "(%.1f,%.1f,%.1f,%.1f,%.1f,0x%X)",
                  P,
                  L,
                  D,
                  cn0,
                  lock_time,
                  flags);
    return -1;
  }

  msg->P = P_fp;

  double Li = floor(L);
  if (Li < INT32_MIN || Li > INT32_MAX) {
    log_error_sid(sid,
                  "observation message packing: L integer overflow "
                  "(%.1f,%.1f,%.1f,%.1f,%.1f,0x%X)",
                  P,
                  L,
                  D,
                  cn0,
                  lock_time,
                  flags);
    return -1;
  }

  double Lf = L - Li;

  msg->L.i = Li;
  u16 frac_part_cp = round(Lf * MSG_OBS_LF_MULTIPLIER);
  if (frac_part_cp >= MSG_OBS_LF_OVERFLOW) {
    frac_part_cp = 0;
    msg->L.i += 1;
  }
  msg->L.f = frac_part_cp;

  double Di = floor(D);
  if (Di < INT16_MIN || Di > INT16_MAX) {
    log_error_sid(sid,
                  "observation message packing: D integer overflow "
                  "(%.1f,%.1f,%.1f,%.1f,%.1f,0x%X)",
                  P,
                  L,
                  D,
                  cn0,
                  lock_time,
                  flags);
    return -1;
  }

  double Df = D - Di;

  msg->D.i = Di;
  u16 frac_part_d = round(Df * MSG_OBS_DF_MULTIPLIER);
  if (frac_part_d >= MSG_OBS_DF_OVERFLOW) {
    frac_part_d = 0;
    msg->D.i += 1;
  }
  msg->D.f = frac_part_d;

  if (0 != (flags & NAV_MEAS_FLAG_CN0_VALID)) {
    s32 cn0_fp = lround(cn0 * MSG_OBS_CN0_MULTIPLIER);
    if (cn0 < 0 || cn0_fp > UINT8_MAX) {
      log_error_sid(sid,
                    "observation message packing: C/N0 integer overflow "
                    "(%.1f,%.1f,%.1f,%.1f,%.1f,0x%X)",
                    P,
                    L,
                    D,
                    cn0,
                    lock_time,
                    flags);
      return -1;
    }

    msg->cn0 = cn0_fp;
  } else {
    msg->cn0 = 0;
  }

  msg->lock = encode_lock_time(lock_time);

  msg->flags = nm_flags_to_sbp(flags);

  msg->sid = sid_to_sbp(sid);

  return 0;
}

static void unpack_ephemeris_common(const ephemeris_common_content_t *common,
                                    ephemeris_t *e) {
  e->toe.tow = common->toe.tow;
  e->toe.wn = common->toe.wn;
  e->valid = common->valid;
  e->health_bits = common->health_bits;
  e->sid = sid_from_sbp(common->sid);
  e->fit_interval = common->fit_interval;
  e->ura = common->ura;
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

static void unpack_ephemeris_gps(const msg_ephemeris_t *m, ephemeris_t *e) {
  const msg_ephemeris_gps_t *msg = &m->gps;
  unpack_ephemeris_common(&msg->common, e);
  e->kepler.tgd_gps_s = msg->tgd;
  e->kepler.crs = msg->c_rs;
  e->kepler.crc = msg->c_rc;
  e->kepler.cuc = msg->c_uc;
  e->kepler.cus = msg->c_us;
  e->kepler.cic = msg->c_ic;
  e->kepler.cis = msg->c_is;
  e->kepler.dn = msg->dn;
  e->kepler.m0 = msg->m0;
  e->kepler.ecc = msg->ecc;
  e->kepler.sqrta = msg->sqrta;
  e->kepler.omega0 = msg->omega0;
  e->kepler.omegadot = msg->omegadot;
  e->kepler.w = msg->w;
  e->kepler.inc = msg->inc;
  e->kepler.inc_dot = msg->inc_dot;
  e->kepler.af0 = msg->af0;
  e->kepler.af1 = msg->af1;
  e->kepler.af2 = msg->af2;
  e->kepler.toc.tow = msg->toc.tow;
  e->kepler.toc.wn = msg->toc.wn;
  e->kepler.iode = msg->iode;
  e->kepler.iodc = msg->iodc;
}

static void pack_ephemeris_gps(const ephemeris_t *e, msg_ephemeris_t *m) {
  msg_ephemeris_gps_t *msg = &m->gps;
  pack_ephemeris_common(e, &msg->common);
  msg->tgd = e->kepler.tgd_gps_s;
  msg->c_rs = e->kepler.crs;
  msg->c_rc = e->kepler.crc;
  msg->c_uc = e->kepler.cuc;
  msg->c_us = e->kepler.cus;
  msg->c_ic = e->kepler.cic;
  msg->c_is = e->kepler.cis;
  msg->dn = e->kepler.dn;
  msg->m0 = e->kepler.m0;
  msg->ecc = e->kepler.ecc;
  msg->sqrta = e->kepler.sqrta;
  msg->omega0 = e->kepler.omega0;
  msg->omegadot = e->kepler.omegadot;
  msg->w = e->kepler.w;
  msg->inc = e->kepler.inc;
  msg->inc_dot = e->kepler.inc_dot;
  msg->af0 = e->kepler.af0;
  msg->af1 = e->kepler.af1;
  msg->af2 = e->kepler.af2;
  msg->toc.tow = round(e->kepler.toc.tow);
  msg->toc.wn = e->kepler.toc.wn;
  msg->iode = e->kepler.iode;
  msg->iodc = e->kepler.iodc;
}

static void unpack_ephemeris_sbas(const msg_ephemeris_t *m, ephemeris_t *e) {
  const msg_ephemeris_sbas_t *msg = &m->sbas;
  unpack_ephemeris_common(&msg->common, e);
  MEMCPY_S(e->xyz.pos, sizeof(e->xyz.pos), msg->pos, sizeof(msg->pos));
  MEMCPY_S(e->xyz.vel, sizeof(e->xyz.vel), msg->vel, sizeof(msg->vel));
  MEMCPY_S(e->xyz.acc, sizeof(e->xyz.acc), msg->acc, sizeof(msg->acc));
  e->xyz.a_gf0 = msg->a_gf0;
  e->xyz.a_gf1 = msg->a_gf1;
}

static void pack_ephemeris_sbas(const ephemeris_t *e, msg_ephemeris_t *m) {
  msg_ephemeris_sbas_t *msg = &m->sbas;
  pack_ephemeris_common(e, &msg->common);
  MEMCPY_S(msg->pos, sizeof(msg->pos), e->xyz.pos, sizeof(e->xyz.pos));
  MEMCPY_S(msg->vel, sizeof(msg->vel), e->xyz.vel, sizeof(e->xyz.vel));
  MEMCPY_S(msg->acc, sizeof(msg->acc), e->xyz.acc, sizeof(e->xyz.acc));
  msg->a_gf0 = e->xyz.a_gf0;
  msg->a_gf1 = e->xyz.a_gf1;
}

static void unpack_ephemeris_glo(const msg_ephemeris_t *m, ephemeris_t *e) {
  const msg_ephemeris_glo_t *msg = &m->glo;
  unpack_ephemeris_common(&msg->common, e);
  MEMCPY_S(e->glo.pos, sizeof(e->glo.pos), msg->pos, sizeof(msg->pos));
  MEMCPY_S(e->glo.vel, sizeof(e->glo.vel), msg->vel, sizeof(msg->vel));
  MEMCPY_S(e->glo.acc, sizeof(e->glo.acc), msg->acc, sizeof(msg->acc));
  e->glo.gamma = msg->gamma;
  e->glo.tau = msg->tau;
  e->glo.d_tau = msg->d_tau;
  e->glo.iod = msg->iod;
  e->glo.fcn = (u16)msg->fcn;
  glo_map_set_slot_id(construct_mesid(msg->common.sid.code, (u16)msg->fcn),
                      msg->common.sid.sat);
}

static void pack_ephemeris_glo(const ephemeris_t *e, msg_ephemeris_t *m) {
  msg_ephemeris_glo_t *msg = &m->glo;
  pack_ephemeris_common(e, &msg->common);
  MEMCPY_S(msg->pos, sizeof(msg->pos), e->glo.pos, sizeof(e->glo.pos));
  MEMCPY_S(msg->vel, sizeof(msg->vel), e->glo.vel, sizeof(e->glo.vel));
  MEMCPY_S(msg->acc, sizeof(msg->acc), e->glo.acc, sizeof(e->glo.acc));
  msg->gamma = e->glo.gamma;
  msg->tau = e->glo.tau;
  msg->d_tau = e->glo.d_tau;
  msg->iod = e->glo.iod;
  msg->fcn = (u8)e->glo.fcn;
}

#define TYPE_TABLE_INVALID_MSG_ID 0

typedef void (*pack_ephe_func)(const ephemeris_t *, msg_ephemeris_t *);
typedef void (*unpack_ephe_func)(const msg_ephemeris_t *, ephemeris_t *);

typedef struct {
  const msg_info_t msg_info;
  const pack_ephe_func pack;
  const unpack_ephe_func unpack;
  sbp_msg_callbacks_node_t cbk_node;
} ephe_type_table_element_t;

static ephe_type_table_element_t ephe_type_table[CONSTELLATION_COUNT] = {

        /* GPS */
        [CONSTELLATION_GPS] = {{SBP_MSG_EPHEMERIS_GPS,
                                sizeof(msg_ephemeris_gps_t)},
                               pack_ephemeris_gps,
                               unpack_ephemeris_gps,
                               {0}},

        /* SBAS */
        [CONSTELLATION_SBAS] = {{SBP_MSG_EPHEMERIS_SBAS,
                                 sizeof(msg_ephemeris_sbas_t)},
                                pack_ephemeris_sbas,
                                unpack_ephemeris_sbas,
                                {0}},

        /* GLO */
        [CONSTELLATION_GLO] = {{SBP_MSG_EPHEMERIS_GLO,
                                sizeof(msg_ephemeris_glo_t)},
                               pack_ephemeris_glo,
                               unpack_ephemeris_glo,
                               {0}},

       /* GAL */
       [CONSTELLATION_GAL] = {{SBP_MSG_EPHEMERIS_GPS,
           sizeof(msg_ephemeris_gps_t)},
           pack_ephemeris_gps,
           unpack_ephemeris_gps,
           {0}},

                                   /* BDS */
        [CONSTELLATION_BDS2] = {
            {SBP_MSG_EPHEMERIS_GPS, sizeof(msg_ephemeris_gps_t)},
            pack_ephemeris_gps,
            unpack_ephemeris_gps,
            {0}}
};

void unpack_ephemeris(const msg_ephemeris_t *msg, ephemeris_t *e) {
  /* NOTE: here we use common part of GPS message to take sid.code info.
   *       this also should work for other GNSS because common part is located
   *       at the same place in memory for all structures.*/
  constellation_t c =
      code_to_constellation(((msg_ephemeris_gps_t *)msg)->common.sid.code);

  assert(NULL != ephe_type_table[c].unpack);

  ephe_type_table[c].unpack(msg, e);
}

msg_info_t pack_ephemeris(const ephemeris_t *e, msg_ephemeris_t *msg) {
  constellation_t c = sid_to_constellation(e->sid);

  assert(NULL != ephe_type_table[c].pack);

  ephe_type_table[c].pack(e, msg);

  return ephe_type_table[c].msg_info;
}

void sbp_ephe_reg_cbks(void (*ephemeris_msg_callback)(u16, u8, u8 *, void *)) {
  assert(ARRAY_SIZE(ephe_type_table) == CONSTELLATION_COUNT);

  for (u8 i = 0; i < ARRAY_SIZE(ephe_type_table); i++) {
    /* check if type is valid */
    if (TYPE_TABLE_INVALID_MSG_ID == ephe_type_table[i].msg_info.msg_id) {
      continue;
    }

    sbp_register_cbk(ephe_type_table[i].msg_info.msg_id,
                     ephemeris_msg_callback,
                     &ephe_type_table[i].cbk_node);
  }
}

/**
 * This is helper function packs and sends iono parameters over SBP
 * @param[in] iono pointer to Iono parameters
 */
void sbp_send_iono(const ionosphere_t *iono) {
  msg_iono_t msg_iono = {
      .t_nmct =
          {/* TODO: set this as 0 for now, beccause functionality
            * decodes tnmct is not available */
           .tow = 0,
           .wn = 0},
      .a0 = iono->a0,
      .a1 = iono->a1,
      .a2 = iono->a2,
      .a3 = iono->a3,
      .b0 = iono->b0,
      .b1 = iono->b1,
      .b2 = iono->b2,
      .b3 = iono->b3};

  /* send data over sbp */
  sbp_send_msg(SBP_MSG_IONO, sizeof(msg_iono_t), (u8 *)&msg_iono);
}

/**
 * This is helper function packs and sends L2C capabilities over SBP
 * @param[in] l2c_cap pointer to L2C capabilities mask
 */
void sbp_send_l2c_capabilities(const u32 *l2c_cap) {
  msg_sv_configuration_gps_t msg_l2c = {
      .t_nmct =
          {/* TODO: set this as 0 for now, beccause functionality
            * decodes tnmct is not available */
           .tow = 0,
           .wn = 0},
      .l2c_mask = *l2c_cap};

  /* send data over sbp */
  sbp_send_msg(SBP_MSG_SV_CONFIGURATION_GPS,
               sizeof(msg_sv_configuration_gps_t),
               (u8 *)&msg_l2c);
}

/**
 * This is helper function packs and sends Group delay over SBP
 * @param[in] cnav pointer to GPS CNAV message structure
 */
void sbp_send_group_delay(const cnav_msg_t *cnav) {
  gps_time_t t = get_current_time();
  msg_group_delay_t msg_cnav = {
      .t_op =
          {/* Convert from 6 seconds unit */
           .tow = (u32)(cnav->tow * 6),
           .wn = t.wn},
      .sid = (sbp_gnss_signal_t){.code = CODE_GPS_L2CM, .sat = cnav->prn},
      .valid = cnav->data.type_30.tgd_valid |
               cnav->data.type_30.isc_l2c_valid << 1 |
               cnav->data.type_30.isc_l1ca_valid << 2,
      .tgd = cnav->data.type_30.tgd,
      .isc_l1ca = cnav->data.type_30.isc_l1ca,
      .isc_l2c = cnav->data.type_30.isc_l2c};

  /* send data over sbp */
  sbp_send_msg(SBP_MSG_GROUP_DELAY, sizeof(msg_group_delay_t), (u8 *)&msg_cnav);
}

/** This is helper function packs SBAS raw data structure.
 *
 * \param sid          GNSS signal identifier
 * \param tow_ms       GPS TOW [ms]
 * \param msg          SBAS message type
 * \param decoded      Unpacked decoded data
 * \param sbas_raw_msg The destination structure
 */
void sbp_pack_sbas_raw_data(const gnss_signal_t sid,
                            u32 tow_ms,
                            u8 msg,
                            const u8 *decoded,
                            msg_sbas_raw_t *sbas_raw_msg) {
  memset(sbas_raw_msg->data, 0, sizeof(sbas_raw_msg->data));
  sbas_raw_msg->sid = sid_to_sbp(sid);
  sbas_raw_msg->tow = tow_ms;
  sbas_raw_msg->message_type = msg;
  /* Copy data ignoring preamble + msg_id (8 + 6) bits. */
  bitcopy(sbas_raw_msg->data, 0, decoded, 14, 212);
  /* Pad with zeros. */
  sbas_raw_msg->data[26] &= 0xF0;
}

void unpack_sbas_raw_data(const msg_sbas_raw_t *m, sbas_raw_data_t *d) {
  d->sid = sid_from_sbp(m->sid);
  d->message_type = m->message_type;
  d->time_of_transmission.tow = ((double)m->tow) / SECS_MS;
  MEMCPY_S(d->data, SBAS_RAW_PAYLOAD_LENGTH, m->data, SBAS_RAW_PAYLOAD_LENGTH);
}

/**
 * Helper function for rounding tow to integer milliseconds, taking care of
 * week roll-over
 * @param[in] tow Time-of-week in seconds
 * @return Time-of-week in milliseconds
 */
u32 round_tow_ms(double tow) {
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
void round_time_nano(const gps_time_t *t_in, sbp_gps_time_t *t_out) {
  t_out->wn = t_in->wn;
  t_out->tow = round(t_in->tow * 1e3);
  t_out->ns_residual = round((t_in->tow - t_out->tow / 1e3) * 1e9);
  /* week roll-over */
  if (t_out->tow >= WEEK_MS) {
    t_out->wn++;
    t_out->tow -= WEEK_MS;
  }
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

typedef void (*pack_alma_func)(const almanac_t *, msg_almanac_t *);
typedef void (*unpack_alma_func)(const msg_almanac_t *, almanac_t *);

typedef struct {
  const msg_info_t msg_info;
  const pack_alma_func pack;
  const unpack_alma_func unpack;
  sbp_msg_callbacks_node_t cbk_node;
} alma_type_table_element_t;

/* TODO: almanac unpacking functions*/
static alma_type_table_element_t alma_type_table[CONSTELLATION_COUNT] = {

    /* GPS */
    {{SBP_MSG_ALMANAC_GPS, sizeof(msg_almanac_gps_t)},
     pack_almanac_gps,
     NULL,
     {0}},

    /* SBAS almanac not supported at the moment */
    {{TYPE_TABLE_INVALID_MSG_ID, 0}, NULL, NULL, {0}},

    /* GLO */
    {{SBP_MSG_ALMANAC_GLO, sizeof(msg_almanac_glo_t)},
     pack_almanac_glo,
     NULL,
     {0}}};

void unpack_almanac(const msg_almanac_t *msg, almanac_t *a) {
  /* NOTE: here we use common part of GPS message to take sid.code info.
   *       this also should work for other GNSS because common part is located
   *       at the same place in memory for all structures.*/
  constellation_t c =
      code_to_constellation(((msg_almanac_gps_t *)msg)->common.sid.code);

  assert(NULL != alma_type_table[c].unpack);

  alma_type_table[c].unpack(msg, a);
}

msg_info_t pack_almanac(const almanac_t *a, msg_almanac_t *msg) {
  constellation_t c = sid_to_constellation(a->sid);

  assert(NULL != alma_type_table[c].pack);

  alma_type_table[c].pack(a, msg);

  return alma_type_table[c].msg_info;
}

void sbp_alma_reg_cbks(void (*almanac_msg_callback)(u16, u8, u8 *, void *)) {
  assert(ARRAY_SIZE(alma_type_table) == CONSTELLATION_COUNT);

  for (u8 i = 0; i < ARRAY_SIZE(alma_type_table); i++) {
    /* check if type is valid */
    if (TYPE_TABLE_INVALID_MSG_ID == alma_type_table[i].msg_info.msg_id) {
      continue;
    }

    sbp_register_cbk(alma_type_table[i].msg_info.msg_id,
                     almanac_msg_callback,
                     &alma_type_table[i].cbk_node);
  }
}

void sbp_init_gps_time(msg_gps_time_t *gps_time, gps_time_t *t) {
  memset(gps_time, 0, sizeof(msg_gps_time_t));
  sbp_make_gps_time(gps_time, t, NO_POSITION);
}

void sbp_init_utc_time(msg_utc_time_t *utc_time, gps_time_t *t) {
  memset(utc_time, 0, sizeof(msg_utc_time_t));
  sbp_make_utc_time(utc_time, t, NO_POSITION);
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

/** \} */
/** \} */
