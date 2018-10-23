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

#include <starling/pvt_engine/firmware_binding.h>
#include <swiftnav/constants.h>
#include <swiftnav/glo_map.h>
#include <swiftnav/logging.h>
#include <swiftnav/memcpy_s.h>

#include "ndb/ndb.h"
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

u8 sbp_get_time_quality_flags(u8 time_qual) {
  /* time_qual is the same as get_time_quality() return in ME API */
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

void sbp_make_gps_time(msg_gps_time_t *t_out,
                       const gps_time_t *t_in,
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
  t_out->flags = sbp_get_time_quality_flags(time_qual) & 0x7;
}

void sbp_make_utc_time(msg_utc_time_t *t_out,
                       const gps_time_t *t_in,
                       u8 time_qual) {
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

void unpack_obs_content(const packed_obs_content_t *msg, starling_obs_t *obs) {
  obs->pseudorange = ((double)msg->P) / MSG_OBS_P_MULTIPLIER;
  obs->carrier_phase =
      (((double)msg->L.i) + (((double)msg->L.f) / MSG_OBS_LF_MULTIPLIER));
  obs->doppler =
      (((double)msg->D.i) + (((double)msg->D.f) / MSG_OBS_DF_MULTIPLIER));
  obs->cn0 = ((double)msg->cn0) / MSG_OBS_CN0_MULTIPLIER;
  obs->lock_time = decode_lock_time(msg->lock);
  obs->flags = nm_flags_from_sbp(msg->flags);
  if (msg->cn0 != 0) {
    obs->flags |= NAV_MEAS_FLAG_CN0_VALID;
  }
  obs->sid = sid_from_sbp(msg->sid);

  /* Leave the TOT to be filled in by someone with knowledge of local time. */
  obs->tot = GPS_TIME_UNKNOWN;
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
  ephemeris_kepler_t *k = &e->kepler;
  unpack_ephemeris_common(&msg->common, e);
  k->tgd_gps_s = msg->tgd;
  k->crs = msg->c_rs;
  k->crc = msg->c_rc;
  k->cuc = msg->c_uc;
  k->cus = msg->c_us;
  k->cic = msg->c_ic;
  k->cis = msg->c_is;
  k->dn = msg->dn;
  k->m0 = msg->m0;
  k->ecc = msg->ecc;
  k->sqrta = msg->sqrta;
  k->omega0 = msg->omega0;
  k->omegadot = msg->omegadot;
  k->w = msg->w;
  k->inc = msg->inc;
  k->inc_dot = msg->inc_dot;
  k->af0 = msg->af0;
  k->af1 = msg->af1;
  k->af2 = msg->af2;
  k->toc.tow = msg->toc.tow;
  k->toc.wn = msg->toc.wn;
  k->iode = msg->iode;
  k->iodc = msg->iodc;
}

static void pack_ephemeris_gps(const ephemeris_t *e, msg_ephemeris_t *m) {
  const ephemeris_kepler_t *k = &e->kepler;
  msg_ephemeris_gps_t *msg = &m->gps;
  pack_ephemeris_common(e, &msg->common);
  msg->tgd = k->tgd_gps_s;
  msg->c_rs = k->crs;
  msg->c_rc = k->crc;
  msg->c_uc = k->cuc;
  msg->c_us = k->cus;
  msg->c_ic = k->cic;
  msg->c_is = k->cis;
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
  msg->af2 = k->af2;
  msg->toc.tow = round(k->toc.tow);
  msg->toc.wn = k->toc.wn;
  msg->iode = k->iode;
  msg->iodc = k->iodc;
}

static void unpack_ephemeris_bds(const msg_ephemeris_t *m, ephemeris_t *e) {
  const msg_ephemeris_bds_t *msg = &m->bds;
  ephemeris_kepler_t *k = &e->kepler;
  unpack_ephemeris_common(&msg->common, e);
  k->tgd_bds_s[0] = msg->tgd1;
  k->tgd_bds_s[1] = msg->tgd2;
  k->crs = msg->c_rs;
  k->crc = msg->c_rc;
  k->cuc = msg->c_uc;
  k->cus = msg->c_us;
  k->cic = msg->c_ic;
  k->cis = msg->c_is;
  k->dn = msg->dn;
  k->m0 = msg->m0;
  k->ecc = msg->ecc;
  k->sqrta = msg->sqrta;
  k->omega0 = msg->omega0;
  k->omegadot = msg->omegadot;
  k->w = msg->w;
  k->inc = msg->inc;
  k->inc_dot = msg->inc_dot;
  k->af0 = msg->af0;
  k->af1 = msg->af1;
  k->af2 = msg->af2;
  k->toc.tow = msg->toc.tow;
  k->toc.wn = msg->toc.wn;
  k->iode = msg->iode;
  k->iodc = msg->iodc;
}

static void pack_ephemeris_bds(const ephemeris_t *e, msg_ephemeris_t *m) {
  const ephemeris_kepler_t *k = &e->kepler;
  msg_ephemeris_bds_t *msg = &m->bds;
  pack_ephemeris_common(e, &msg->common);
  msg->tgd1 = k->tgd_bds_s[0];
  msg->tgd2 = k->tgd_bds_s[1];
  msg->c_rs = k->crs;
  msg->c_rc = k->crc;
  msg->c_uc = k->cuc;
  msg->c_us = k->cus;
  msg->c_ic = k->cic;
  msg->c_is = k->cis;
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
  msg->af2 = k->af2;
  msg->toc.tow = round(k->toc.tow);
  msg->toc.wn = k->toc.wn;
  msg->iode = k->iode;
  msg->iodc = k->iodc;
}

static void unpack_ephemeris_gal(const msg_ephemeris_t *m, ephemeris_t *e) {
  const msg_ephemeris_gal_t *msg = &m->gal;
  ephemeris_kepler_t *k = &e->kepler;
  unpack_ephemeris_common(&msg->common, e);
  k->tgd_gal_s[0] = msg->bgd_e1e5a;
  k->tgd_gal_s[1] = msg->bgd_e1e5b;
  k->crs = msg->c_rs;
  k->crc = msg->c_rc;
  k->cuc = msg->c_uc;
  k->cus = msg->c_us;
  k->cic = msg->c_ic;
  k->cis = msg->c_is;
  k->dn = msg->dn;
  k->m0 = msg->m0;
  k->ecc = msg->ecc;
  k->sqrta = msg->sqrta;
  k->omega0 = msg->omega0;
  k->omegadot = msg->omegadot;
  k->w = msg->w;
  k->inc = msg->inc;
  k->inc_dot = msg->inc_dot;
  k->af0 = msg->af0;
  k->af1 = msg->af1;
  k->af2 = msg->af2;
  k->toc.tow = msg->toc.tow;
  k->toc.wn = msg->toc.wn;
  k->iode = msg->iode;
  k->iodc = msg->iodc;
}

static void pack_ephemeris_gal(const ephemeris_t *e, msg_ephemeris_t *m) {
  const ephemeris_kepler_t *k = &e->kepler;
  msg_ephemeris_gal_t *msg = &m->gal;
  pack_ephemeris_common(e, &msg->common);
  msg->bgd_e1e5a = k->tgd_gal_s[0];
  msg->bgd_e1e5b = k->tgd_gal_s[1];
  msg->c_rs = k->crs;
  msg->c_rc = k->crc;
  msg->c_uc = k->cuc;
  msg->c_us = k->cus;
  msg->c_ic = k->cic;
  msg->c_is = k->cis;
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
  msg->af2 = k->af2;
  msg->toc.tow = round(k->toc.tow);
  msg->toc.wn = k->toc.wn;
  msg->iode = k->iode;
  msg->iodc = k->iodc;
}

static void unpack_ephemeris_sbas(const msg_ephemeris_t *m, ephemeris_t *e) {
  const msg_ephemeris_sbas_t *msg = &m->sbas;
  ephemeris_xyz_t *k = &e->xyz;
  unpack_ephemeris_common(&msg->common, e);
  k->pos[0] = msg->pos[0];
  k->pos[1] = msg->pos[1];
  k->pos[2] = msg->pos[2];
  k->vel[0] = msg->vel[0];
  k->vel[1] = msg->vel[1];
  k->vel[2] = msg->vel[2];
  k->acc[0] = msg->acc[0];
  k->acc[1] = msg->acc[1];
  k->acc[2] = msg->acc[2];
  k->a_gf0 = msg->a_gf0;
  k->a_gf1 = msg->a_gf1;
}

static void pack_ephemeris_sbas(const ephemeris_t *e, msg_ephemeris_t *m) {
  const ephemeris_xyz_t *k = &e->xyz;
  msg_ephemeris_sbas_t *msg = &m->sbas;
  pack_ephemeris_common(e, &msg->common);
  msg->pos[0] = k->pos[0];
  msg->pos[1] = k->pos[1];
  msg->pos[2] = k->pos[2];
  msg->vel[0] = k->vel[0];
  msg->vel[1] = k->vel[1];
  msg->vel[2] = k->vel[2];
  msg->acc[0] = k->acc[0];
  msg->acc[1] = k->acc[1];
  msg->acc[2] = k->acc[2];
  msg->a_gf0 = k->a_gf0;
  msg->a_gf1 = k->a_gf1;
}

static void unpack_ephemeris_glo(const msg_ephemeris_t *m, ephemeris_t *e) {
  const msg_ephemeris_glo_t *msg = &m->glo;
  ephemeris_glo_t *k = &e->glo;
  unpack_ephemeris_common(&msg->common, e);
  k->pos[0] = msg->pos[0];
  k->pos[1] = msg->pos[1];
  k->pos[2] = msg->pos[2];
  k->vel[0] = msg->vel[0];
  k->vel[1] = msg->vel[1];
  k->vel[2] = msg->vel[2];
  k->acc[0] = msg->acc[0];
  k->acc[1] = msg->acc[1];
  k->acc[2] = msg->acc[2];
  k->gamma = msg->gamma;
  k->tau = msg->tau;
  k->d_tau = msg->d_tau;
  k->iod = msg->iod;
  k->fcn = (u16)msg->fcn;

  if (!glo_slot_id_is_valid(msg->common.sid.sat)) {
    log_warn("Received GLO ephemeris from peer with invalid slot id %u",
             msg->common.sid.sat);
  } else if (!glo_fcn_is_valid((u16)msg->fcn)) {
    log_warn("Received GLO ephemeris from peer with invalid FCN %u", msg->fcn);
  } else {
    glo_map_set_slot_id(construct_mesid(msg->common.sid.code, (u16)msg->fcn),
                        msg->common.sid.sat);
  }
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
  msg->acc[0] = k->acc[0];
  msg->acc[1] = k->acc[1];
  msg->acc[2] = k->acc[2];
  msg->gamma = k->gamma;
  msg->tau = k->tau;
  msg->d_tau = k->d_tau;
  msg->iod = k->iod;
  msg->fcn = (u8)k->fcn;
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

        /* BDS */
        [CONSTELLATION_BDS] = {{SBP_MSG_EPHEMERIS_BDS,
                                sizeof(msg_ephemeris_bds_t)},
                               pack_ephemeris_bds,
                               unpack_ephemeris_bds,
                               {0}},

        /* GAL */
        [CONSTELLATION_GAL] = {{SBP_MSG_EPHEMERIS_GAL,
                                sizeof(msg_ephemeris_gal_t)},
                               pack_ephemeris_gal,
                               unpack_ephemeris_gal,
                               {0}},
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
 * This is helper function packs and sends gnss capabilities over SBP
 * @param[in] gc pointer to gnss capabilities
 */
void sbp_send_gnss_capb(const gnss_capb_t *gc) {
  assert(gc);
  msg_gnss_capb_t msg = {.t_nmct =
                             {/* TODO: set this as 0 for now, beccause
                               * functionality decodes tnmct is not available */
                              .tow = 0,
                              .wn = 0},
                         .gc = *gc};

  sbp_send_msg(SBP_MSG_GNSS_CAPB, sizeof(msg), (u8 *)&msg);
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

void sbp_unpack_glonass_biases_content(const msg_glo_biases_t msg,
                                       glo_biases_t *glonass_biases) {
  glonass_biases->mask = ((u8)msg.mask);
  glonass_biases->l1of_bias_m =
      ((double)msg.l1ca_bias / MSG_GLO_BIASES_MULTIPLIER);
  glonass_biases->l1p_bias_m =
      ((double)msg.l1p_bias / MSG_GLO_BIASES_MULTIPLIER);
  glonass_biases->l2of_bias_m =
      ((double)msg.l2ca_bias / MSG_GLO_BIASES_MULTIPLIER);
  glonass_biases->l2p_bias_m =
      ((double)msg.l2p_bias / MSG_GLO_BIASES_MULTIPLIER);
}

void sbp_pack_glonass_biases_content(const glo_biases_t glonass_biases,
                                     msg_glo_biases_t *msg) {
  msg->mask = ((u8)glonass_biases.mask);
  msg->l1ca_bias =
      (s16)round(glonass_biases.l1of_bias_m * MSG_GLO_BIASES_MULTIPLIER);
  msg->l1p_bias =
      (s16)round(glonass_biases.l1p_bias_m * MSG_GLO_BIASES_MULTIPLIER);
  msg->l2ca_bias =
      (s16)round(glonass_biases.l2of_bias_m * MSG_GLO_BIASES_MULTIPLIER);
  msg->l2p_bias =
      (s16)round(glonass_biases.l2p_bias_m * MSG_GLO_BIASES_MULTIPLIER);
}

u8 sbp_format_time_qual(u8 piksi_time_qual) {
  return (TIME_PROPAGATED <= piksi_time_qual) ? TIME_PROPAGATED : 0;
}

/** \} */
/** \} */
