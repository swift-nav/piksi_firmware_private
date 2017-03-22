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
#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include <math.h>

#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/observation.h>

#include "sbp.h"
#include "sbp_utils.h"
#include "timing.h"

/** \addtogroup sbp
 * \{ */

/** \defgroup sbp_utils SBP Utils
 * Convert to and from SBP message types and other useful functions.
 * \{ */

u32 round_tow_ms(double tow);
void round_time_nano(const gps_time_t *t_in, gps_time_nano_t *t_out);

sbp_gnss_signal_t sid_to_sbp(const gnss_signal_t from)
{
  sbp_gnss_signal_t sbp_sid = {
    .code = from.code,
    .sat = from.sat,
    .reserved = 0,
  };

  /* Maintain legacy compatibility with GPS PRN encoding. Sat values for other
   * constellations are "real" satellite identifiers.
   */
  if (sid_to_constellation(from) == CONSTELLATION_GPS) {
    sbp_sid.sat -= GPS_FIRST_PRN;
  }

  return sbp_sid;
}

gnss_signal_t sid_from_sbp(const sbp_gnss_signal_t from)
{
  gnss_signal_t sid = {
    .code = from.code,
    .sat = from.sat,
  };

  /* Maintain legacy compatibility with GPS PRN encoding. Sat values for other
   * constellations are "real" satellite identifiers.
   */
  if (code_valid(sid.code) &&
     (code_to_constellation(sid.code) == CONSTELLATION_GPS)) {
    sid.sat += GPS_FIRST_PRN;
  }

  return sid;
}

gnss_signal16_t sid_to_sbp16(const gnss_signal_t from)
{
  gnss_signal16_t sbp_sid = {
    .code = from.code,
    .sat = from.sat,
  };

  return sbp_sid;
}

gnss_signal_t sid_from_sbp16(const gnss_signal16_t from)
{
  gnss_signal_t sid = {
    .code = from.code,
    .sat = from.sat,
  };

  return sid;
}

void sbp_make_gps_time(msg_gps_time_t *t_out, const gps_time_t *t_in, u8 flags)
{
  /* TODO(Leith): SBP message should reuse the GPSTimeNano struct */
  gps_time_nano_t t_nano;
  round_time_nano(t_in, &t_nano);
  t_out->wn = t_nano.wn;
  t_out->tow = t_nano.tow;
  t_out->ns = t_nano.ns;
  t_out->flags = flags;
}

void sbp_make_utc_time(msg_utc_time_t *t_out, const gps_time_t *t_in, u8 flags)
{
  time_t unix_t;
  struct tm utc_time;

  unix_t = gps2time(t_in);
  gmtime_r(&unix_t, &utc_time);

  t_out->tow = round_tow_ms(t_in->tow);
  t_out->year = utc_time.tm_year;
  t_out->month = utc_time.tm_mon;
  t_out->day = utc_time.tm_mday;
  t_out->hours = utc_time.tm_hour;
  t_out->minutes = utc_time.tm_min;
  t_out->seconds = floor(utc_time.tm_sec);
  t_out->ns = (u32) round((utc_time.tm_sec - t_out->seconds)*1e9);
  t_out->flags = flags;
}

void sbp_make_dgnss_status(msg_dgnss_status_t *dgnss_status, u8 num_sats, double obs_latency, u8 flags){

  if( flags > DGNSS_POSITION) {
    dgnss_status->flags = 2;
  } else {
    dgnss_status->flags = 1;
  }
  dgnss_status->latency = MIN(round(10 * obs_latency), UINT16_MAX);
  dgnss_status->num_signals = num_sats;
}

void sbp_make_pos_llh_vect(msg_pos_llh_t *pos_llh, const double llh[3],
                           double h_accuracy, double v_accuracy,
                           const gps_time_t *gps_t, u8 n_sats_used, u8 flags)
{
  pos_llh->tow = round_tow_ms(gps_t->tow);
  pos_llh->lat = llh[0] * R2D;
  pos_llh->lon = llh[1] * R2D;
  pos_llh->height = llh[2];
  pos_llh->h_accuracy = MIN(round(1e3 * h_accuracy), UINT16_MAX);
  pos_llh->v_accuracy = MIN(round(1e3 * v_accuracy), UINT16_MAX);
  pos_llh->n_sats = n_sats_used;
  pos_llh->flags = flags;
}

void sbp_make_pos_ecef_vect(msg_pos_ecef_t *pos_ecef, const double ecef[3],
                            double accuracy, const gps_time_t *gps_t, u8 n_sats_used,
                            u8 flags)
{
  pos_ecef->tow = round_tow_ms(gps_t->tow);
  pos_ecef->x = ecef[0];
  pos_ecef->y = ecef[1];
  pos_ecef->z = ecef[2];
  pos_ecef->accuracy = MIN(round(1e3 * accuracy), UINT16_MAX);
  pos_ecef->n_sats = n_sats_used;
  pos_ecef->flags = flags;
}

void sbp_make_vel_ned(msg_vel_ned_t *vel_ned, const gnss_solution *soln, u8 flags)
{
  vel_ned->tow = round_tow_ms(soln->time.tow);
  vel_ned->n = round(soln->vel_ned[0] * 1e3);
  vel_ned->e = round(soln->vel_ned[1] * 1e3);
  vel_ned->d = round(soln->vel_ned[2] * 1e3);
  /* TODO: fill in accuracy fields. */
  vel_ned->h_accuracy = 0;
  vel_ned->v_accuracy = 0;
  vel_ned->n_sats = soln->n_sats_used;
  vel_ned->flags = flags;
}

void sbp_make_vel_ecef(msg_vel_ecef_t *vel_ecef, const gnss_solution *soln, u8 flags)
{
  vel_ecef->tow = round_tow_ms(soln->time.tow);
  vel_ecef->x = round(soln->vel_ecef[0] * 1e3);
  vel_ecef->y = round(soln->vel_ecef[1] * 1e3);
  vel_ecef->z = round(soln->vel_ecef[2] * 1e3);
  /* TODO: fill in accuracy field. */
  vel_ecef->accuracy = 0;
  vel_ecef->n_sats = soln->n_sats_used;
  vel_ecef->flags = flags;
}

void sbp_make_dops(msg_dops_t *dops_out, const dops_t *dops_in, const u32 tow, u8 flags)
{
  dops_out->tow = tow;
  dops_out->pdop = round(dops_in->pdop * 100);
  dops_out->gdop = round(dops_in->gdop * 100);
  dops_out->tdop = round(dops_in->tdop * 100);
  dops_out->hdop = round(dops_in->hdop * 100);
  dops_out->vdop = round(dops_in->vdop * 100);
  dops_out->flags = flags;
}

void sbp_make_baseline_ecef(msg_baseline_ecef_t *baseline_ecef, const gps_time_t *t,
                            u8 n_sats, const double b_ecef[3], double accuracy,
                            u8 flags) {
  baseline_ecef->tow = round_tow_ms(t->tow);
  baseline_ecef->x = round(1e3 * b_ecef[0]);
  baseline_ecef->y = round(1e3 * b_ecef[1]);
  baseline_ecef->z = round(1e3 * b_ecef[2]);
  baseline_ecef->accuracy = MIN(round(1e3 * accuracy), UINT16_MAX);
  baseline_ecef->n_sats = n_sats;
  baseline_ecef->flags = flags;
}

void sbp_make_baseline_ned(msg_baseline_ned_t *baseline_ned, const gps_time_t *t,
                           u8 n_sats, const double b_ned[3], double h_accuracy,
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

void sbp_make_heading(msg_baseline_heading_t *baseline_heading, const gps_time_t *t,
                      const double heading, u8 n_sats, u8 flags) {
    baseline_heading->tow = round_tow_ms(t->tow);
    baseline_heading->heading = round(heading * 1e3);
    baseline_heading->n_sats = n_sats;
    baseline_heading->flags = flags;
}

void sbp_make_age_corrections(msg_age_corrections_t *age_corrections, const gps_time_t *t, double propagation_time){
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
    msg.object_sid = sid_to_sbp16(*object_sid);
  }

  if (NULL != src_sid) {
    msg.src_sid = sid_to_sbp16(*src_sid);
  }

  msg.original_sender = sender;

  sbp_send_msg(SBP_MSG_NDB_EVENT, sizeof(msg), (u8 *)&msg);
}

void unpack_obs_header(const observation_header_t *msg, gps_time_t* t, u8* total, u8* count)
{
  t->wn  = msg->t.wn;
  t->tow = ((double)msg->t.tow) / 1e3 +
           ((double)msg->t.ns) / 1e9;
  normalize_gps_time(t);
  *total = (msg->n_obs >> MSG_OBS_HEADER_SEQ_SHIFT);
  *count = (msg->n_obs & MSG_OBS_HEADER_SEQ_MASK);
}

void pack_obs_header(const gps_time_t *t, u8 total, u8 count, observation_header_t *msg)
{
  round_time_nano(t, &msg->t);
  msg->n_obs = ((total << MSG_OBS_HEADER_SEQ_SHIFT) |
                 (count & MSG_OBS_HEADER_SEQ_MASK));
}

u8 nm_flags_to_sbp(nav_meas_flags_t from)
{
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
  return to;
}

nav_meas_flags_t nm_flags_from_sbp(u8 from)
{
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
  return to;
}

void unpack_obs_content(const packed_obs_content_t *msg, double *P, double *L,
                        double *D, double *cn0, double *lock_time,
                        nav_meas_flags_t *flags, gnss_signal_t *sid)
{
  *P   = ((double)msg->P) / MSG_OBS_P_MULTIPLIER;
  *L   = -(((double)msg->L.i) + (((double)msg->L.f) / MSG_OBS_LF_MULTIPLIER));
  *D   = (((double)msg->D.i) + (((double)msg->D.f) / MSG_OBS_DF_MULTIPLIER));
  *cn0 = ((double)msg->cn0) / MSG_OBS_CN0_MULTIPLIER;
  *lock_time = decode_lock_time(msg->lock);
  *flags = nm_flags_from_sbp(msg->flags);
  if (msg->cn0 != 0) {
    *flags |= NAV_MEAS_FLAG_CN0_VALID;
  }
  *sid = sid_from_sbp16(msg->sid);
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
s8 pack_obs_content(double P, double L, double D, double cn0, double lock_time,
                    nav_meas_flags_t flags, gnss_signal_t sid,
                    packed_obs_content_t *msg)
{
  s64 P_fp = llround(P * MSG_OBS_P_MULTIPLIER);
  if (P < 0 || P_fp > UINT32_MAX) {
    log_error("observation message packing: P integer overflow (%f)", P);
    return -1;
  }

  msg->P = P_fp;

  double Li = floor(-L);
  if (Li < INT32_MIN || Li > INT32_MAX) {
    log_error("observation message packing: L integer overflow (%f)", L);
    return -1;
  }

  double Lf = -L - Li;

  msg->L.i = Li;
  msg->L.f = Lf * MSG_OBS_LF_MULTIPLIER;

  double Di = floor(D);
  if (Di < INT16_MIN || Di > INT16_MAX) {
    log_error("observation message packing: D integer overflow (%f)", D);
    return -1;
  }

  double Df = D - Di;

  msg->D.i = Di;
  msg->D.f = Df * MSG_OBS_DF_MULTIPLIER;

  if (0 != (flags & NAV_MEAS_FLAG_CN0_VALID)) {
    s32 cn0_fp = lround(cn0 * MSG_OBS_CN0_MULTIPLIER);
    if (cn0 < 0 || cn0_fp > UINT8_MAX) {
      log_error("observation message packing: C/N0 integer overflow (%f)", cn0);
      return -1;
    }

    msg->cn0 = cn0_fp;
  } else {
    msg->cn0 = 0;
  }

  msg->lock = encode_lock_time(lock_time);

  msg->flags = nm_flags_to_sbp(flags);

  msg->sid = sid_to_sbp16(sid);

  return 0;
}

static void unpack_ephemeris_common(const ephemeris_common_content_t *common,
                                    ephemeris_t *e)
{
  e->toe.tow          = common->toe.tow;
  e->toe.wn           = common->toe.wn;
  e->valid            = common->valid;
  e->health_bits      = common->health_bits;
  e->sid              = sid_from_sbp(common->sid);
  e->fit_interval     = common->fit_interval;
  e->ura              = common->ura;
}

static void pack_ephemeris_common(const ephemeris_t *e,
                                  ephemeris_common_content_t *common)
{
  common->toe.tow      = e->toe.tow;
  common->toe.wn       = e->toe.wn;
  common->valid        = e->valid;
  common->health_bits  = e->health_bits;
  common->sid          = sid_to_sbp(e->sid);
  common->fit_interval = e->fit_interval;
  common->ura          = e->ura;
}

static void unpack_ephemeris_gps(const msg_ephemeris_t *m, ephemeris_t *e)
{
  const msg_ephemeris_gps_t *msg = &m->gps;
  unpack_ephemeris_common(&msg->common, e);
  e->kepler.tgd       = msg->tgd;
  e->kepler.crs       = msg->c_rs;
  e->kepler.crc       = msg->c_rc;
  e->kepler.cuc       = msg->c_uc;
  e->kepler.cus       = msg->c_us;
  e->kepler.cic       = msg->c_ic;
  e->kepler.cis       = msg->c_is;
  e->kepler.dn        = msg->dn;
  e->kepler.m0        = msg->m0;
  e->kepler.ecc       = msg->ecc;
  e->kepler.sqrta     = msg->sqrta;
  e->kepler.omega0    = msg->omega0;
  e->kepler.omegadot  = msg->omegadot;
  e->kepler.w         = msg->w;
  e->kepler.inc       = msg->inc;
  e->kepler.inc_dot   = msg->inc_dot;
  e->kepler.af0       = msg->af0;
  e->kepler.af1       = msg->af1;
  e->kepler.af2       = msg->af2;
  e->kepler.toc.tow   = msg->toc.tow;
  e->kepler.toc.wn    = msg->toc.wn;
  e->kepler.iode      = msg->iode;
  e->kepler.iodc      = msg->iodc;
}

static void pack_ephemeris_gps(const ephemeris_t *e, msg_ephemeris_t *m)
{
  msg_ephemeris_gps_t *msg = &m->gps;
  pack_ephemeris_common(e, &msg->common);
  msg->tgd            = e->kepler.tgd;
  msg->c_rs           = e->kepler.crs;
  msg->c_rc           = e->kepler.crc;
  msg->c_uc           = e->kepler.cuc;
  msg->c_us           = e->kepler.cus;
  msg->c_ic           = e->kepler.cic;
  msg->c_is           = e->kepler.cis;
  msg->dn             = e->kepler.dn;
  msg->m0             = e->kepler.m0;
  msg->ecc            = e->kepler.ecc;
  msg->sqrta          = e->kepler.sqrta;
  msg->omega0         = e->kepler.omega0;
  msg->omegadot       = e->kepler.omegadot;
  msg->w              = e->kepler.w;
  msg->inc            = e->kepler.inc;
  msg->inc_dot        = e->kepler.inc_dot;
  msg->af0            = e->kepler.af0;
  msg->af1            = e->kepler.af1;
  msg->af2            = e->kepler.af2;
  msg->toc.tow        = e->kepler.toc.tow;
  msg->toc.wn         = e->kepler.toc.wn;
  msg->iode           = e->kepler.iode;
  msg->iodc           = e->kepler.iodc;
}

static void unpack_ephemeris_sbas(const msg_ephemeris_t *m, ephemeris_t *e)
{
  const msg_ephemeris_sbas_t *msg = &m->sbas;
  unpack_ephemeris_common(&msg->common, e);
  memcpy(e->xyz.pos, msg->pos, sizeof(e->xyz.pos));
  memcpy(e->xyz.vel, msg->vel, sizeof(e->xyz.vel));
  memcpy(e->xyz.acc, msg->acc, sizeof(e->xyz.acc));
  e->xyz.a_gf0        = msg->a_gf0;
  e->xyz.a_gf1        = msg->a_gf1;
}

static void pack_ephemeris_sbas(const ephemeris_t *e, msg_ephemeris_t *m)
{
  msg_ephemeris_sbas_t *msg = &m->sbas;
  pack_ephemeris_common(e, &msg->common);
  memcpy(msg->pos, e->xyz.pos, sizeof(e->xyz.pos));
  memcpy(msg->vel, e->xyz.vel, sizeof(e->xyz.vel));
  memcpy(msg->acc, e->xyz.acc, sizeof(e->xyz.acc));
  msg->a_gf0          = e->xyz.a_gf0;
  msg->a_gf1          = e->xyz.a_gf1;
}

static void unpack_ephemeris_glo(const msg_ephemeris_t *m, ephemeris_t *e)
{
  const msg_ephemeris_glo_t *msg = &m->glo;
  unpack_ephemeris_common(&msg->common, e);
  memcpy(e->glo.pos, msg->pos, sizeof(e->glo.pos));
  memcpy(e->glo.vel, msg->vel, sizeof(e->glo.vel));
  memcpy(e->glo.acc, msg->acc, sizeof(e->glo.acc));
  e->glo.gamma        = msg->gamma;
  e->glo.tau          = msg->tau;
}

static void pack_ephemeris_glo(const ephemeris_t *e, msg_ephemeris_t *m)
{
  msg_ephemeris_glo_t *msg = &m->glo;
  pack_ephemeris_common(e, &msg->common);
  memcpy(msg->pos, e->glo.pos, sizeof(msg->pos));
  memcpy(msg->vel, e->glo.vel, sizeof(msg->vel));
  memcpy(msg->acc, e->glo.acc, sizeof(msg->acc));
  msg->gamma          = e->glo.gamma;
  msg->tau            = e->glo.tau;
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
  {{SBP_MSG_EPHEMERIS_GPS, sizeof(msg_ephemeris_gps_t)},
   pack_ephemeris_gps, unpack_ephemeris_gps, {0}},

  /* SBAS */
  {{SBP_MSG_EPHEMERIS_SBAS, sizeof(msg_ephemeris_sbas_t)},
   pack_ephemeris_sbas, unpack_ephemeris_sbas, {0}},

  /* GLO */
  {{SBP_MSG_EPHEMERIS_GLO, sizeof(msg_ephemeris_glo_t)},
   pack_ephemeris_glo, unpack_ephemeris_glo, {0}}
};

void unpack_ephemeris(const msg_ephemeris_t *msg, ephemeris_t *e)
{
  constellation_t c = sid_to_constellation(e->sid);

  assert(NULL != ephe_type_table[c].unpack);

  ephe_type_table[c].unpack(msg, e);
}

msg_info_t pack_ephemeris(const ephemeris_t *e, msg_ephemeris_t *msg)
{
  constellation_t c = sid_to_constellation(e->sid);

  assert(NULL != ephe_type_table[c].pack);

  ephe_type_table[c].pack(e, msg);

  return ephe_type_table[c].msg_info;
}

void sbp_ephe_reg_cbks(void (*ephemeris_msg_callback)(u16, u8, u8*, void*))
{
  assert(ARRAY_SIZE(ephe_type_table) == CONSTELLATION_COUNT);

  for (u8 i = 0; i < ARRAY_SIZE(ephe_type_table); i++) {
    /* check if type is valid */
    if (TYPE_TABLE_INVALID_MSG_ID == ephe_type_table[i].msg_info.msg_id) {
      continue;
    }

    sbp_register_cbk(
      ephe_type_table[i].msg_info.msg_id,
      ephemeris_msg_callback,
      &ephe_type_table[i].cbk_node
    );
  }
}

/**
 * This is helper function packs and sends iono parameters over SBP
 * @param[in] iono pointer to Iono parameters
 */
void sbp_send_iono(const ionosphere_t *iono)
{
  msg_iono_t msg_iono = {
    .t_nmct = {
      /* TODO: set this as 0 for now, beccause functionality
       * decodes tnmct is not available */
      .tow = 0,
      .wn = 0
    },
    .a0 = iono->a0,
    .a1 = iono->a1,
    .a2 = iono->a2,
    .a3 = iono->a3,
    .b0 = iono->b0,
    .b1 = iono->b1,
    .b2 = iono->b2,
    .b3 = iono->b3
  };

  /* send data over sbp */
  sbp_send_msg(SBP_MSG_IONO,
               sizeof(msg_iono_t),
               (u8 *)&msg_iono);
}

/**
 * This is helper function packs and sends L2C capabilities over SBP
 * @param[in] l2c_cap pointer to L2C capabilities mask
 */
void sbp_send_l2c_capabilities(const u32 *l2c_cap)
{
  msg_sv_configuration_gps_t msg_l2c = {
    .t_nmct = {
      /* TODO: set this as 0 for now, beccause functionality
       * decodes tnmct is not available */
      .tow = 0,
      .wn = 0
    },
    .l2c_mask = *l2c_cap
  };

  /* send data over sbp */
  sbp_send_msg(SBP_MSG_SV_CONFIGURATION_GPS,
               sizeof(msg_sv_configuration_gps_t),
               (u8 *)&msg_l2c);
}

/**
 * This is helper function packs and sends Group delay over SBP
 * @param[in] cnav pointer to GPS CNAV message structure
 */
void sbp_send_group_delay(const cnav_msg_t *cnav)
{
  gps_time_t t = get_current_time();
  msg_group_delay_t msg_cnav = {
    .t_op = {
      /* Convert from 6 seconds unit */
      .tow = (u32)(cnav->tow * 6),
      .wn = t.wn
    },
    .prn = cnav->prn,
    .valid = cnav->data.type_30.tgd_valid          |
             cnav->data.type_30.isc_l2c_valid << 1 |
             cnav->data.type_30.isc_l1ca_valid << 2,
    .tgd = cnav->data.type_30.tgd,
    .isc_l1ca = cnav->data.type_30.isc_l1ca,
    .isc_l2c = cnav->data.type_30.isc_l2c
  };

  /* send data over sbp */
  sbp_send_msg(SBP_MSG_GROUP_DELAY,
               sizeof(msg_group_delay_t),
               (u8 *)&msg_cnav);
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
 * Helper function for converting GPS tiem to integer milliseconds with
 * nanosecond remainder, taking care of week roll-over
 * @param[in] t_in GPS time
 * @param[out] t_out SBP time
*/
void round_time_nano(const gps_time_t *t_in, gps_time_nano_t *t_out) {
  t_out->wn = t_in->wn;
  t_out->tow = round(t_in->tow * 1e3);
  t_out->ns = round((t_in->tow - t_out->tow / 1e3) *
              1e9);
  /* week roll-over */
  if (t_out->tow >= WEEK_MS) {
    t_out->wn++;
    t_out->tow -= WEEK_MS;
  }
}

static void pack_almanac_common(const almanac_t *a,
                                almanac_common_content_t *common)
{
  common->toa.tow      = a->toa.tow;
  common->toa.wn       = a->toa.wn;
  common->valid        = a->valid;
  common->health_bits  = a->health_bits;
  common->sid          = sid_to_sbp(a->sid);
  common->fit_interval = a->fit_interval;
  common->ura          = a->ura;
}

static void pack_almanac_gps(const almanac_t *a, msg_almanac_t *m)
{
  msg_almanac_gps_t *msg = &m->gps;
  pack_almanac_common(a, &msg->common);
  msg->m0             = a->kepler.m0;
  msg->ecc            = a->kepler.ecc;
  msg->sqrta          = a->kepler.sqrta;
  msg->omega0         = a->kepler.omega0;
  msg->omegadot       = a->kepler.omegadot;
  msg->w              = a->kepler.w;
  msg->inc            = a->kepler.inc;
  msg->af0            = a->kepler.af0;
  msg->af1            = a->kepler.af1;
}

static void pack_almanac_glo(const almanac_t *a, msg_almanac_t *m)
{
  msg_almanac_glo_t *msg = &m->glo;
  pack_almanac_common(a, &msg->common);
  msg->lambda_na      = a->glo.lambda;
  msg->t_lambda_na    = a->glo.t_lambda;
  msg->i              = a->glo.i;
  msg->t              = a->glo.t;
  msg->t_dot          = a->glo.t_dot;
  msg->epsilon        = a->glo.epsilon;
  msg->omega          = a->glo.omega;
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
   pack_almanac_gps, NULL, {0}},

  /* SBAS almanac not supported at the moment */
  {{TYPE_TABLE_INVALID_MSG_ID, 0}, NULL, NULL, {0}},

  /* GLO */
  {{SBP_MSG_ALMANAC_GLO, sizeof(msg_almanac_glo_t)},
   pack_almanac_glo, NULL, {0}}
};

void unpack_almanac(const msg_almanac_t *msg, almanac_t *a)
{
  constellation_t c = sid_to_constellation(a->sid);

  assert(NULL != alma_type_table[c].unpack);

  alma_type_table[c].unpack(msg, a);
}

msg_info_t pack_almanac(const almanac_t *a, msg_almanac_t *msg)
{
  constellation_t c = sid_to_constellation(a->sid);

  assert(NULL != alma_type_table[c].pack);

  alma_type_table[c].pack(a, msg);

  return alma_type_table[c].msg_info;
}

void sbp_alma_reg_cbks(void (*almanac_msg_callback)(u16, u8, u8*, void*))
{
  assert(ARRAY_SIZE(alma_type_table) == CONSTELLATION_COUNT);

  for (u8 i = 0; i < ARRAY_SIZE(alma_type_table); i++) {
    /* check if type is valid */
    if (TYPE_TABLE_INVALID_MSG_ID == alma_type_table[i].msg_info.msg_id) {
      continue;
    }

    sbp_register_cbk(
      alma_type_table[i].msg_info.msg_id,
      almanac_msg_callback,
      &alma_type_table[i].cbk_node
    );
  }
}

/** \} */
/** \} */
