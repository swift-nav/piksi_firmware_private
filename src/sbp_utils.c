/*
 * Copyright (C) 2014, 2016 Swift Navigation Inc.
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

#include "sbp.h"
#include "sbp_utils.h"
#include "timing.h"

/** \addtogroup sbp
 * \{ */

/** \defgroup sbp_utils SBP Utils
 * Convert to and from SBP message types and other useful functions.
 * \{ */

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
  if (sid_to_constellation(from) == CONSTELLATION_GPS)
    sbp_sid.sat -= GPS_FIRST_PRN;

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

void sbp_make_gps_time(msg_gps_time_dep_a_t *t_out, const gps_time_t *t_in, u8 flags)
{
  t_out->wn = t_in->wn;
  t_out->tow = round(t_in->tow * 1e3);
  /* week roll-over */
  if (t_out->tow >= WEEK_MS) {
    t_out->wn++;
    t_out->tow -= WEEK_MS;
  }
  t_out->ns = round((t_in->tow - t_out->tow*1e-3) * 1e9);
  t_out->flags = flags;
}

void sbp_make_pos_llh(msg_pos_llh_dep_a_t *pos_llh, const gnss_solution *soln, u8 flags)
{
  pos_llh->tow = round_tow_ms(soln->time.tow);
  pos_llh->lat = soln->pos_llh[0] * R2D;
  pos_llh->lon = soln->pos_llh[1] * R2D;
  pos_llh->height = soln->pos_llh[2];
  /* TODO: fill in accuracy fields. */
  pos_llh->h_accuracy = 0;
  pos_llh->v_accuracy = 0;
  pos_llh->n_sats = soln->n_sats_used;
  pos_llh->flags = flags;
}

void sbp_make_pos_llh_vect(msg_pos_llh_dep_a_t *pos_llh, const double llh[3],
                           double h_accuracy, double v_accuracy,
                           const gps_time_t *gps_t, u8 n_sats_used, u8 flags)
{
  pos_llh->tow = round_tow_ms(gps_t->tow);
  pos_llh->lat = llh[0] * R2D;
  pos_llh->lon = llh[1] * R2D;
  pos_llh->height = llh[2];
  pos_llh->h_accuracy = round(1e3 * h_accuracy);
  pos_llh->v_accuracy = round(1e3 * v_accuracy);
  pos_llh->n_sats = n_sats_used;
  pos_llh->flags = flags;
}

void sbp_make_pos_ecef(msg_pos_ecef_dep_a_t *pos_ecef, const gnss_solution *soln, u8 flags)
{
  pos_ecef->tow = round_tow_ms(soln->time.tow);
  pos_ecef->x = soln->pos_ecef[0];
  pos_ecef->y = soln->pos_ecef[1];
  pos_ecef->z = soln->pos_ecef[2];
  /* this is the estimate of 3D position standard deviation assuming
   * the pseudorange weighting model is correct */
  pos_ecef->accuracy = round(1000.0 *
                  sqrt(soln->err_cov[0] + soln->err_cov[3] + soln->err_cov[5]));
  pos_ecef->n_sats = soln->n_sats_used;
  pos_ecef->flags = flags;
}

void sbp_make_pos_ecef_vect(msg_pos_ecef_dep_a_t *pos_ecef, const double ecef[3],
                            double accuracy, const gps_time_t *gps_t, u8 n_sats_used,
                            u8 flags)
{
  pos_ecef->tow = round_tow_ms(gps_t->tow);
  pos_ecef->x = ecef[0];
  pos_ecef->y = ecef[1];
  pos_ecef->z = ecef[2];
  pos_ecef->accuracy = round(1e3 * accuracy);
  pos_ecef->n_sats = n_sats_used;
  pos_ecef->flags = flags;
}

void sbp_make_vel_ned(msg_vel_ned_dep_a_t *vel_ned, const gnss_solution *soln, u8 flags)
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

void sbp_make_vel_ecef(msg_vel_ecef_dep_a_t *vel_ecef, const gnss_solution *soln, u8 flags)
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

void sbp_make_dops(msg_dops_dep_a_t *dops_out, const dops_t *dops_in, const gps_time_t *t)
{
  dops_out->tow = round_tow_ms(t->tow);
  dops_out->pdop = round(dops_in->pdop * 100);
  dops_out->gdop = round(dops_in->gdop * 100);
  dops_out->tdop = round(dops_in->tdop * 100);
  dops_out->hdop = round(dops_in->hdop * 100);
  dops_out->vdop = round(dops_in->vdop * 100);
}

void sbp_make_baseline_ecef(msg_baseline_ecef_dep_a_t *baseline_ecef, const gps_time_t *t,
                            u8 n_sats, const double b_ecef[3], double accuracy,
                            u8 flags) {
  baseline_ecef->tow = round_tow_ms(t->tow);
  baseline_ecef->x = round(1e3 * b_ecef[0]);
  baseline_ecef->y = round(1e3 * b_ecef[1]);
  baseline_ecef->z = round(1e3 * b_ecef[2]);
  baseline_ecef->accuracy = round(1e3 * accuracy);
  baseline_ecef->n_sats = n_sats;
  baseline_ecef->flags = flags;
}

void sbp_make_baseline_ned(msg_baseline_ned_dep_a_t *baseline_ned, const gps_time_t *t,
                           u8 n_sats, const double b_ned[3], double h_accuracy,
                           double v_accuracy, u8 flags) {
  baseline_ned->tow = round_tow_ms(t->tow);
  baseline_ned->n = round(1e3 * b_ned[0]);
  baseline_ned->e = round(1e3 * b_ned[1]);
  baseline_ned->d = round(1e3 * b_ned[2]);
  baseline_ned->h_accuracy = round(1e3 * h_accuracy);
  baseline_ned->v_accuracy = round(1e3 * v_accuracy);
  baseline_ned->n_sats = n_sats;
  baseline_ned->flags = flags;
}

void sbp_make_heading(msg_baseline_heading_dep_a_t *baseline_heading, const gps_time_t *t,
                      const double heading, u8 n_sats, u8 flags) {
    baseline_heading->tow = round_tow_ms(t->tow);
    baseline_heading->heading = (u32)round(heading * 1e3);
    baseline_heading->n_sats = n_sats;
    baseline_heading->flags = flags;
}

void unpack_obs_header(const observation_header_dep_t *msg, gps_time_t* t, u8* total, u8* count)
{
  t->tow = ((double)msg->t.tow) / MSG_OBS_TOW_MULTIPLIER;
  t->wn  = msg->t.wn;
  *total = (msg->n_obs >> MSG_OBS_HEADER_SEQ_SHIFT);
  *count = (msg->n_obs & MSG_OBS_HEADER_SEQ_MASK);
}

void pack_obs_header(const gps_time_t *t, u8 total, u8 count, observation_header_dep_t *msg)
{
  msg->t.tow = (u32)round(t->tow * MSG_OBS_TOW_MULTIPLIER);
  msg->t.wn  = t->wn;
  msg->n_obs = ((total << MSG_OBS_HEADER_SEQ_SHIFT) |
                 (count & MSG_OBS_HEADER_SEQ_MASK));
}

void unpack_obs_content(const packed_obs_content_dep_c_t *msg, double *P, double *L,
                        double *cn0, u16 *lock_counter, gnss_signal_t *sid)
{
  *P   = ((double)msg->P) / MSG_OBS_P_MULTIPLIER;
  *L   = -(((double)msg->L.i) + (((double)msg->L.f) / MSG_OSB_LF_MULTIPLIER));
  *cn0 = ((double)msg->cn0) / MSG_OBS_CN0_MULTIPLIER;
  *lock_counter = ((u16)msg->lock);
  *sid = sid_from_sbp(msg->sid);
}

/** Pack GPS observables into a `msg_obs_content_t` struct.
 * For use in constructing a `MSG_NEW_OBS` SBP message.
 *
 * \param P Pseudorange in meters
 * \param L Carrier phase in cycles
 * \param cn0 Signal-to-noise ratio
 * \param lock_counter Lock counter is an arbitrary integer that should change
 *                     if the carrier phase ambiguity is ever reset
 * \param sid Signal ID
 * \param msg Pointer to a `msg_obs_content_t` struct to fill out
 * \return `0` on success or `-1` on an overflow error
 */
s8 pack_obs_content(double P, double L, double cn0, u16 lock_counter,
                    gnss_signal_t sid, packed_obs_content_dep_c_t *msg)
{

  s64 P_fp = llround(P * MSG_OBS_P_MULTIPLIER);
  if (P < 0 || P_fp > UINT32_MAX) {
    log_error("observation message packing: P integer overflow (%f)", P);
    return -1;
  }

  msg->P = (u32)P_fp;

  double Li = floor(-L);
  if (Li < INT32_MIN || Li > INT32_MAX) {
    log_error("observation message packing: L integer overflow (%f)", L);
    return -1;
  }

  double Lf = -L - Li;

  msg->L.i = (s32) Li;
  msg->L.f = (u8) (Lf * MSG_OSB_LF_MULTIPLIER);

  s32 cn0_fp = lround(cn0 * MSG_OBS_CN0_MULTIPLIER);
  if (cn0 < 0 || cn0_fp > UINT8_MAX) {
    log_error("observation message packing: C/N0 integer overflow (%f)", cn0);
    return -1;
  }

  msg->cn0 = (u8)cn0_fp;

  msg->lock = lock_counter;

  msg->sid = sid_to_sbp(sid);

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

typedef void (*pack_ephe_func)(const ephemeris_t *, msg_ephemeris_t *);
typedef void (*unpack_ephe_func)(const msg_ephemeris_t *, ephemeris_t *);

#define EPHE_TYPE_COUNT 3

typedef struct {
  const msg_ephemeris_info_t msg_info;
  const pack_ephe_func pack;
  const unpack_ephe_func unpack;
  sbp_msg_callbacks_node_t cbk_node;
} ephe_type_table_element_t;
static ephe_type_table_element_t ephe_type_table[EPHE_TYPE_COUNT] = {
  {{SBP_MSG_EPHEMERIS_GPS, sizeof(msg_ephemeris_gps_t)},
   pack_ephemeris_gps, unpack_ephemeris_gps, {0}},
  {{SBP_MSG_EPHEMERIS_SBAS, sizeof(msg_ephemeris_sbas_t)},
   pack_ephemeris_sbas, unpack_ephemeris_sbas, {0}},
  {{SBP_MSG_EPHEMERIS_GLO, sizeof(msg_ephemeris_glo_t)},
   pack_ephemeris_glo, unpack_ephemeris_glo, {0}}
};

void unpack_ephemeris(const msg_ephemeris_t *msg, ephemeris_t *e)
{
  constellation_t c = sid_to_constellation(e->sid);

  assert(c < EPHE_TYPE_COUNT);
  assert(NULL != ephe_type_table[c].unpack);

  ephe_type_table[c].unpack(msg, e);
}

msg_ephemeris_info_t pack_ephemeris(const ephemeris_t *e, msg_ephemeris_t *msg)
{
  constellation_t c = sid_to_constellation(e->sid);

  assert(c < EPHE_TYPE_COUNT);
  assert(NULL != ephe_type_table[c].pack);

  ephe_type_table[c].pack(e, msg);

  return ephe_type_table[c].msg_info;
}

void sbp_ephe_reg_cbks(void (*ephemeris_msg_callback)(u16, u8, u8*, void*))
{
  if (EPHE_TYPE_COUNT != CONSTELLATION_COUNT)
    log_warn("EPHE_TYPE_COUNT != CONSTELLATION_COUNT");

  for (u8 i = 0; i < EPHE_TYPE_COUNT; i++) {
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
 * @param[out] tow_ms Time-of-week in milliseconds
*/
u32 round_tow_ms(double tow) {
  /* week roll-over */
  u32 tow_ms = round(tow * 1e3);
  while (tow_ms >= WEEK_MS) {
    tow_ms -= WEEK_MS;
  }
  return tow_ms;
}

/** \} */
/** \} */
