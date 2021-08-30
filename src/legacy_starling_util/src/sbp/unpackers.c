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

#include <assert.h>
#include <legacy_starling_util/sbp/misc.h>
#include <legacy_starling_util/sbp/unpackers.h>
#include <string.h>
#include <swiftnav/glo_map.h>
#include <swiftnav/memcpy_s.h>

#define IMU_TYPE_BMI160 0
#define IMU_TYPE_ASM330 1
#define IMU_TYPE_ANDROID 2

static const double BMI160_ACCL_SF_MPSPS_LUT[] = {
    (2.0 * 9.80665 / 32768.0), /* 2g  */
    (4.0 * 9.80665 / 32768.0), /* 4g  */
    (8.0 * 9.80665 / 32768.0), /* 8g  */
    (16.0 * 9.80665 / 32768.0) /* 16g */
};

/* see datasheet page 14. scaling is NOT over entire range if int16 */
/* https://www.st.com/resource/en/datasheet/asm330lhh.pdf */

static const double ASM330_ACCL_SF_MPSPS_LUT[] = {
    (0.061 * 9.80665 / 1000.0), /* 2g  */
    (0.122 * 9.80665 / 1000.0), /* 4g  */
    (0.244 * 9.80665 / 1000.0), /* 8g  */
    (0.488 * 9.80665 / 1000.0)  /* 16g */
};

/* For Android smartphones we chose the same scaling values as the BMI160 */

static const double ANDROID_ACCL_SF_MPSPS_LUT[] = {
    (2.0 * 9.80665 / 32768.0), /* 2g  */
    (4.0 * 9.80665 / 32768.0), /* 4g  */
    (8.0 * 9.80665 / 32768.0), /* 8g  */
    (16.0 * 9.80665 / 32768.0) /* 16g */
};

static const double BMI160_GYRO_SF_DEGPS_LUT[] = {
    (2000.0 / 32768.0), /* 2000 deg/s */
    (1000.0 / 32768.0), /* 1000 deg/s */
    (500.0 / 32768.0),  /* 500 deg/s */
    (250.0 / 32768.0),  /* 250 deg/s */
    (125.0 / 32768.0)   /* 125 deg/s */
};

/* see datasheet page 14. scaling is NOT over entire range if int16 */
/* https://www.st.com/resource/en/datasheet/asm330lhh.pdf */

static const double ASM330_GYRO_SF_DEGPS_LUT[] = {
    /*4000 deg/s not supported */
    (70.0 / 1000.0), /* 2000 deg/s */
    (35.0 / 1000.0), /* 1000 deg/s */
    (17.5 / 1000.0), /* 500 deg/s */
    (8.75 / 1000.0), /* 250 deg/s */
    (4.37 / 1000.0)  /* 125 deg/s */
};

/* For Android smartphones we chose the same scaling values as the BMI160 */

static const double ANDROID_GYRO_SF_DEGPS_LUT[] = {
    (2000.0 / 32768.0), /* 2000 deg/s */
    (1000.0 / 32768.0), /* 1000 deg/s */
    (500.0 / 32768.0),  /* 500 deg/s */
    (250.0 / 32768.0),  /* 250 deg/s */
    (125.0 / 32768.0)   /* 125 deg/s */
};

static bool sbp_gps_time_valid(const sbp_gps_time_t *t) {
  /* all-zero time struct is invalid */
  return (t->wn != 0 || t->tow != 0 || t->ns_residual != 0);
}

void unpack_obs_header(const observation_header_t *msg,
                       gps_time_t *t,
                       u8 *total,
                       u8 *count) {
  if (sbp_gps_time_valid(&msg->t)) {
    t->wn = msg->t.wn;
    t->tow =
        ((double)msg->t.tow) / SECS_MS + ((double)msg->t.ns_residual) / SECS_NS;
    normalize_gps_time(t);
  } else {
    *t = GPS_TIME_UNKNOWN;
  }
  *total = (msg->n_obs >> MSG_OBS_HEADER_SEQ_SHIFT);
  *count = (msg->n_obs & MSG_OBS_HEADER_SEQ_MASK);
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
  *cn0 = ((float)msg->cn0) / MSG_OBS_CN0_MULTIPLIER;
  *lock_time = (float)decode_lock_time(msg->lock);
  *flags = nm_flags_from_sbp(msg->flags);
  if (msg->cn0 != 0) {
    *flags |= NAV_MEAS_FLAG_CN0_VALID;
  }
  *sid = sid_from_sbp(msg->sid);
}

void unpack_obs_content_into_starling_obs(const packed_obs_content_t *msg,
                                          starling_obs_t *dst) {
  double D;
  double cn0;
  double lock_time;

  unpack_obs_content(msg,
                     &dst->pseudorange,
                     &dst->carrier_phase,
                     &D,
                     &cn0,
                     &lock_time,
                     &dst->flags,
                     &dst->sid);

  dst->doppler = (float)D;
  dst->cn0 = (float)cn0;
  dst->lock_time = (float)lock_time;
}

void unpack_osr_content(const packed_osr_content_t *msg, starling_obs_t *dst) {
  dst->pseudorange = ((double)msg->P) / MSG_OBS_P_MULTIPLIER;
  dst->carrier_phase =
      (((double)msg->L.i) + (((double)msg->L.f) / MSG_OBS_LF_MULTIPLIER));
  dst->doppler = 0.0;
  dst->cn0 = 50;
  dst->lock_time = (float)decode_lock_time(msg->lock);
  dst->osr_flags = msg->flags;
  dst->sid = sid_from_sbp(msg->sid);
  dst->iono_std = (float)((double)msg->iono_std / MSG_OSR_IONO_STD_MULTIPLIER);
  dst->tropo_std =
      (float)((double)msg->tropo_std / MSG_OSR_TROPO_STD_MULTIPLIER);
  dst->range_std =
      (float)((double)msg->range_std / MSG_OSR_RANGE_STD_MULTIPLIER);

  dst->flags = 0;
  if ((msg->flags & INVALID_CODE_CORRECTIONS) != INVALID_CODE_CORRECTIONS) {
    dst->flags |= NAV_MEAS_FLAG_CODE_VALID | NAV_MEAS_FLAG_CN0_VALID;
  }
  if ((msg->flags & INVALID_PHASE_CORRECTIONS) != INVALID_PHASE_CORRECTIONS) {
    dst->flags |= NAV_MEAS_FLAG_PHASE_VALID | NAV_MEAS_FLAG_HALF_CYCLE_KNOWN |
                  NAV_MEAS_FLAG_CN0_VALID;
  }
}

static void unpack_ephemeris_common(const ephemeris_common_content_t *common,
                                    ephemeris_t *e) {
  e->toe.tow = common->toe.tow;
  e->toe.wn = (s16)common->toe.wn;
  e->valid = common->valid;
  e->health_bits = common->health_bits;
  e->sid = sid_from_sbp(common->sid);
  e->fit_interval = common->fit_interval;
  e->ura = common->ura;
}

static void unpack_ephemeris_gps(const msg_ephemeris_t *m, ephemeris_t *e) {
  const msg_ephemeris_gps_t *msg = &m->gps;
  unpack_ephemeris_common(&msg->common, e);
  e->source = EPH_SOURCE_GPS_LNAV;
  ephemeris_kepler_t *k = &e->data.kepler;
  k->tgd.gps_s[0] = msg->tgd;
  k->tgd.gps_s[1] = 0.0;
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

static void unpack_ephemeris_qzss(const msg_ephemeris_t *m, ephemeris_t *e) {
  const msg_ephemeris_qzss_t *msg = &m->qzss;
  unpack_ephemeris_common(&msg->common, e);
  e->source = EPH_SOURCE_QZS_LNAV;
  ephemeris_kepler_t *k = &e->data.kepler;
  k->tgd.qzss_s[0] = msg->tgd;
  k->tgd.qzss_s[1] = 0.0f;
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

static void unpack_ephemeris_bds(const msg_ephemeris_t *m, ephemeris_t *e) {
  const msg_ephemeris_bds_t *msg = &m->bds;
  unpack_ephemeris_common(&msg->common, e);
  e->source = EPH_SOURCE_BDS_D1_D2_NAV;
  ephemeris_kepler_t *k = &e->data.kepler;
  k->tgd.bds_s[0] = msg->tgd1;
  k->tgd.bds_s[1] = msg->tgd2;
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

static void unpack_ephemeris_gal(const msg_ephemeris_t *m, ephemeris_t *e) {
  const msg_ephemeris_gal_t *msg = &m->gal;
  unpack_ephemeris_common(&msg->common, e);
  e->source = msg->source;
  ephemeris_kepler_t *k = &e->data.kepler;
  k->tgd.gal_s[0] = msg->bgd_e1e5a;
  k->tgd.gal_s[1] = msg->bgd_e1e5b;
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

static void unpack_ephemeris_sbas(const msg_ephemeris_t *m, ephemeris_t *e) {
  const msg_ephemeris_sbas_t *msg = &m->sbas;
  unpack_ephemeris_common(&msg->common, e);
  e->source = 0;  // No sources currently defined for SBAS
  ephemeris_xyz_t *k = &e->data.xyz;
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

static void unpack_ephemeris_glo(const msg_ephemeris_t *m, ephemeris_t *e) {
  const msg_ephemeris_glo_t *msg = &m->glo;
  unpack_ephemeris_common(&msg->common, e);
  e->source = EPH_SOURCE_GLO_FDMA;
  ephemeris_glo_t *k = &e->data.glo;
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
  k->fcn = msg->fcn;

  if (!glo_slot_id_is_valid(msg->common.sid.sat)) {
    log_warn("Received GLO ephemeris from peer with invalid slot id %u",
             msg->common.sid.sat);
  } else if (!glo_fcn_is_valid(msg->fcn)) {
    log_warn("Received GLO ephemeris from peer with invalid FCN %u", msg->fcn);
  } else {
    glo_map_set_slot_id(msg->fcn, msg->common.sid.sat);
  }
}

#define TYPE_TABLE_INVALID_MSG_ID 0

typedef void (*unpack_ephe_func)(const msg_ephemeris_t *, ephemeris_t *);

static unpack_ephe_func ephe_unpacker_table[CONSTELLATION_COUNT] = {

    /* GPS */
    [CONSTELLATION_GPS] = unpack_ephemeris_gps,

    /* SBAS */
    [CONSTELLATION_SBAS] = unpack_ephemeris_sbas,

    /* GLO */
    [CONSTELLATION_GLO] = unpack_ephemeris_glo,

    /* BDS */
    [CONSTELLATION_BDS] = unpack_ephemeris_bds,

    /* GAL */
    [CONSTELLATION_GAL] = unpack_ephemeris_gal,

    /* QZSS */
    [CONSTELLATION_QZS] = unpack_ephemeris_qzss,
};

void unpack_ephemeris(const msg_ephemeris_t *msg, ephemeris_t *e) {
  /* NOTE: here we use common part of GPS message to take sid.code info.
   *       this also should work for other GNSS because common part is located
   *       at the same place in memory for all structures.*/
  constellation_t c =
      code_to_constellation(((msg_ephemeris_gps_t *)msg)->common.sid.code);

  assert(NULL != ephe_unpacker_table[c]);

  ephe_unpacker_table[c](msg, e);
}

void unpack_sbas_raw_data(const msg_sbas_raw_t *m, sbas_raw_data_t *d) {
  d->sid = sid_from_sbp(m->sid);
  d->message_type = m->message_type;
  d->time_of_transmission.tow = ((double)m->tow) / SECS_MS;
  MEMCPY_S(d->data, SBAS_RAW_PAYLOAD_LENGTH, m->data, SBAS_RAW_PAYLOAD_LENGTH);
}

void sbp_unpack_glonass_biases_content(const msg_glo_biases_t msg,
                                       glo_biases_t *glonass_biases) {
  glonass_biases->mask = (msg.mask);
  glonass_biases->l1of_bias_m =
      ((double)msg.l1ca_bias / MSG_GLO_BIASES_MULTIPLIER);
  glonass_biases->l1p_bias_m =
      ((double)msg.l1p_bias / MSG_GLO_BIASES_MULTIPLIER);
  glonass_biases->l2of_bias_m =
      ((double)msg.l2ca_bias / MSG_GLO_BIASES_MULTIPLIER);
  glonass_biases->l2p_bias_m =
      ((double)msg.l2p_bias / MSG_GLO_BIASES_MULTIPLIER);
}

void sbp_unpack_imu_raw(const u8 *msg,
                        double accl_sf,
                        double gyro_sf,
                        imu_data_t *starling_imu_data,
                        imu_time_quality_t *time_quality) {
  /* Note, this function really decodes the message in a way that works for any
   * system This functionality should be a part of libsbp c library and
   * generated, but isn't yet. Casting to a struct is bad and we need to get
   * libsbp and starling off of that pattern ASAP. Granted, it is kind of
   * hackey, but a useful pattern to show how to make a better sbp decoder */
  u32 tow_ms = ((u32)msg[0] << 0) | ((u32)msg[1] << 8) | ((u32)msg[2] << 16) |
               ((u32)msg[3] << 24);
  // Timestamp information encoded into the highest two bits.
  const u32 time_alignment_mode = (tow_ms & 0xC0000000) >> 30;

  // Mask off the highest two bits.
  tow_ms &= 0x3FFFFFFF;

  double tow_sec = (tow_ms + msg[4] / 256.0) / 1000.0;
  starling_imu_data->t.wn = WN_UNKNOWN;
  starling_imu_data->t.tow = tow_sec;
  /* it is okay to compare with zero, as this is the initialized value for
   * struct indicating that the scale factors are currently unknown */
  for (int i = 0; i < 3; i++) {
    s16 tics = (s16)(((u16)msg[5 + i * 2]) | ((u16)msg[6 + i * 2] << 8));
    starling_imu_data->acc_xyz[i] = (double)tics * accl_sf;
  }
  for (int i = 0; i < 3; i++) {
    s16 tics = (s16)(((u16)msg[11 + i * 2]) | ((u16)msg[12 + i * 2] << 8));
    starling_imu_data->gyr_xyz[i] = (double)tics * gyro_sf;
  }
  // Time quality is decoded according to document:
  // https://docs.google.com/document/d/1MIWJPxcE066henG6x00ugKtJJVIhKjRa6uuxDqJj7Y8
  *time_quality = time_alignment_mode;
}

int sbp_unpack_imu_aux_accl_sf(const u8 *msg, double *accl_sf) {
  u8 imu_type = msg[0];
  const double *lut = NULL;
  switch (imu_type) {
    case IMU_TYPE_ASM330: {
      lut = ASM330_ACCL_SF_MPSPS_LUT;
      // TODO(who?): Possible handle IMU types inside sensor fusion
      break;
    }
    case IMU_TYPE_BMI160: {
      lut = BMI160_ACCL_SF_MPSPS_LUT;
      break;
    }
    case IMU_TYPE_ANDROID: {
      lut = ANDROID_ACCL_SF_MPSPS_LUT;
      break;
    }
    default: { return IMU_AUX_ERROR_UNKNOWN_IMU; }
  }
  u8 accl_range = msg[3] & 0x0f;
  if (accl_range < 4) {
    *accl_sf = lut[accl_range];
    return IMU_AUX_EXIT_SUCCESS;
  }
  return IMU_AUX_ERROR_INVALID_RANGE;
}

int sbp_unpack_imu_aux_gyro_sf(const u8 *msg, double *gyro_sf) {
  u8 imu_type = msg[0];
  const double *lut = NULL;
  switch (imu_type) {
    case IMU_TYPE_ASM330: {
      lut = ASM330_GYRO_SF_DEGPS_LUT;
      // deliberately fall through. Should we warn?
      // TODO(who?): Possible handle IMU types inside sensor fusion
      break;
    }
    case IMU_TYPE_BMI160: {
      lut = BMI160_GYRO_SF_DEGPS_LUT;
      break;
    }
    case IMU_TYPE_ANDROID: {
      lut = ANDROID_GYRO_SF_DEGPS_LUT;
      break;
    }
    default: { return IMU_AUX_ERROR_UNKNOWN_IMU; }
  }
  u8 gyro_range = (msg[3] & 0xf0) >> 4;
  if (gyro_range < 5) {
    *gyro_sf = lut[gyro_range];
    return IMU_AUX_EXIT_SUCCESS;
  }
  return IMU_AUX_ERROR_INVALID_RANGE;
}

int sbp_unpack_imu_aux_temp(const u8 *msg, double *imu_temp) {
  assert(imu_temp != NULL);
  u8 imu_type = msg[0];

  u16 raw_imu_temp_u = msg[2] << 8 | msg[1];
  s16 raw_imu_temp = *(s16 *)&raw_imu_temp_u;

  switch (imu_type) {
    case IMU_TYPE_BMI160: {
      const int16_t kTemperatureInvalid = 0x8000;
      if (raw_imu_temp == kTemperatureInvalid) {
        return IMU_AUX_ERROR_INVALID_TEMPERATURE;
      }
      *imu_temp = (double)raw_imu_temp / 512.0 + 23.0;
      return IMU_AUX_EXIT_SUCCESS;
    } break;
    case IMU_TYPE_ASM330: {
      *imu_temp = (double)raw_imu_temp / 256.0 + 25.0;
      return IMU_AUX_EXIT_SUCCESS;
    } break;
    default:
      return IMU_AUX_ERROR_UNKNOWN_IMU;
  }
}

void unpack_ephemeris_gal_dep_a(const msg_ephemeris_gal_dep_a_t *msg,
                                ephemeris_t *e) {
  unpack_ephemeris_common(&msg->common, e);
  e->source = EPH_SOURCE_GAL_INAV;
  e->data.kepler.tgd.gps_s[0] = msg->bgd_e1e5a;
  e->data.kepler.tgd.gps_s[1] = msg->bgd_e1e5b;
  e->data.kepler.crs = msg->c_rs;
  e->data.kepler.crc = msg->c_rc;
  e->data.kepler.cuc = msg->c_uc;
  e->data.kepler.cus = msg->c_us;
  e->data.kepler.cic = msg->c_ic;
  e->data.kepler.cis = msg->c_is;
  e->data.kepler.dn = msg->dn;
  e->data.kepler.m0 = msg->m0;
  e->data.kepler.ecc = msg->ecc;
  e->data.kepler.sqrta = msg->sqrta;
  e->data.kepler.omega0 = msg->omega0;
  e->data.kepler.omegadot = msg->omegadot;
  e->data.kepler.w = msg->w;
  e->data.kepler.inc = msg->inc;
  e->data.kepler.inc_dot = msg->inc_dot;
  e->data.kepler.af0 = msg->af0;
  e->data.kepler.af1 = msg->af1;
  e->data.kepler.af2 = msg->af2;
  e->data.kepler.toc.tow = msg->toc.tow;
  e->data.kepler.toc.wn = (s16)msg->toc.wn;
  e->data.kepler.iode = msg->iode;
  e->data.kepler.iodc = msg->iodc;
}

void sbp_unpack_sbp_gps_time(const sbp_gps_time_t *msg, gps_time_t *output) {
  output->wn = msg->wn;
  output->tow = (msg->tow / 1000.0) + (msg->ns_residual / 1000000000.0);
}

void sbp_unpack_gps_time(const msg_gps_time_t *msg, gps_time_t *output) {
  output->wn = msg->wn;
  output->tow = (msg->tow / 1000.0) + (msg->ns_residual / 1000000000.0);
}

void sbp_unpack_gps_time_gnss(const msg_gps_time_gnss_t *msg,
                              gps_time_t *output) {
  output->wn = msg->wn;
  output->tow = (msg->tow / 1000.0) + (msg->ns_residual / 1000000000.0);
}

void sbp_unpack_tow_from_pos_ecef(const msg_pos_ecef_t *msg,
                                  gps_time_t *output) {
  output->tow = (msg->tow / 1000.0);
}

void sbp_unpack_pos_ecef_gnss(const msg_pos_ecef_gnss_t *msg,
                              pos_ecef_data_t *output) {
  output->tow_secs = msg->tow / SECS_MS;
  output->pos_xyz[0] = msg->x;
  output->pos_xyz[1] = msg->y;
  output->pos_xyz[2] = msg->z;
  output->flags = msg->flags;
  output->n_sats = msg->n_sats;
  output->accuracy_in_meters = msg->accuracy / 1000.0;
}

void sbp_unpack_tow_from_pos_ecef_cov(const msg_pos_ecef_cov_t *msg,
                                      gps_time_t *output) {
  output->tow = (msg->tow / 1000.0);
}

void sbp_unpack_tow_from_pos_llh(const msg_pos_llh_t *msg, gps_time_t *output) {
  output->tow = (msg->tow / 1000.0);
}

void sbp_unpack_tow_from_vel_ned(const msg_vel_ned_t *msg, gps_time_t *output) {
  output->tow = (msg->tow / 1000.0);
}

void sbp_unpack_tow_from_vel_ned_cov(const msg_vel_ned_cov_t *msg,
                                     gps_time_t *output) {
  output->tow = (msg->tow / 1000.0);
}

void sbp_unpack_tow_from_orient_euler(const msg_orient_euler_t *msg,
                                      gps_time_t *output) {
  output->tow = (msg->tow / 1000.0);
}

void sbp_unpack_iono(const msg_iono_t *msg, ionosphere_t *iono) {
  iono->toa.wn = msg->t_nmct.wn;
  iono->toa.tow = msg->t_nmct.tow;
  iono->a0 = msg->a0;
  iono->a1 = msg->a1;
  iono->a2 = msg->a2;
  iono->a3 = msg->a3;
  iono->b0 = msg->b0;
  iono->b1 = msg->b1;
  iono->b2 = msg->b2;
  iono->b3 = msg->b3;
}
