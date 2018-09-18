/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <libswiftnav/bits.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/gnss_time.h>
#include <libswiftnav/logging.h>

#include "timing/timing.h"

#include "nav_msg/nav_msg_glo.h"

#define BIT_POLARITY_NORMAL 0
#define BIT_POLARITY_INVERTED 1
#define BIT_POLARITY_UNKNOWN -1

/* GLO fit intervals do not seem to overlap.
   P1 field from string 1 defines a fit interval.
   tb field from string 2 changes at the beginning of the fit interval.
   tb field is TOE.
   The previous fit interval ends at the start of the frame, which delivers
   the new tb value. So there is a small time window, after the old ephemeris
   gets stale and the new ephemeris is not decoded yet.
   This define sets an overlap margin of #FIT_INTERVAL_MARGIN_S / 2 seconds
   to allow for the new ephemeris decoding. */
#define FIT_INTERVAL_MARGIN_S (10 * MINUTE_SECS)

/* Minimum one-sided data validity time around toe value in seconds. */
#define MIN_VALIDITY_WINDOW_S (15 * MINUTE_SECS)
/* Maximum one-sided data validity time around toe value in seconds. */
#define MAX_VALIDITY_WINDOW_S (30 * MINUTE_SECS)
/* The time difference of one-sided data validity windows in seconds.
 * Difference between 15 min vs. 22.5 min vs. 30 min is 7.5 min. */
#define DIFF_VALIDITY_WINDOW_S ((u32)(7.5f * MINUTE_SECS))

enum glo_sv_model { SV_GLONASS, SV_GLONASS_M };

/* GLO parameter limits (ICD L1,L2 GLONASS edition 5.1 2008 Table 4.5) */
#define GLO_POS_MAX_M (2.7e4 * 1e3)         /* [m] */
#define GLO_VEL_MAX_M_S (4.3 * 1e3)         /* [m/s] */
#define GLO_ACC_MAX_M_S2 (6.2e-9 * 1e3)     /* [m/s^2] */
#define GLO_TK_HOURS_MAX 23                 /* [hours] */
#define GLO_TK_MINS_MAX 59                  /* [mins] */
#define GLO_TB_MIN_S (15 * MINUTE_SECS)     /* [s] */
#define GLO_TB_MAX_S (1425 * MINUTE_SECS)   /* [s] */
#define GLO_GAMMA_MAX 9.313225746154785e-10 /* 2^(-30)[unitless] */
#define GLO_TAU_MAX_S 0.001953125           /* 2^(-9)[s] */
#define GLO_D_TAU_MAX_S 13.97e-9            /* [s] */
#define GLO_NT_MAX_DAYS 1461                /* [days] */

/* ICD L1,L2 GLONASS edition 5.1 2008 Table 4.5 Table 4.9 */
#define GLO_TAU_GPS_MAX_S 1.9e-3 /* [s] */

/* The figures below are from the analysis of GLO ephemeris data
   across 2013-2017 at ftp://cddis.gsfc.nasa.gov/gnss/data/daily/

   The GLO orbit radius max sigma (STD) was 48 km (2014)
   The GLO velocity max sigma was 0.150505 km/s^2 (2014)
   A 2% margin results in ~10 sigma for position.
   A 22% margin results in ~5 sigma for velocity,
   which seems to be a safe assumption.

   The sigmas were computed without outliers which could be caused by
   unhealthy SV (the analysis did not check the SVs health status):
   for position the assumeed valid range was [ >0 km .. 30000 km]
   for velocity the range was >0 m/s
*/

#define GLO_ORBIT_RADIUS_M (25508.2207198 * 1e3)              /* [m] */
#define GLO_ORBIT_RADIUS_MARGIN_M (0.02 * GLO_ORBIT_RADIUS_M) /* [m] */

#define GLO_VEL_MAG_M_S (3.37874 * 1e3)                 /* [m/s] */
#define GLO_VEL_MAG_MARGIN_M_S (0.22 * GLO_VEL_MAG_M_S) /* [m/s] */

/* Word Ft (accuracy of measurements), refer to GLO ICD, Table 4.4 */
static const float f_t[] = {1.0f,
                            2.0f,
                            2.5f,
                            4.0f,
                            5.0f,
                            7.0f,
                            10.0f,
                            12.0f,
                            14.0f,
                            16.0f,
                            32.0f,
                            64.0f,
                            128.0f,
                            256.0f,
                            512.0f,
                            INVALID_URA_VALUE};

/* Word P1 (Time interval between adjacent values of tb).
   Refer to table 4.3 of GLO ICD */
static const u8 p1_lookup_min[] = {0, 30, 45, 60}; /* [min] */

/* These bit masks (for data bits 9..85) correspond to table 4.13 of GLO ICD
 * used in error correction algorithm */
static const u32 e_masks[7][3] = {
    {0xaaad5b00, 0x55555556, 0xaaaab},
    {0x33366d00, 0x9999999b, 0xccccd},
    {0xc3c78e00, 0xe1e1e1e3, 0x10f0f1},
    {0xfc07f000, 0xfe01fe03, 0xff01},
    {0xfff80000, 0xfffe0003, 0x1f0001},
    {0, 0xfffffffc, 1},
    {0, 0, 0x1ffffe},
};

/** Initialize the structure for removal of relative code transformation.
 *  Initialization is called after finding GLO time mark.
 * \param relcode Pointer to relcode removal structure.
 */
static void relcode_init(relcode_t *relcode) { relcode->state = 0; }

/** Update the structure for removal of relative code transformation.
 * \param relcode Pointer to relcode removal structure.
 * \param bit     Data bit after Meander removal.
 * \return        Decoded data bit.
 */
static u8 relcode_decode(relcode_t *relcode, u8 bit) {
  u8 decoded = relcode->state ^ bit;
  relcode->state = bit;
  return decoded;
}

/** Initialize the necessary parts of the nav message state structure.
 * \param n Pointer to GLO nav message structure to be initialized
 * \param mesid Decoding channel ME sid
 */
void nav_msg_init_glo(nav_msg_glo_t *n, me_gnss_signal_t mesid) {
  memset(n, 0, sizeof(nav_msg_glo_t));
  n->state = SYNC_TM;
  n->bit_polarity = BIT_POLARITY_UNKNOWN;
  n->eph.toe = GPS_TIME_UNKNOWN;
  n->eph.glo.fcn = mesid.sat;
  n->gps_time = GPS_TIME_UNKNOWN;
  n->mesid = mesid;
}

/** Extract a word of n_bits length (n_bits <= 32) at position bit_index into
 * the subframe. Refer to bit index to Table 4.6 and 4.11 in GLO ICD 5.1 (pg.
 * 34)
 * \param n pointer to GLO nav message structure to be parsed
 * \param bit_index number of bit the extract process start with. Range [1..85]
 * \param n_bits how many bits should be extracted [1..32]
 * \return word extracted from navigation string
 */
u32 extract_word_glo(const nav_msg_glo_t *n, u16 bit_index, u8 n_bits) {
  assert(bit_index);
  assert(bit_index <= GLO_STR_LEN);

  assert(n_bits);
  assert(n_bits <= 32);

  /* Extract a word of n_bits length (n_bits <= 32) at position bit_index into
   * the GLO string.*/
  bit_index--;
  u32 word = 0;
  u8 bix_hi = bit_index >> 5;
  u8 bix_lo = bit_index & 0x1F;
  if (bix_lo + n_bits <= 32) {
    word = n->string_bits[bix_hi] >> bix_lo;
    word &= (0xffffffff << (32 - n_bits)) >> (32 - n_bits);
  } else {
    u8 s = 32 - bix_lo;
    word = extract_word_glo(n, bit_index + 1, s) |
           extract_word_glo(n, bit_index + 1 + s, n_bits - s) << s;
  }

  return word;
}

/** The function performs data verification and error detection
 * in received GLO navigation string. Refer to GLO ICD, section 4.7
 * \param n pointer to GLO nav message structure
 * \return -1 -- received string is bad and should be dropped out,
 *          0 -- received string is good
 *          >0 -- number of bit in n->string_bits to be corrected (inverted)
 *                range[9..85]*/
s8 error_detection_glo(const nav_msg_glo_t *n) {
  u8 c = 0;
  u32 data1, data2, data3;
  bool p0, p1, p2, p3, beta, c_sum;
  u8 bit_set = 0;
  u8 k = 0;

  /* calculate C1..7 */
  for (u8 i = 0; i < 7; i++) {
    /* extract corresponding check bit of Hamming code */
    beta = extract_word_glo(n, i + 1, 1);
    /* extract data bits and apply mask */
    data1 = extract_word_glo(n, 1, 32) & e_masks[i][0];
    data2 = extract_word_glo(n, 33, 32) & e_masks[i][1];
    data3 = extract_word_glo(n, 65, 32) & e_masks[i][2];
    /* calculate parity for data[1..3] */
    p1 = parity(data1);
    p2 = parity(data2);
    p3 = parity(data3);
    bool p = beta ^ p1 ^ p2 ^ p3;
    /* calculate common parity and set according C bit */
    c |= p << i;
    if (p) {
      bit_set++; /* how many bits are set, used in error criteria */
      k = i + 1; /* store number of most significant checksum not equal to 0,
                    used in error criteria */
    }
  }

  /* calculate C sum */
  data1 = extract_word_glo(n, 1, 32) & 0xffffff00;
  data2 = extract_word_glo(n, 33, 32);
  data3 = extract_word_glo(n, 65, 32);
  p1 = parity(data1);
  p2 = parity(data2);
  p3 = parity(data3);
  p0 = parity(extract_word_glo(n, 1, 8));
  c_sum = p0 ^ p1 ^ p2 ^ p3;

  /* Now check C word to figure out is the string good, bad or
   * correction is needed */

  /* case a) from ICD */
  if ((!c_sum && !bit_set) || (1 == bit_set && c_sum)) {
    return 0; /* The string is good */
  }

  /* case b) from ICD */
  if (bit_set > 1 && c_sum) {
    u8 i_corr = (c & 0x7f) + 8 - k; /* define number of bit to be corrected */

    if (i_corr > GLO_STR_LEN) {
      return -1; /* odd number of multiple errors, bad string */
    }

    return i_corr; /* return the bit to be corrected */
  }

  /* case c) from ICD */
  if ((bit_set > 0 && !c_sum) || (0 == bit_set && c_sum)) {
    return -1; /* multiple errors, bad string */
  }

  /* should not be here */
  log_error("GLO error correction: unexpected case");
  return -1;
}

/** Seek GLO timemark.
 * Tries to find either normal or inverted timemark sequence.
 * Switch to data decoding state once timemark is found.
 *
 * \param n      GLO Nav message decode state struct
 * \param symbol State of the nav symbol to process, 0 or 1
 * \return GLO timemark was decoded (true) or not (false)
 */
bool timemark_glo_decoded(nav_msg_glo_t *n, bool symbol) {
  /* put incoming symbol at the tail of the buffer */
  n->string_bits[0] <<= 1; /* use one word of buffer for that purpose */
  n->string_bits[0] |= symbol;
  /* collected symbols match time mark? if not stay at this state */
  u32 tm = extract_word_glo(n, 1, GLO_TM_LEN_SYMBOLS);
  if ((GLO_TM != tm) && (GLO_TM_INV != tm)) {
    return false;
  }

  /* time mark found, next time start collecting data symbols */
  n->meander_bits_cnt = 0;
  n->manchester = 0;
  n->state = GET_DATA_BIT;
  n->string_bits[0] = 0;
  relcode_init(&n->relcode);
  s8 prev_bit_polarity = n->bit_polarity;
  n->bit_polarity =
      (GLO_TM == tm) ? BIT_POLARITY_NORMAL : BIT_POLARITY_INVERTED;
  if (BIT_POLARITY_UNKNOWN == prev_bit_polarity) {
    return true;
  }
  if (prev_bit_polarity != n->bit_polarity) {
    log_warn_mesid(n->mesid, "Bit polarity mismatch");
  }
  return true;
}

/** Collect data bits of GLO string.
 * Applies meander removal for 2 symbols (10 ms each).
 * Also applies relative code removal.
 *
 * \param n      GLO Nav message decode state struct
 * \param symbol State of the nav symbol to process, 0 or 1
 * \return GLO_STRING_READY if GLO nav string ready for decoding,
 *         GLO_STRING_NOT_READY otherwise.
 */
nav_msg_status_t get_data_bits_glo(nav_msg_glo_t *n, bool symbol) {
  nav_msg_status_t ret = GLO_STRING_NOT_READY;
  n->meander_bits_cnt++;
  n->manchester <<= 1;
  n->manchester |= symbol; /* store incoming symbol */
                           /* did we take 2 symbols of line code?
                            * if no, stay at the state */
  if (2 != n->meander_bits_cnt) {
    return ret;
  }

  /* Check if there was a symbol error.
   * Two consecutive symbols should not be equal. */
  if (3 == n->manchester || 0 == n->manchester) {
    log_debug_mesid(n->mesid, "GLO-NAV-ERR: symbol error");
    nav_msg_init_glo(n, n->mesid);
    return ret;
  }

  /* remove meander and store bit in buffer */
  /* shift whole buffer by 1 bit left */
  for (u8 i = NAV_MSG_GLO_STRING_BITS_LEN - 1; i > 0; i--) {
    u32 tmp =
        (n->string_bits[i] << 1) | ((n->string_bits[i - 1] & (1u << 31)) >> 31);
    n->string_bits[i] = tmp;
  }
  n->string_bits[0] <<= 1;

  /* set type of meander depending on inversion */
  u8 meander = (BIT_POLARITY_NORMAL == n->bit_polarity) ? 1 : 2;
  /* meander removal */
  u8 bit = (n->manchester ^ meander) & 1;
  /* relative code removal */
  u8 relcode = relcode_decode(&n->relcode, bit);
  /* store bit to buffer */
  n->string_bits[0] |= relcode;
  n->current_head_bit_index++;
  n->meander_bits_cnt = 0;
  n->manchester = 0;
  /* did we received all bits of a string?
   * if yes, notify user and start searching time mark again*/
  if (GLO_STR_LEN == n->current_head_bit_index) {
    n->current_head_bit_index = 0;
    n->state = SYNC_TM;
    ret = GLO_STRING_READY;
  }
  return ret;
}

/** GLO navigation message decoding update.
 * Called once per nav symbol interval (10 ms).
 * Performs the necessary steps to store the nav bits in buffer.
 *
 * \param n GLO Nav message decode state struct
 * \param symbol State of the nav symbol to process, 0 or 1
 *
 * \return GLO_STRING_READY if Glo nav string ready for decoding,
 *         GLO_TIME_MARK_DECODED if GLO time mark was just decoded,
 *         GLO_STRING_NOT_READY otherwise.
 */
nav_msg_status_t nav_msg_update_glo(nav_msg_glo_t *n, bool symbol) {
  s8 ret = GLO_STRING_NOT_READY;

  switch (n->state) {
    case SYNC_TM: /* try to find time mark */
      if (timemark_glo_decoded(n, symbol)) {
        ret = GLO_TIME_MARK_DECODED;
      }
      break;
    case GET_DATA_BIT: /* collect data bits of string */
      ret = get_data_bits_glo(n, symbol);
      break;
    default:
      assert(0 && "Unexpected GLO nav msg update state");
      break;
  }
  return ret;
}

/** Decode position component of the ephemeris data (X/Y/Z)
 * \param n GLO nav message decode state struct
 * \return The decoded position component [m]
 */
static double decode_position_component(const nav_msg_glo_t *n) {
  double pos_m = extract_word_glo(n, 9, 26) * C_1_2P11 * 1000.0;
  u8 sign = extract_word_glo(n, 9 + 26, 1);
  if (sign) {
    pos_m *= -1;
  }
  return pos_m;
}

/** Decode velocity component of the ephemeris data (Vx/Vy/Vz)
 * \param n GLO nav message decode state struct
 * \return The decoded velocity component [m/s]
 */
static double decode_velocity_component(const nav_msg_glo_t *n) {
  /* extract velocity (Vx or Vy or Vz) */
  double vel_mps = extract_word_glo(n, 41, 23) * C_1_2P20 * 1000.0;
  u8 sign = extract_word_glo(n, 41 + 23, 1);
  if (sign) {
    vel_mps *= -1;
  }
  return vel_mps;
}

/** Decode acceleration component of the ephemeris data (Ax/Ay/Az)
 * \param n GLO nav message decode state struct
 * \return The decoded acceleration component [m/s^2]
 */
static double decode_acceleration_component(const nav_msg_glo_t *n) {
  /* extract acceleration (Ax or Ay or Az) */
  double acc_mps2 = extract_word_glo(n, 36, 4) * C_1_2P30 * 1000.0;
  u8 sign = extract_word_glo(n, 36 + 4, 1);
  if (sign) {
    acc_mps2 *= -1;
  }
  return acc_mps2;
}

static u32 compute_ephe_fit_interval(const nav_msg_glo_t *n, u32 p1) {
  assert(n);
  assert(p1 < ARRAY_SIZE(p1_lookup_min));

  u32 fit_interval_s = MINUTE_SECS * p1_lookup_min[p1];
  if (fit_interval_s != 0) {
    return fit_interval_s + FIT_INTERVAL_MARGIN_S;
  }

  if (0 == n->eph.fit_interval) {
    /* We have not decoded any fit interval yet,
       So let's default to the maximum fit interval possible + a margin.
       The maximum fit interval is defined by the maximum value of P1,
       which is 60 minutes. Once we have a real value from P1 we will
       start using the real value. */
    fit_interval_s = MINUTE_SECS * 60 + FIT_INTERVAL_MARGIN_S;
  } else {
    fit_interval_s = n->eph.fit_interval;
  }

  return fit_interval_s;
}

static bool extract_string_1_components(nav_msg_glo_t *n) {
  double pos_m = decode_position_component(n); /* extract x */
  if ((pos_m < -GLO_POS_MAX_M) || (GLO_POS_MAX_M < pos_m)) {
    log_debug_mesid(n->mesid, "GLO-NAV-ERR: pos_x =%lf m", pos_m);
    return false;
  }
  n->eph.glo.pos[0] = pos_m;

  double vel_m_s = decode_velocity_component(n); /* extract Vx */
  if ((vel_m_s < -GLO_VEL_MAX_M_S) || (GLO_VEL_MAX_M_S < vel_m_s)) {
    log_debug_mesid(n->mesid, "GLO-NAV-ERR: vel_x=%lf m/s", vel_m_s);
    return false;
  }
  n->eph.glo.vel[0] = vel_m_s;

  double acc_m_s2 = decode_acceleration_component(n); /* extract Ax */
  if ((acc_m_s2 < -GLO_ACC_MAX_M_S2) || (GLO_ACC_MAX_M_S2 < acc_m_s2)) {
    log_debug_mesid(n->mesid, "GLO-NAV-ERR: acc_x=%lf m/s^2", acc_m_s2);
    return false;
  }
  n->eph.glo.acc[0] = acc_m_s2;

  /* extract tk */
  n->tk.h = (u8)extract_word_glo(n, 72, 5);
  if (n->tk.h > GLO_TK_HOURS_MAX) {
    log_debug_mesid(n->mesid, "GLO-NAV-ERR: tk_h=%" PRIu8 " h", n->tk.h);
    return false;
  }
  n->tk.m = (u8)extract_word_glo(n, 66, 6);
  if (n->tk.m > GLO_TK_MINS_MAX) {
    log_debug_mesid(n->mesid, "GLO-NAV-ERR: tk_m=%" PRIu8 " min", n->tk.m);
    return false;
  }
  n->tk.s = extract_word_glo(n, 65, 1) ? MINUTE_SECS / 2 : 0.0;

  /* extract P1 */
  u32 p1 = extract_word_glo(n, 77, 2);
  n->eph.fit_interval = compute_ephe_fit_interval(n, p1);

  return true;
}

static bool extract_string_2_components(nav_msg_glo_t *n) {
  double pos_m = decode_position_component(n); /* extract y */
  if ((pos_m < -GLO_POS_MAX_M) || (GLO_POS_MAX_M < pos_m)) {
    log_debug_mesid(n->mesid, "GLO-NAV-ERR: pos_y =%lf m", pos_m);
    return false;
  }
  n->eph.glo.pos[1] = pos_m;

  double vel_m_s = decode_velocity_component(n); /* extract Vy */
  if ((vel_m_s < -GLO_VEL_MAX_M_S) || (GLO_VEL_MAX_M_S < vel_m_s)) {
    log_debug_mesid(n->mesid, "GLO-NAV-ERR: vel_y=%lf m/s", vel_m_s);
    return false;
  }
  n->eph.glo.vel[1] = vel_m_s;

  double acc_m_s2 = decode_acceleration_component(n); /* extract Ay */
  if ((acc_m_s2 < -GLO_ACC_MAX_M_S2) || (GLO_ACC_MAX_M_S2 < acc_m_s2)) {
    log_debug_mesid(n->mesid, "GLO-NAV-ERR: acc_y=%lf m/s^2", acc_m_s2);
    return false;
  }
  n->eph.glo.acc[1] = acc_m_s2;

  /* extract MSB of B (if the bit is 0 the SV is OK ) */
  n->eph.health_bits |= extract_word_glo(n, 80, 1);

  u32 tb_s = extract_word_glo(n, 70, 7) * 15 * MINUTE_SECS;
  if ((tb_s < GLO_TB_MIN_S) || (GLO_TB_MAX_S < tb_s)) {
    log_debug_mesid(n->mesid, "GLO-NAV-ERR: tb_s=%" PRIu32 " s", tb_s);
    return false;
  }
  n->toe.h = tb_s / HOUR_SECS;
  n->toe.m = (tb_s - n->toe.h * HOUR_SECS) / MINUTE_SECS;
  n->toe.s = tb_s - (n->toe.h * HOUR_SECS) - (n->toe.m * MINUTE_SECS);
  n->eph.glo.iod = tb_s & 0x7f; /* 7 LSB of Tb as IOD */

  return true;
}

static bool extract_string_3_components(nav_msg_glo_t *n) {
  u32 ret;
  u8 sign;

  double pos_m = decode_position_component(n); /* extract z */
  if ((pos_m < -GLO_POS_MAX_M) || (GLO_POS_MAX_M < pos_m)) {
    log_debug_mesid(n->mesid, "GLO-NAV-ERR: pos_z =%lf m", pos_m);
    return false;
  }
  n->eph.glo.pos[2] = pos_m;

  double vel_m_s = decode_velocity_component(n); /* extract Vz */
  if ((vel_m_s < -GLO_VEL_MAX_M_S) || (GLO_VEL_MAX_M_S < vel_m_s)) {
    log_debug_mesid(n->mesid, "GLO-NAV-ERR: vel_z=%lf m/s", vel_m_s);
    return false;
  }
  n->eph.glo.vel[2] = vel_m_s;

  double acc_m_s2 = decode_acceleration_component(n); /* extract Az */
  if ((acc_m_s2 < -GLO_ACC_MAX_M_S2) || (GLO_ACC_MAX_M_S2 < acc_m_s2)) {
    log_debug_mesid(n->mesid, "GLO-NAV-ERR: acc_z=%lf m/s^2", acc_m_s2);
    return false;
  }
  n->eph.glo.acc[2] = acc_m_s2;

  /* extract gamma */
  ret = extract_word_glo(n, 69, 10);
  sign = extract_word_glo(n, 69 + 10, 1);
  if (sign) {
    ret *= -1;
  }
  double gamma = (s32)ret * C_1_2P40;
  if ((gamma < -GLO_GAMMA_MAX) || (GLO_GAMMA_MAX < gamma)) {
    log_debug_mesid(n->mesid, "GLO-NAV-ERR: gamma=%lf", gamma);
    return false;
  }
  n->eph.glo.gamma = gamma;
  /* extract l, if it is 0 the SV is OK, so OR it with B */
  n->eph.health_bits |= extract_word_glo(n, 65, 1);

  return true;
}

static bool extract_string_4_components(nav_msg_glo_t *n) {
  u32 ret;
  u8 sign;

  /* extract tau */
  ret = extract_word_glo(n, 59, 21);
  sign = extract_word_glo(n, 59 + 21, 1);
  if (sign) {
    ret *= -1;
  }
  double tau_s = (s32)ret * C_1_2P30;
  if ((tau_s < -GLO_TAU_MAX_S) || (GLO_TAU_MAX_S < tau_s)) {
    log_debug_mesid(n->mesid, "GLO-NAV-ERR: tau=%lf s", tau_s);
    return false;
  }
  n->eph.glo.tau = tau_s;

  /* extract d_tau */
  ret = extract_word_glo(n, 54, 4);
  sign = extract_word_glo(n, 54 + 4, 1);
  if (sign) {
    ret *= -1;
  }
  double d_tau_s = (s32)ret * C_1_2P30;
  if ((d_tau_s < -GLO_D_TAU_MAX_S) || (GLO_D_TAU_MAX_S < d_tau_s)) {
    log_debug_mesid(n->mesid, "GLO-NAV-ERR: d_tau=%lf s", d_tau_s);
    return false;
  }
  n->eph.glo.d_tau = d_tau_s;

  /* extract n */
  u16 glo_slot_id = extract_word_glo(n, 11, 5);
  if (!glo_slot_id_is_valid(glo_slot_id)) {
    log_debug_mesid(n->mesid, "GLO-NAV-ERR: glo_slot_id=%" PRIu16, glo_slot_id);
    return false;
  }
  n->eph.sid.sat = glo_slot_id;

  /* extract Ft (URA) */
  n->eph.ura = f_t[extract_word_glo(n, 30, 4)];

  /*extract Nt*/
  u16 nt_days = (u16)extract_word_glo(n, 16, 11);
  if (GLO_NT_MAX_DAYS < nt_days) {
    log_debug_mesid(n->mesid, "GLO-NAV-ERR: nt=%" PRIu16 " days", nt_days);
    return false;
  }
  n->tk.nt = n->toe.nt = nt_days;

  u32 M = extract_word_glo(n, 9, 2);
  if (SV_GLONASS == M) {
    /* this breaks the assumption that all visible GLO satellites should be
       at least of "Glonass M" model*/
    log_warn_mesid(n->mesid, "Non GLONASS M SV detected");
  }
  return true;
}

static bool extract_string_5_components(nav_msg_glo_t *n) {
  u8 sign;

  /* extract N4 */
  u8 n4 = (u8)extract_word_glo(n, 32, 5);
  /* ICD L1,L2 GLONASS edition 5.1 2008 Table 4.5 Table 4.9 */
  if (0 == n4) {
    log_debug_mesid(n->mesid, "GLO-NAV-ERR: n4=0");
    return false;
  }
  n->tk.n4 = n->toe.n4 = n4;

  /* extract tau GPS [s] */
  double tau_gps_s = extract_word_glo(n, 10, 21) * C_1_2P30;
  if (GLO_TAU_GPS_MAX_S < tau_gps_s) {
    log_debug_mesid(n->mesid, "GLO-NAV-ERR: tau_gps=%lf", tau_gps_s);
    return false;
  }
  /* convert to [ns] */
  n->tau_gps_ns = (s32)(tau_gps_s * 1e9 + .5); /* 0.5 is for rounding */
  sign = extract_word_glo(n, 31, 1);
  if (sign) {
    n->tau_gps_ns *= -1;
  }

  return true;
}

/** Find out how many strings have been decoded after last string 1.
 *  Each additional string adds 2 seconds to the decoded TOW.
 *
 * \param string_receive_time Array containing the latest time tag
 *                            for each string.
 * \param tow_age             TOW age in seconds.
 * \return TRUE, if time tags are ok, FALSE otherwise.
 */
static bool find_tow_age(u32 string_receive_time_ms[GLO_STRINGS_TO_COLLECT],
                         u32 *tow_age_s) {
  bool retval = false;
  u32 tow_tag_ms = string_receive_time_ms[0]; /* String 1 (TOW) time tag. */
  u32 max_tag_ms = 0; /* Max tag is for finding last decoded string time tag. */

  for (u8 i = 0; i < GLO_STRINGS_TO_COLLECT; i++) {
    if (string_receive_time_ms[i] > max_tag_ms) {
      max_tag_ms = string_receive_time_ms[i];
    }
  }
  /* If string 1 was the last decoded string, then max_tag == tow_tag. */

  /* Convert millisecond time difference into seconds. */
  u32 tag_diff_s = roundf((max_tag_ms - tow_tag_ms) / (float)SECS_MS);

  /* The time tag diff must be divisible by 2 seconds,
   * and it must be less than the validity window. */
  if ((tag_diff_s % GLO_STR_LEN_S) == 0 &&
      (tag_diff_s <= MAX_VALIDITY_WINDOW_S)) {
    *tow_age_s = tag_diff_s;
    retval = true;
  }

  return retval;
}

/** Restart GLO decoder after receiving all 5 strings,
 *  and fill remaining ephemeris parameters.
 *
 * \param n GLO nav message decode state struct
 *
 */
static void restart_decoding(nav_msg_glo_t *n) {
  n->decoded_strings = 0;
  /* We only support single ephemeris storage per GLO satellite.
     Ephemeris decoded from GLO L1CA is same as the one decoded
     from GLO L2CA */
  n->eph.sid.code = CODE_GLO_L1OF;

  /* convert GLO TOE to GPS TOE */
  n->eph.toe = glo2gps_with_utc_params(n->mesid, &n->toe);

  n->eph.valid = 1;
}

/** Checks if decoded strings are within same validity window.
 *
 * \param n    GLO nav message decode state struct
 * \param tk_s GLO TOW from string 1, in seconds
 * \param skip Allow single string to be skipped in the check.
 *             [Zero-based index] i.e. string 1 would be index 0.
 *             Use GLO_SKIP_STRING_NONE if no string is skipped.
 *
 * \return TRUE, if all strings from same validity window, FALSE otherwise.
 */
static bool check_validity_window(const nav_msg_glo_t *n, s32 tk_s, u8 skip) {
  u32 tow_tag_ms = n->string_receive_time_ms[0];
  u32 tk_validity_index = tk_s / DIFF_VALIDITY_WINDOW_S;

  for (u8 i = 1; i < GLO_STRINGS_TO_COLLECT; i++) {
    if (i == skip) {
      continue;
    }
    s32 tag_diff_ms = n->string_receive_time_ms[i] - tow_tag_ms;
    s32 tag_diff_s = roundf(tag_diff_ms / (float)SECS_MS);
    s32 receive_time_s = tk_s + tag_diff_s;
    u32 string_validity_index = receive_time_s / DIFF_VALIDITY_WINDOW_S;

    if (tk_validity_index != string_validity_index) {
      return false;
    }
  }
  return true;
}

/** Checks if decoded strings are from same ephemeris data set.
 *  Case: MIN_VALIDITY_WINDOW_S < ABS(tk-tb) < MAX_VALIDITY_WINDOW_S
 *  In this case a longer than nominal 30 min ephemeris validity period
 *  must be assumed.
 *  (We may have missed the string where the actual P1 flag was set)
 *  It is unknown if the validity period is 45 min or 60 min, but there
 *  is a 7.5 min time window where the ephemeris is guaranteed not to change.
 *  If all received strings are from the same 7.5 min window, we can
 *  declare all strings valid.
 *
 * \param n    GLO nav message decode state struct
 * \param skip Allow single string to be skipped in the check.
 *             [Zero-based index] i.e. string 1 would be index 0.
 *             Use GLO_SKIP_STRING_NONE if no string is skipped.
 *
 * \return TRUE, if all strings from same validity window, FALSE otherwise.
 */
static bool check_time_validity(const nav_msg_glo_t *n, u8 skip) {
  /* Return value */
  bool strings_valid = false;

  /* Convert t_b and t_k to seconds.
   * t_b is GLO equivalent of GPS Time of Ephemeris.
   * t_k is GLO equivalent of GPS Time of Week.
   * (t_k for GLO is actually Time of Day) */
  s32 tb_sec = n->toe.h * HOUR_SECS + n->toe.m * MINUTE_SECS + (u32)n->toe.s;
  s32 tk_sec = n->tk.h * HOUR_SECS + n->tk.m * MINUTE_SECS + (u32)n->tk.s;

  log_debug_mesid(n->mesid,
                  "Strings done? tb: %" PRIi32 " tk: %" PRIi32 ", win: %d",
                  tb_sec,
                  tk_sec,
                  MIN_VALIDITY_WINDOW_S);

  s32 time_diff_sec = ABS(tb_sec - tk_sec);
  /* Check the first case where t_b and t_k
   * are too far away from each other. */
  if (time_diff_sec >= MAX_VALIDITY_WINDOW_S) {
    return strings_valid;
    /* Check the second case where time diff of t_b and t_k
     * could indicate longer than nominal P1 ephemeris validity.
     * I.e. ephemeris validity of 45 or 60 min. */
  } else if (time_diff_sec >= MIN_VALIDITY_WINDOW_S) {
    if (check_validity_window(n, tk_sec, skip)) {
      strings_valid = true;
    }
    return strings_valid;
  }
  /* Check the third (nominal) case, where time diff of t_b and t_k
   * are within 15 min of each other. */

  /* String 1 (index 0) can be used to tie time tag
   * and GLO time (t_k) together */
  u32 end_time_s = n->string_receive_time_ms[0] / SECS_MS +
                   (tb_sec - tk_sec + MIN_VALIDITY_WINDOW_S);
  strings_valid = true;
  /* Compare time tag of each string against validity boundaries */
  for (u8 i = 0; i < GLO_STRINGS_TO_COLLECT; i++) {
    if (i == skip) {
      continue;
    }
    u32 receive_time_s = n->string_receive_time_ms[i] / SECS_MS;
    log_debug_mesid(n->mesid,
                    "CMP end: %" PRIu32 " rec: %" PRIu32 " diff: %" PRIu32 "",
                    end_time_s,
                    receive_time_s,
                    (end_time_s - receive_time_s));
    if ((receive_time_s > end_time_s) ||
        ((end_time_s - receive_time_s) > (MIN_VALIDITY_WINDOW_S * 2))) {
      strings_valid = false;
      break;
    }
  }
  return strings_valid;
}

/** Checks if strings needed for TOW decoding are received.
 *  tk from string 1
 *  tb from string 2
 *  nt from string 4
 *  n4 from string 5
 *  NOTE: string 3 can be skipped.
 *
 * \param n    GLO nav message decode state struct
 *
 * \return TRUE, if all 4 strings have been decoded and validated,
 *         FALSE otherwise.
 */
static bool is_tow_decode_done(const nav_msg_glo_t *n) {
  /* Return value */
  bool strings_valid = false;

  /* Mask string 3 which is not needed for tow */
  u8 strings = n->decoded_strings & 0x1B;

  /* Check that we have decoded all required strings */
  if (GLO_STRINGS_NEEDED_FOR_TOW != strings) {
    return strings_valid;
  }

  strings_valid = check_time_validity(n, GLO_SKIP_STRING_3);
  return strings_valid;
}

/** Checks if strings 1-5 needed for ephemeris have been decoded.
 *
 * \param n    GLO nav message decode state struct
 *
 * \return TRUE, if all 5 strings have been decoded and validated,
 *         FALSE otherwise.
 */
static bool is_eph_decode_done(const nav_msg_glo_t *n) {
  /* Return value */
  bool strings_valid = false;

  /* Check that we have decoded all required strings */
  if (GLO_STRINGS_NEEDED != n->decoded_strings) {
    return strings_valid;
  }

  strings_valid = check_time_validity(n, GLO_SKIP_STRING_NONE);
  return strings_valid;
}

static bool is_ephe_valid(const nav_msg_glo_t *n) {
  double pos_min_m = GLO_ORBIT_RADIUS_M - GLO_ORBIT_RADIUS_MARGIN_M;
  double pos_max_m = GLO_ORBIT_RADIUS_M + GLO_ORBIT_RADIUS_MARGIN_M;
  double pos_m = sqrt((n->eph.glo.pos[0] * n->eph.glo.pos[0]) +
                      (n->eph.glo.pos[1] * n->eph.glo.pos[1]) +
                      (n->eph.glo.pos[2] * n->eph.glo.pos[2]));
  if ((pos_m < pos_min_m) || (pos_max_m < pos_m)) {
    log_info_mesid(n->mesid, "GLO-NAV-ERR: position mag = %lf m", pos_m);
    return false;
  }

  double vel_min_m_s = GLO_VEL_MAG_M_S - GLO_VEL_MAG_MARGIN_M_S;
  double vel_max_m_s = GLO_VEL_MAG_M_S + GLO_VEL_MAG_MARGIN_M_S;
  double vel_m_s = sqrt((n->eph.glo.vel[0] * n->eph.glo.vel[0]) +
                        (n->eph.glo.vel[1] * n->eph.glo.vel[1]) +
                        (n->eph.glo.vel[2] * n->eph.glo.vel[2]));
  if ((vel_m_s < vel_min_m_s) || (vel_max_m_s < vel_m_s)) {
    log_info_mesid(n->mesid, "GLO-NAV-ERR: velocity mag = %lf m/s", vel_m_s);
    return false;
  }

  return true;
}

/** The function decodes a GLO navigation string.
 *  Assume we receive signal from GLONASS-M
 * \param n           Pointer to nav_msg_glo_t structure containing GLO string
 * \param time_tag_ms Time tag of the last data bit in the string [ms]
 * \return GLO_STRING_DECODE_ERROR  -- in case of an error.
 *         GLO_STRING_DECODE_STRING -- single string decoded,
 *         GLO_STRING_DECODE_TOW    -- all strings for TOW decoded,
 *         GLO_STRING_DECODE_EPH    -- all strings for ephemeris decoded
 *
 */
string_decode_status_t process_string_glo(nav_msg_glo_t *n, u32 time_tag_ms) {
  /* Extract and check dummy bit from GLO string, bit 85 in GLO string */
  if (extract_word_glo(n, GLO_STR_LEN, 1) != 0) {
    return GLO_STRING_DECODE_ERROR;
  }
  /* Extract string number */
  u32 m = extract_word_glo(n, 81, 4);
  /* Decode string specific data, we are interested only in strings 1-5 */
  if (m > GLO_STRINGS_TO_COLLECT) {
    return GLO_STRING_DECODE_STRING;
  }
  /* Check that time tag is valid.
   * At this point we should have received at least one full string.
   * I.e. time tag must be greater than 2 seconds. */
  if (time_tag_ms < (2 * SECS_MS)) {
    log_warn_mesid(
        n->mesid, "Time tag [ms]: %" PRIu32 " < 2 seconds", time_tag_ms);
    return GLO_STRING_DECODE_ERROR;
  }

  n->string_receive_time_ms[m - 1] = time_tag_ms; /* Time tag the string */

  switch (m) {
    case 1: /* string 1 */
      if (!extract_string_1_components(n)) {
        return GLO_STRING_DECODE_ERROR;
      }
      break;
    case 2: /* string 2 */
      if (!extract_string_2_components(n)) {
        return GLO_STRING_DECODE_ERROR;
      }
      break;
    case 3: /* string 3 */
      if (!extract_string_3_components(n)) {
        return GLO_STRING_DECODE_ERROR;
      }
      break;
    case 4: /* string 4 */
      if (!extract_string_4_components(n)) {
        return GLO_STRING_DECODE_ERROR;
      }
      break;
    case 5: /* string 5 */
      if (!extract_string_5_components(n)) {
        return GLO_STRING_DECODE_ERROR;
      }
      break;
    default:
      break;
  }

  /* Mark string as decoded */
  n->decoded_strings |= (1 << (m - 1));

  bool tow_ready = false;
  u32 tow_age_s;
  if (is_tow_decode_done(n) &&
      find_tow_age(n->string_receive_time_ms, &tow_age_s)) {
    /* find_tow_age() checks how many strings have been decoded
     * since the last received string 1.
     * Each string after last string 1, adds 2 seconds to the decoded TOW.
     *
     * Get GPS time at the end of the last received data bit.
     * We do not wait for the time mark of the string to come.
     * That's why we only account for the length of the data
     * part of the last string - 1.7s below.
     *
     * Also the Glonass time is converted to UTC first and then to GPS.
     * So it requires UTC parameters describing leap seconds difference
     * plus GPS to UTC time offset. Note that the conversion may return
     * an invalid time stamp if conversion parameters are not yet available.
     */

    n->gps_time = glo2gps_with_utc_params(n->mesid, &n->tk);
    n->gps_time.tow += (GLO_STR_LEN_S - GLO_STR_TIME_MARK_LEN_S) + tow_age_s;
    normalize_gps_time(&n->gps_time);
    tow_ready = gps_time_valid(&n->gps_time);
  }

  bool eph_ready = false;
  /* Have we decoded all five strings successfully? */
  if (is_eph_decode_done(n)) {
    /* if the SV is unhealthy we will not call is_ephe_valid due to
     * lazy evaluation, so no return under "if" happens */
    if (!n->eph.health_bits && !is_ephe_valid(n)) {
      return GLO_STRING_DECODE_ERROR;
    }
    /* Full decoding done, start over */
    restart_decoding(n);
    eph_ready = true;
  }

  if (eph_ready && tow_ready) {
    return GLO_STRING_DECODE_EPH;
  } else if (tow_ready) {
    return GLO_STRING_DECODE_TOW;
  } else {
    return GLO_STRING_DECODE_STRING;
  }
}
