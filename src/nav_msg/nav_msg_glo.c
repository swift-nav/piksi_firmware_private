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
#include "nav_msg/nav_msg_glo.h"

#include <assert.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <swiftnav/bits.h>
#include <swiftnav/constants.h>
#include <swiftnav/decode_glo.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/logging.h>

#include "timing/timing.h"

#define BIT_POLARITY_NORMAL 0
#define BIT_POLARITY_INVERTED 1
#define BIT_POLARITY_UNKNOWN (-1)

/* Minimum one-sided data validity time around toe value in seconds. */
#define MIN_VALIDITY_WINDOW_S (15 * MINUTE_SECS)
/* Maximum one-sided data validity time around toe value in seconds. */
#define MAX_VALIDITY_WINDOW_S (30 * MINUTE_SECS)
/* The time difference of one-sided data validity windows in seconds.
 * Difference between 15 min vs. 22.5 min vs. 30 min is 7.5 min. */
#define DIFF_VALIDITY_WINDOW_S ((u32)(7.5f * MINUTE_SECS))

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
  n->string.word[0] <<= 1; /* use one word of buffer for that purpose */
  n->string.word[0] |= symbol;
  /* collected symbols match time mark? if not stay at this state */
  u32 tm = extract_word_glo(&n->string, 1, GLO_TM_LEN_SYMBOLS);
  if ((GLO_TM != tm) && (GLO_TM_INV != tm)) {
    return false;
  }

  /* time mark found, next time start collecting data symbols */
  n->meander_bits_cnt = 0;
  n->manchester = 0;
  n->state = GET_DATA_BIT;
  n->string.word[0] = 0;
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
  for (u8 i = GLO_NAV_STR_BITS - 1; i > 0; i--) {
    u32 tmp =
        (n->string.word[i] << 1) | ((n->string.word[i - 1] & (1u << 31)) >> 31);
    n->string.word[i] = tmp;
  }
  n->string.word[0] <<= 1;

  /* set type of meander depending on inversion */
  u8 meander = (BIT_POLARITY_NORMAL == n->bit_polarity) ? 1 : 2;
  /* meander removal */
  u8 bit = (n->manchester ^ meander) & 1;
  /* relative code removal */
  u8 relcode = relcode_decode(&n->relcode, bit);
  /* store bit to buffer */
  n->string.word[0] |= relcode;
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

/** Find out how many strings have been decoded after last string 1.
 *  Each additional string adds 2 seconds to the decoded TOW.
 *
 * \param string_receive_time Array containing the latest time tag
 *                            for each string.
 * \param tow_age             TOW age in seconds.
 * \return TRUE, if time tags are ok, FALSE otherwise.
 */
static bool find_tow_age(
    const u32 string_receive_time_ms[GLO_STRINGS_TO_COLLECT], u32 *tow_age_s) {
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
  u32 tag_diff_s = (u32)lrintf((max_tag_ms - tow_tag_ms) / (float)SECS_MS);

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
  n->eph.toe = glo2gps_with_utc_params(&n->toe, /* ref_time = */ NULL);

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
    s32 tag_diff_s = lrintf(tag_diff_ms / (float)SECS_MS);
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
  }
  if (time_diff_sec >= MIN_VALIDITY_WINDOW_S) {
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
  if (extract_word_glo(&n->string, GLO_STR_LEN, 1) != 0) {
    return GLO_STRING_DECODE_ERROR;
  }
  /* Extract string number */
  u32 m = extract_word_glo(&n->string, 81, 4);
  /* Decode string specific data, we are interested only in strings 1-5 */
  /* According to ICD L1,L2 GLONASS edition 5.1 2008 Table 4.5 the valid
     range of m is [0..15] */
  if ((0 == m) || (m > GLO_STRINGS_TO_COLLECT)) {
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
      if (!decode_glo_string_1(&n->string, &n->eph, &n->tk)) {
        return GLO_STRING_DECODE_ERROR;
      }
      break;
    case 2: /* string 2 */
      if (!decode_glo_string_2(&n->string, &n->eph, &n->toe)) {
        return GLO_STRING_DECODE_ERROR;
      }
      break;
    case 3: /* string 3 */
      if (!decode_glo_string_3(&n->string, &n->eph, &n->toe)) {
        return GLO_STRING_DECODE_ERROR;
      }
      break;
    case 4: /* string 4 */
      if (!decode_glo_string_4(
              &n->string, &n->eph, &n->tk, &n->toe, &n->age_of_data_days)) {
        return GLO_STRING_DECODE_ERROR;
      }
      break;
    case 5: /* string 5 */
      if (!decode_glo_string_5(
              &n->string, &n->eph, &n->tk, &n->toe, &n->tau_gps_s)) {
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
     */
    double time_offset = (GLO_STR_LEN_S - GLO_STR_TIME_MARK_LEN_S) + tow_age_s;

    /* expected GPS time at the end of last bit */
    gps_time_t ref_time = GPS_TIME_UNKNOWN;
    if (TIME_COARSE <= get_time_quality()) {
      ref_time = get_current_time();
      ref_time.tow -= time_offset;
      normalize_gps_time(&ref_time);
    }

    /* Glonass time is converted to UTC first and then to GPS.
     * First apply the integer leap second offset from UTC parameters.
     * (The fractional GPS to UTC time offset tau_gps will be applied separately
     * as constellation time offset.) Note that this conversion may return
     * an invalid time stamp if conversion parameters are not yet available.
     */
    n->gps_time = glo2gps_with_utc_params(&n->tk, &ref_time);
    n->gps_time.tow += time_offset;
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
  }
  if (tow_ready) {
    return GLO_STRING_DECODE_TOW;
  }
  return GLO_STRING_DECODE_STRING;
}
