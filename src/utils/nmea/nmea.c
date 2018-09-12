/*
 * Copyright (C) 2013-2017 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <assert.h>
#include <inttypes.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <libswiftnav/pvt_engine/firmware_binding.h>
#include <swiftnav/array_tools.h>
#include <swiftnav/constants.h>
#include <swiftnav/coord_system.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/logging.h>

#include "board/nap/track_channel.h"
#include "calc_pvt_me.h"
#include "io_support.h"
#include "main.h"
#include "nmea.h"
#include "piksi_systime.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "settings/settings.h"
#include "starling_integration.h"
#include "timing/timing.h"
#include "track/track_sid_db.h"

static u32 gpgga_msg_rate = 1; /* By design GGA should be output at the
                                  solution rate. */
static u32 gpgsv_msg_rate = 10;
static u32 gprmc_msg_rate = 10;
static u32 gpvtg_msg_rate = 1;
static u32 gphdt_msg_rate = 1;
static u32 gpgll_msg_rate = 10;
static u32 gpzda_msg_rate = 10;
static u32 gsa_msg_rate = 10;

/** \addtogroup io
 * \{ */

/** \defgroup nmea NMEA
 * Send messages in NMEA 2.30 format.
 * \{ */

/* SBAS NMEA SV IDs are from 33 to 54 */
#define NMEA_SV_ID_OFFSET_SBAS (-87)

/* GLO NMEA SV IDs are from 65 to 96 */
#define NMEA_SV_ID_OFFSET_GLO (64)

/* GAL NMEA SV IDs are from 301 to 336 */
#define NMEA_SV_ID_OFFSET_GAL (300)

/* BDS NMEA SV IDs are from 401 to 437 */
#define NMEA_SV_ID_OFFSET_BDS2 (400)

/* Max SVs reported per GSA message */
#define GSA_MAX_SV 12

/* Number of decimals in NMEA time stamp (valid values 1-4) */
#define NMEA_UTC_S_DECIMALS 2
#define NMEA_UTC_S_FRAC_DIVISOR pow(10, NMEA_UTC_S_DECIMALS)

/* Adequate until end of year 999999.
   Worst case: "%02d%02d%05.2f,%02d,%02d,%lu" */
#define NMEA_TS_MAX_LEN (21 + NMEA_UTC_S_DECIMALS)

/* Accuracy of Course Over Ground */
#define NMEA_COG_DECIMALS 1
#define NMEA_COG_FRAC_DIVISOR pow(10, NMEA_COG_DECIMALS)

/* Based on testing calculated Course Over Ground starts deviating noticeably
 * below this limit. */
#define NMEA_COG_STATIC_LIMIT_MS 0.1f
#define NMEA_COG_STATIC_LIMIT_KNOTS MS2KNOTS(NMEA_COG_STATIC_LIMIT_MS, 0, 0)
#define NMEA_COG_STATIC_LIMIT_KPH MS2KMHR(NMEA_COG_STATIC_LIMIT_MS, 0, 0)

typedef enum talker_id_e {
  TALKER_ID_INVALID = -1,
  TALKER_ID_GP = 0,
  TALKER_ID_GL = 1,
  TALKER_ID_GA = 2,
  TALKER_ID_GB = 3,
  TALKER_ID_COUNT = 4
} talker_id_t;

#define NMEA_SUFFIX_LEN                    \
  6 /* How much room to leave for the NMEA \
       checksum, CRLF + null termination,  \
       i.e. "*%02X\r\n\0" */

/** Some helper macros for functions generating NMEA sentences. */

/** NMEA_SENTENCE_START: declare a buffer and set up some pointers
 * max_len = max possible length of the body of the message
 * (not including suffix)
 */
#define NMEA_SENTENCE_START(max_len)            \
  char sentence_buf[max_len + NMEA_SUFFIX_LEN]; \
  char *sentence_bufp = sentence_buf;           \
  char *const sentence_buf_end = sentence_buf + max_len;

/** NMEA_SENTENCE_PRINTF: use like printf, can use multiple times
    within a sentence. */
#define NMEA_SENTENCE_PRINTF(fmt, ...)                                        \
  do {                                                                        \
    sentence_bufp += snprintf(                                                \
        sentence_bufp, sentence_buf_end - sentence_bufp, fmt, ##__VA_ARGS__); \
    if (sentence_bufp >= sentence_buf_end) sentence_bufp = sentence_buf_end;  \
  } while (0)

/** NMEA_SENTENCE_DONE: append checksum and dispatch.
 * \note According to section 5.3.1 of the NMEA 0183 spec, sentences are
 *       terminated with <CR><LF>. The sentence_buf is null_terminated.
 *       The call to nmea_output has been modified to remove the NULL.
 *       This will also affect all registered dispatchers
 */
#define NMEA_SENTENCE_DONE()                                         \
  do {                                                               \
    if (sentence_bufp == sentence_buf_end)                           \
      log_warn("NMEA %.6s cut off", sentence_buf);                   \
    nmea_append_checksum(sentence_buf, sizeof(sentence_buf));        \
    nmea_output(sentence_buf,                                        \
                sentence_bufp - sentence_buf + NMEA_SUFFIX_LEN - 1); \
  } while (0)

/** Output NMEA sentence.

 * \param s The NMEA sentence to output.
 * \param size This is the C-string size, not including the null character
 */
static void nmea_output(char *s, size_t size) {
  static MUTEX_DECL(send_mutex);
  chMtxLock(&send_mutex);

  io_support_write(SD_NMEA, (u8 *)s, size);

  chMtxUnlock(&send_mutex);
}

void nmea_setup(void) {
  SETTING("nmea", "gpgga_msg_rate", gpgga_msg_rate, TYPE_INT);
  SETTING("nmea", "gpgsv_msg_rate", gpgsv_msg_rate, TYPE_INT);
  SETTING("nmea", "gprmc_msg_rate", gprmc_msg_rate, TYPE_INT);
  SETTING("nmea", "gpvtg_msg_rate", gpvtg_msg_rate, TYPE_INT);
  SETTING("nmea", "gphdt_msg_rate", gphdt_msg_rate, TYPE_INT);
  SETTING("nmea", "gpgll_msg_rate", gpgll_msg_rate, TYPE_INT);
  SETTING("nmea", "gpzda_msg_rate", gpzda_msg_rate, TYPE_INT);
  SETTING("nmea", "gsa_msg_rate", gsa_msg_rate, TYPE_INT);
}

/** Calculate and append the checksum of an NMEA sentence.
 * Calculates the bitwise XOR of the characters in a string until the end of
 * the string or a `*` is encountered. If the first character is `$` then it
 * is skipped.
 *
 * \param s A null-terminated NMEA sentence, up to and optionally
 * including the '*'
 *
 * \param size Length of the buffer.
 *
 */
static void nmea_append_checksum(char *s, size_t size) {
  u8 sum = 0;
  char *p = s;

  /* '$' header not included in checksum calculation */
  if (*p == '$') {
    p++;
  }

  /* '*'  not included in checksum calculation */
  while (*p != '*' && *p && p + NMEA_SUFFIX_LEN < s + size) {
    sum ^= *p;
    p++;
  }

  sprintf(p, "*%02X\r\n", sum);
}

/* General note: the NMEA functions below mask the time source, position and
   velocity modes to ensure prevention of accidental bugs in the future.

   The SBP specification only specifies the first 3 bits to indicate the mode.
   Thus the RAIM repair flag or other future flags need to be masked out.

   It is needed in this code becasue we pass SBP formatted structures, not the
   raw positioning modes.
*/

static u16 nmea_get_id(const gnss_signal_t sid) {
  u16 id = -1;

  switch (sid_to_constellation(sid)) {
    case CONSTELLATION_BDS:
      id = NMEA_SV_ID_OFFSET_BDS2 + sid.sat;
      break;
    case CONSTELLATION_GAL:
      id = NMEA_SV_ID_OFFSET_GAL + sid.sat;
      break;
    case CONSTELLATION_GPS:
      id = sid.sat;
      break;
    case CONSTELLATION_GLO:
      id = NMEA_SV_ID_OFFSET_GLO + sid.sat;
      break;
    case CONSTELLATION_SBAS:
      id = NMEA_SV_ID_OFFSET_SBAS + sid.sat;
      break;
    case CONSTELLATION_QZS:
    case CONSTELLATION_COUNT:
    case CONSTELLATION_INVALID:
    default:
      log_error("NMEA: Unsupported constellation");
      break;
  }

  return id;
}

static const char *talker_id_to_str(const talker_id_t id) {
  switch (id) {
    case TALKER_ID_GA:
      return "GA";
    case TALKER_ID_GB:
      return "GB";
    case TALKER_ID_GL:
      return "GL";
    case TALKER_ID_GP:
      return "GP";
    case TALKER_ID_COUNT:
    case TALKER_ID_INVALID:
    default:
      log_debug("talker_id_to_str() error: invalid talker ID");
      return "";
  }
}

static talker_id_t sid_to_talker_id(const gnss_signal_t sid) {
  switch (sid_to_constellation(sid)) {
    case CONSTELLATION_GAL:
      return TALKER_ID_GA;
    case CONSTELLATION_BDS:
      return TALKER_ID_GB;
    case CONSTELLATION_GLO:
      return TALKER_ID_GL;
    case CONSTELLATION_GPS:
    case CONSTELLATION_QZS:
    case CONSTELLATION_SBAS:
      return TALKER_ID_GP;
    case CONSTELLATION_INVALID:
    case CONSTELLATION_COUNT:
    default:
      log_debug("sid_to_talker_id() error: unsupported constellation");
      return TALKER_ID_INVALID;
  }
}

/** Helper function for nmea_gsv for comparing sids.
 *
 * \param[in] a     ptr to left side sid
 * \param[in] b     ptr to right side sid
 *
 * Return values:
 *   <0 The element pointed to by a goes before the element pointed to by b
 *   0  The element pointed to by a is equivalent to the element pointed to by b
 *   >0 The element pointed to by a goes after the element pointed to by b
 */
int compare_ch_meas(const void *a, const void *b) {
  const channel_measurement_t **ca = (const channel_measurement_t **)a;
  const channel_measurement_t **cb = (const channel_measurement_t **)b;

  return nmea_get_id((*ca)->sid) - nmea_get_id((*cb)->sid);
}

/** Print a NMEA GSV message string and send it out NMEA USARTs.
 * NMEA GSV  message contains GNSS Satellites In View (in this case observed).
 *
 * \param[in] n_used      size of ch_meas
 * \param[in] ch_meas     array of ch_measurement structs from SVs in track
 * \param[in] talker      indicator which talker ID to use
 */
static void nmea_gsv_print(const u8 n_used,
                           const channel_measurement_t *ch_meas[],
                           const talker_id_t talker) {
  const char *talker_str = talker_id_to_str(talker);

  qsort(ch_meas, n_used, sizeof(channel_measurement_t *), compare_ch_meas);

  u8 n_messages = (n_used + 3) / 4;

  u8 n = 0;
  double ele;
  double azi;

  for (u8 i = 0; i < n_messages; i++) {
    NMEA_SENTENCE_START(120);
    NMEA_SENTENCE_PRINTF(
        "$%sGSV,%u,%u,%02u", talker_str, n_messages, i + 1, n_used);

    for (u8 j = 0; j < 4 && n < n_used; n++) {
      u16 sv_id = nmea_get_id(ch_meas[n]->sid);

      NMEA_SENTENCE_PRINTF(",%02u", sv_id);

      if (track_sid_db_elevation_degrees_get(ch_meas[n]->sid, &ele)) {
        NMEA_SENTENCE_PRINTF(",%02d", (s8)round(ele));
      } else {
        NMEA_SENTENCE_PRINTF(",");
      }

      if (track_sid_db_azimuth_degrees_get(ch_meas[n]->sid, &azi)) {
        NMEA_SENTENCE_PRINTF(",%03u", (u16)round(azi));
      } else {
        NMEA_SENTENCE_PRINTF(",");
      }

      NMEA_SENTENCE_PRINTF(",%02u", (u8)roundf(ch_meas[n]->cn0));

      j++; /* 4 sats per message no matter what */
    }
    NMEA_SENTENCE_DONE();
  }
}

/** Group tracked SVs by constellation and forward information to GSV printing
 * function.
 *
 * \note NMEA 0183 - Standard For Interfacing Marine Electronic Devices
 *       versions 2.30, 3.01 and 4.10 state following:
 *       If multiple GPS, GLONASS, Galileo, etc. satellites are in view, use
 *       separate GSV sentences with talker ID GP to show the GPS satellites in
 *       view and talker GL to show the GLONASS satellites in view and talker GA
 *       to show the Galileo satellites in view, etc. When more than ranging
 *       signal is used per satellite, also use separate GSV sentences with a
 *       Signal ID corresponding to the ranging signal. The GN identifier shall
 *       not be used with this sentence!
 *
 * \param[in] n_used      size of ch_meas
 * \param[in] ch_meas     array of ch_measurement structs from SVs in track
 */
static void nmea_gsv(u8 n_used, const channel_measurement_t *ch_meas) {
  /* Group by constellation */
  const channel_measurement_t *ch_meas_grouped[TALKER_ID_COUNT][n_used];
  u8 num_ch_meas[TALKER_ID_COUNT] = {0};

  for (u8 i = 0; i < n_used; ++i) {
    talker_id_t id = sid_to_talker_id(ch_meas[i].sid);

    if (TALKER_ID_INVALID == id) {
      /* Unsupported constellation */
      continue;
    }

    /* check if sat is already picked up from another code */
    bool in_array = false;
    for (u8 j = 0; j < num_ch_meas[id]; ++j) {
      if (ch_meas_grouped[id][j]->sid.sat == ch_meas[i].sid.sat) {
        in_array = true;
        break;
      }
    }

    if (in_array) {
      /* SV already on the list */
      continue;
    }

    ch_meas_grouped[id][num_ch_meas[id]++] = &ch_meas[i];
  }

  /* Print grouped sentences */
  u8 talkers = 0;
  for (u8 i = 0; i < TALKER_ID_COUNT; ++i) {
    if (0 == num_ch_meas[i]) {
      continue;
    }
    nmea_gsv_print(num_ch_meas[i], ch_meas_grouped[i], i);
    talkers++;
  }

  /* Check if anything was printed */
  if (0 == talkers) {
    /* Print bare minimum */
    NMEA_SENTENCE_START(120);
    NMEA_SENTENCE_PRINTF("$GPGSV,1,1,0");
    NMEA_SENTENCE_DONE();
  }
}

/** Generate and send periodic GPGSV and GLGSV.
 *
 * \param[in] n_used      size of ch_meas
 * \param[in] ch_meas     array of channel_measurement structs from tracked SVs
 */
void nmea_send_gsv(u8 n_used, const channel_measurement_t *ch_meas) {
  DO_EVERY(gpgsv_msg_rate, nmea_gsv(n_used, ch_meas));
}

bool send_nmea(u32 rate, u32 gps_tow_ms) {
  if (rate == 0) {
    return false;
  }

  /* If the modulo of latest gps time estimate time with configured
  * output period is less than 1/2 the solution period we should send the NMEA
  * message.
  * This way, we still send no_fix messages when receiver clock is drifting. */
  u32 soln_period_ms = (u32)(1.0 / soln_freq_setting * 1e3);
  u32 output_period_ms = (u32)soln_period_ms * rate;
  if ((gps_tow_ms % output_period_ms) < (soln_period_ms / 2)) {
    return true;
  }
  return false;
}

/** Convert the SBP status flag into NMEA Status field.
 * Ref: NMEA-0183 version 2.30 pp.42,43
 *
 * \param flags        u8 sbp_pos_llh->flags
 */
char get_nmea_status(u8 flags) {
  switch (flags & POSITION_MODE_MASK) {
    case POSITION_MODE_NONE:
      return 'V';
    case POSITION_MODE_SPP: /* autonomous mode */
    case POSITION_MODE_DGNSS:
    case POSITION_MODE_SBAS:
    case POSITION_MODE_FLOAT:
    case POSITION_MODE_FIXED:
      return 'A';
    default:
      assert(!"Unsupported position type indicator");
      return 'V';
  }
}

/** Convert the SBP status flag into NMEA Mode Indicator field:
 * Ref: NMEA-0183 version 2.30 pp.42,43
 *
 * \param flags        u8 sbp_pos_llh->flags
 */
char get_nmea_mode_indicator(u8 flags) {
  switch (flags & POSITION_MODE_MASK) {
    case POSITION_MODE_NONE:
      return 'N';
    case POSITION_MODE_SPP: /* autonomous mode */
      return 'A';
    case POSITION_MODE_DGNSS: /* differential mode */
    case POSITION_MODE_SBAS:
    case POSITION_MODE_FLOAT:
    case POSITION_MODE_FIXED:
      return 'D';
    default:
      assert(!"Unsupported position type indicator");
      return 'N';
  }
}

/** \} */

/** \} */
