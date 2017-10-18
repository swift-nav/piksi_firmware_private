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
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <libswiftnav/constants.h>
#include <libswiftnav/coord_system.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/observation.h>
#include <libswiftnav/time.h>

#include "board/nap/track_channel.h"
#include "io_support.h"
#include "main.h"
#include "me_calc_pvt.h"
#include "nmea.h"
#include "piksi_systime.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "settings.h"
#include "starling_calc_pvt.h"
#include "timing.h"
#include "track.h"

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

/** Wrapper function for vsnprintf to accommodate return value handling. Updates
 *  buf_ptr accordingly if there's no encoding error. If buffer is full, sets
 *  buf_ptr to buf_end.
 */
static void vsnprintf_wrap(char **buf_ptr,
                           char *buf_end,
                           const char *format,
                           ...) {
  va_list args;
  va_start(args, format);

  int res = vsnprintf(*buf_ptr, buf_end - *buf_ptr, format, args);

  va_end(args);

  if (res < 0) {
    log_warn("Error %d in vsnprintf_wrap()", res);
    return;
  }

  *buf_ptr += res;

  if (*buf_ptr >= buf_end) {
    log_warn("Buffer overflow in vsnprintf_wrap()");
    *buf_ptr = buf_end;
  }
}

/** Generate UTC date time string. Time field is before date field.
 *
 * \param[in] sbp_msg_time Time and date to create the str from.
 * \param[in] time Time field is to be added.
 * \param[in] date Date field is to be added.
 * \param[in] trunc_date Truncate date field. No effect if param date is false.
 * \param[in] t UTC time structure
 * \param[out] utc_str Created date time string.
 * \param[in] size utc_str size.
 *
 */
static void get_utc_time_string(const msg_gps_time_t *sbp_msg_time,
                                bool time,
                                bool date,
                                bool trunc_date,
                                const utc_tm *t,
                                char *utc_str,
                                u8 size) {
  char *buf_end = utc_str + size;

  if ((sbp_msg_time->flags & TIME_SOURCE_MASK) == NO_TIME) {
    if (time) {
      vsnprintf_wrap(&utc_str, buf_end, ","); /* Time (UTC) */
    }

    if (date) {
      vsnprintf_wrap(
          &utc_str, buf_end, trunc_date ? "," : ",,,"); /* Date Stamp */
    }

    return;
  }

  if (time) {
    /* Time (UTC) */
    vsnprintf_wrap(&utc_str,
                   buf_end,
                   "%02u%02u%02u.%0*u,",
                   t->hour,
                   t->minute,
                   t->second_int,
                   NMEA_UTC_S_DECIMALS,
                   (u16)roundf(NMEA_UTC_S_FRAC_DIVISOR * t->second_frac));
  }

  if (date) {
    /* Date Stamp */

    if (trunc_date) {
      vsnprintf_wrap(&utc_str,
                     buf_end,
                     "%02u%02u%02u,",
                     t->month_day,
                     t->month,
                     (u8)(t->year % 100));
    } else {
      vsnprintf_wrap(&utc_str,
                     buf_end,
                     "%02u,%02u,%" PRIu32 ",",
                     t->month_day,
                     t->month,
                     t->year);
    }
  }
}

/* General note: the NMEA functions below mask the time source, position and
   velocity modes to ensure prevention of accidental bugs in the future.

   The SBP specification only specifies the first 3 bits to indicate the mode.
   Thus the RAIM repair flag or other future flags need to be masked out.

   It is needed in this code becasue we pass SBP formatted structures, not the
   raw positioning modes.
*/

/** Assemble a NMEA GPGGA message and send it out NMEA USARTs.
 * NMEA GPGGA message contains Global Positioning System Fix Data.
 *
 * \param sbp_pos_llh       SBP LLH position messages
 * \param sbp_msg_time      SBP GPS Time message
 * \param utc_time          Pointer to UTC time
 * \param sbp_dops          SBP DOP Message for this epoch
 * \param propagation_time  Age of differential corrections [s].
 * \param station_id        Differential reference station ID.
 */
void nmea_gpgga(const msg_pos_llh_t *sbp_pos_llh,
                const msg_gps_time_t *sbp_msg_time,
                const utc_tm *utc_time,
                const msg_dops_t *sbp_dops,
                double propagation_time,
                u8 station_id) {
  /* GGA sentence is formed by splitting latitude and longitude
     into degrees and minutes parts and then printing them separately
     using printf. Before doing the split we want to take care of
     the proper rounding. Doing it after the split would lead to the need
     of handling the case of minutes part overflow separately. Otherwise,
     printf would do the rounding of the minutes part, which could result
     in printing 60 minutes value.
     E.g. doing this way lat = 15.9999999996 would be printed as
     $GPGGA,hhmmss.ss,1600.000000,...
     and NOT
     $GPGGA,hhmmss.ss,1560.000000,...
   */
  double lat = fabs(round(sbp_pos_llh->lat * 1e8) / 1e8);
  double lon = fabs(round(sbp_pos_llh->lon * 1e8) / 1e8);

  char lat_dir = sbp_pos_llh->lat < 0.0 ? 'S' : 'N';
  assert(lat <= UINT16_MAX);
  u16 lat_deg = (u16)lat; /* truncation towards zero */
  double lat_min = (lat - (double)lat_deg) * 60.0;

  char lon_dir = sbp_pos_llh->lon < 0.0 ? 'W' : 'E';
  assert(lon <= UINT16_MAX);
  u16 lon_deg = (u16)lon; /* truncation towards zero */
  double lon_min = (lon - (double)lon_deg) * 60.0;

  u8 fix_type = NMEA_GGA_QI_INVALID;
  if ((sbp_msg_time->flags & POSITION_MODE_MASK) != NO_POSITION) {
    fix_type = get_nmea_quality_indicator(sbp_pos_llh->flags);
  }

  NMEA_SENTENCE_START(120);
  NMEA_SENTENCE_PRINTF("$GPGGA,");

  char utc[NMEA_TS_MAX_LEN];
  get_utc_time_string(
      sbp_msg_time, true, false, false, utc_time, utc, NMEA_TS_MAX_LEN);
  NMEA_SENTENCE_PRINTF("%s", utc);

  if (fix_type != NMEA_GGA_QI_INVALID) {
    NMEA_SENTENCE_PRINTF("%02u%010.7f,%c,%03u%010.7f,%c,",
                         lat_deg,
                         lat_min,
                         lat_dir,
                         lon_deg,
                         lon_min,
                         lon_dir);
  } else {
    NMEA_SENTENCE_PRINTF(",,,,");
  }

  NMEA_SENTENCE_PRINTF("%01d,", fix_type);

  if (fix_type != NMEA_GGA_QI_INVALID) {
    NMEA_SENTENCE_PRINTF("%02d,%.1f,%.2f,M,0.0,M,",
                         sbp_pos_llh->n_sats,
                         round(10 * sbp_dops->hdop * 0.01) / 10,
                         sbp_pos_llh->height);
  } else {
    NMEA_SENTENCE_PRINTF(",,,M,,M,");
  }

  if ((fix_type == NMEA_GGA_QI_DGPS) || (fix_type == NMEA_GGA_QI_FLOAT) ||
      (fix_type == NMEA_GGA_QI_RTK)) {
    NMEA_SENTENCE_PRINTF("%.1f,%04d",
                         propagation_time,
                         station_id & 0x3FF); /* ID range is 0000 to 1023 */
  } else {
    NMEA_SENTENCE_PRINTF(",");
  }

  NMEA_SENTENCE_DONE();
}

int gsa_cmp(const void *a, const void *b) { return (*(u8 *)a - *(u8 *)b); }

/** Assemble a NMEA GSA message and send it out NMEA USARTs.
 * NMEA GSA message contains GNSS DOP and Active Satellites.
 *
 * \param prns      Array of PRNs to output.
 * \param num_prns  Number of valid PRNs in array.
 * \param fix       Fix indicator.
 * \param sbp_dops  Pointer to SBP MSG DOP struct (PDOP, HDOP, VDOP).
 * \param cons      Working constellation.
 */
void nmea_gsa(u8 *prns,
              const u8 num_prns,
              const bool fix,
              const msg_dops_t *sbp_dops,
              const constellation_t cons) {
  assert(prns);
  assert(sbp_dops);

  /* Our fix is always 3D */
  char fix_mode = fix ? '3' : '1';

  NMEA_SENTENCE_START(120);

  if (CONSTELLATION_GPS == cons) {
    NMEA_SENTENCE_PRINTF("$GPGSA,A,%c,", fix_mode); /* Always automatic mode */
  } else if (CONSTELLATION_GLO == cons) {
    NMEA_SENTENCE_PRINTF("$GLGSA,A,%c,", fix_mode); /* Always automatic mode */
  } else {
    assert(!"Unknown constellation");
  }

  qsort(prns, num_prns, sizeof(u8), gsa_cmp);

  for (u8 i = 0; i < GSA_MAX_SV; i++) {
    if (i < num_prns) {
      NMEA_SENTENCE_PRINTF("%02d,", prns[i]);
    } else {
      NMEA_SENTENCE_PRINTF(",");
    }
  }

  if (fix && (NULL != sbp_dops)) {
    NMEA_SENTENCE_PRINTF("%.1f,%.1f,%.1f",
                         round(sbp_dops->pdop * 0.1) / 10,
                         round(sbp_dops->hdop * 0.1) / 10,
                         round(sbp_dops->vdop * 0.1) / 10);
  } else {
    NMEA_SENTENCE_PRINTF(",,");
  }

  NMEA_SENTENCE_DONE();
}

/** Helper function for nmea_gsv for comparing sids. Function assumes
 *  parameter sids have same constellation.
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

  assert(sid_to_constellation((*ca)->sid) == sid_to_constellation((*cb)->sid));

  return (*ca)->sid.sat - (*cb)->sid.sat;
}

/** Assemble a NMEA GPGSV or GLGSV message and send it out NMEA USARTs.
 * NMEA GPGSV and GLGSV message contains GNSS Satellites In View
 * (in this case observed)
 *
 * \param[in] n_used      size of ch_meas
 * \param[in] ch_meas     array of ch_measurement structs from SVs in track
 * \param[in] gnss        GNSS type
 */
void nmea_gsv(u8 n_used,
              const channel_measurement_t *ch_meas,
              constellation_t gnss) {
  const channel_measurement_t *ch_meas_gnss[n_used];

  if (NULL == ch_meas) {
    n_used = 0;
  }

  u8 n_gnss_used = 0;
  for (u8 i = 0; i < n_used; i++) {
    if (gnss != sid_to_constellation(ch_meas[i].sid)) {
      continue;
    }

    /* check if sat is already picked up from another GPS code */
    bool in_array = false;
    for (u8 j = 0; j < n_gnss_used; j++) {
      if (ch_meas_gnss[j]->sid.sat == ch_meas[i].sid.sat) {
        in_array = true;
        break;
      }
    }

    /* for extra security */
    if (!sid_valid(ch_meas[i].sid)) {
      log_debug_sid(ch_meas[i].sid, "Invalid SV in nmea_gsv()");
      continue;
    }

    if (!in_array) {
      ch_meas_gnss[n_gnss_used++] = &ch_meas[i];
    }
  }

  char *gnss_s = "";
  if (CONSTELLATION_GPS == gnss) {
    gnss_s = "GPGSV";
  } else if (CONSTELLATION_GLO == gnss) {
    gnss_s = "GLGSV";
  } else {
    assert(!"Unsupported GNSS type");
  }

  if (0 == n_gnss_used) {
    NMEA_SENTENCE_START(120);
    NMEA_SENTENCE_PRINTF("$%s,1,1,%02u", gnss_s, n_gnss_used);
    NMEA_SENTENCE_DONE();
    return;
  }

  qsort(ch_meas_gnss,
        n_gnss_used,
        sizeof(channel_measurement_t *),
        compare_ch_meas);

  u8 n_messages = (n_gnss_used + 3) / 4;

  u8 n = 0;

  for (u8 i = 0; i < n_messages; i++) {
    NMEA_SENTENCE_START(120);
    NMEA_SENTENCE_PRINTF(
        "$%s,%u,%u,%02u", gnss_s, n_messages, i + 1, n_gnss_used);

    for (u8 j = 0; j < 4 && n < n_gnss_used; n++) {
      s8 ele = sv_elevation_degrees_get(ch_meas_gnss[n]->sid);
      u16 azi = sv_azimuth_degrees_get(ch_meas_gnss[n]->sid);

      u16 sv_id = 0;
      if (CONSTELLATION_GPS == gnss) {
        sv_id = ch_meas_gnss[n]->sid.sat;
      } else if (CONSTELLATION_GLO == gnss) {
        sv_id = NMEA_SV_ID_GLO(ch_meas_gnss[n]->sid.sat);
      } else {
        assert(!"Unsupported GNSS type");
      }

      NMEA_SENTENCE_PRINTF(",%02u", sv_id);

      if (TRACKING_ELEVATION_UNKNOWN == ele) {
        NMEA_SENTENCE_PRINTF(",");
      } else {
        NMEA_SENTENCE_PRINTF(",%02d", ele);
      }

      if (TRACKING_AZIMUTH_UNKNOWN == azi) {
        NMEA_SENTENCE_PRINTF(",");
      } else {
        NMEA_SENTENCE_PRINTF(",%03u", azi);
      }

      NMEA_SENTENCE_PRINTF(",%02u", (u8)roundf(ch_meas_gnss[n]->cn0));

      j++; /* 4 sats per message no matter what */
    }
    NMEA_SENTENCE_DONE();
  }
}

/** Calculate Course and Speed Over Ground values.
 *
 * \param[in]  sbp_vel_ned  pointer to sbp vel ned struct
 * \param[out] cog          true course over ground [deg]
 * \param[out] sog_knots    speed over ground [knots]
 * \param[out] sog_kph      speed over ground [kph]
 */
static void calc_cog_sog(const msg_vel_ned_t *sbp_vel_ned,
                         double *cog,
                         double *sog_knots,
                         double *sog_kph) {
  double vel_north_ms = MM2M(sbp_vel_ned->n);
  double vel_east_ms = MM2M(sbp_vel_ned->e);

  *cog = R2D * atan2(vel_east_ms, vel_north_ms);

  /* Convert negative values to positive */
  if (*cog < 0.0) {
    *cog += FULL_CIRCLE_DEG;
  }

  /* Rounding to specified accuracy */
  *cog = roundf(*cog * NMEA_COG_FRAC_DIVISOR) / NMEA_COG_FRAC_DIVISOR;

  /* Avoid having duplicate values for same point (0 and 360) */
  if (fabs(FULL_CIRCLE_DEG - *cog) < 1 / NMEA_COG_FRAC_DIVISOR) {
    *cog = 0;
  }

  *sog_knots = MS2KNOTS(vel_north_ms, vel_east_ms, 0);
  *sog_kph = MS2KMHR(vel_north_ms, vel_east_ms, 0);
}

/** Assemble an NMEA GPRMC message and send it out NMEA USARTs.
 * NMEA RMC contains Recommended Minimum Specific GNSS Data.
 *
 * \param sbp_pos_llh  pointer to sbp pos llh struct
 * \param sbp_vel_ned  pointer to sbp vel ned struct
 * \param sbp_msg_time Pointer to sbp gps time struct
 * \param utc_time     Pointer to UTC time
 */
void nmea_gprmc(const msg_pos_llh_t *sbp_pos_llh,
                const msg_vel_ned_t *sbp_vel_ned,
                const msg_gps_time_t *sbp_msg_time,
                const utc_tm *utc_time) {
  /* See the relevant comment for the similar code in nmea_gpgga() function
     for the reasoning behind (... * 1e8 / 1e8) trick */
  double lat = fabs(round(sbp_pos_llh->lat * 1e8) / 1e8);
  double lon = fabs(round(sbp_pos_llh->lon * 1e8) / 1e8);

  char lat_dir = sbp_pos_llh->lat < 0.0 ? 'S' : 'N';
  assert(lat <= UINT16_MAX);
  u16 lat_deg = (u16)lat; /* truncation towards zero */
  double lat_min = (lat - (double)lat_deg) * 60.0;

  char lon_dir = sbp_pos_llh->lon < 0.0 ? 'W' : 'E';
  assert(lon <= UINT16_MAX);
  u16 lon_deg = (u16)lon; /* truncation towards zero */
  double lon_min = (lon - (double)lon_deg) * 60.0;

  char mode = get_nmea_mode_indicator(sbp_pos_llh->flags);
  char status = get_nmea_status(sbp_pos_llh->flags);

  double cog, sog_knots, sog_kph;
  calc_cog_sog(sbp_vel_ned, &cog, &sog_knots, &sog_kph);

  NMEA_SENTENCE_START(140);
  NMEA_SENTENCE_PRINTF("$GPRMC,"); /* Command */

  char utc[NMEA_TS_MAX_LEN];
  get_utc_time_string(
      sbp_msg_time, true, false, false, utc_time, utc, NMEA_TS_MAX_LEN);
  NMEA_SENTENCE_PRINTF("%s", utc);

  NMEA_SENTENCE_PRINTF("%c,", /* Status */
                       status);

  if ((sbp_pos_llh->flags & POSITION_MODE_MASK) != NO_POSITION) {
    NMEA_SENTENCE_PRINTF("%02u%010.7f,%c,%03u%010.7f,%c,", /* Lat/Lon */
                         lat_deg,
                         lat_min,
                         lat_dir,
                         lon_deg,
                         lon_min,
                         lon_dir);
  } else {
    NMEA_SENTENCE_PRINTF(",,,,"); /* Lat/Lon */
  }

  if ((sbp_pos_llh->flags & VELOCITY_MODE_MASK) != NO_VELOCITY) {
    NMEA_SENTENCE_PRINTF("%.2f,", sog_knots); /* Speed */
    if (NMEA_COG_STATIC_LIMIT_KNOTS < sog_knots) {
      NMEA_SENTENCE_PRINTF("%.*f,", NMEA_COG_DECIMALS, cog); /* Course */
    } else {
      NMEA_SENTENCE_PRINTF(","); /* Course */
    }
  } else {
    NMEA_SENTENCE_PRINTF(",,"); /* Speed, Course */
  }

  char date[NMEA_TS_MAX_LEN];
  get_utc_time_string(
      sbp_msg_time, false, true, true, utc_time, date, NMEA_TS_MAX_LEN);
  NMEA_SENTENCE_PRINTF("%s", date);

  NMEA_SENTENCE_PRINTF(
      ",,"  /* Magnetic Variation */
      "%c", /* Mode Indicator */
      mode);
  NMEA_SENTENCE_DONE();
}

/** Assemble an NMEA GPVTG message and send it out NMEA USARTs.
 * NMEA VTG contains Course Over Ground & Ground Speed.
 *
 * \param sbp_vel_ned Pointer to sbp vel ned struct.
 */
void nmea_gpvtg(const msg_vel_ned_t *sbp_vel_ned,
                const msg_pos_llh_t *sbp_pos_llh) {
  double cog, sog_knots, sog_kph;
  calc_cog_sog(sbp_vel_ned, &cog, &sog_knots, &sog_kph);

  /* Position indicator is used based upon spec
     "Positioning system mode indicator" means we should
     see the same mode for pos and velocity messages
     in a particular epoch */

  char mode = get_nmea_mode_indicator(sbp_pos_llh->flags);

  NMEA_SENTENCE_START(120);
  NMEA_SENTENCE_PRINTF("$GPVTG,"); /* Command */

  bool is_moving = (sbp_pos_llh->flags & VELOCITY_MODE_MASK) != NO_VELOCITY;

  if (is_moving && NMEA_COG_STATIC_LIMIT_KNOTS < sog_knots) {
    NMEA_SENTENCE_PRINTF("%.*f,T,", NMEA_COG_DECIMALS, cog); /* Course */
  } else {
    NMEA_SENTENCE_PRINTF(",T,"); /* Course */
  }

  NMEA_SENTENCE_PRINTF(",M,"); /* Magnetic Course (omitted) */

  if (is_moving) {
    /* Speed (knots, km/hr) */
    NMEA_SENTENCE_PRINTF("%.2f,N,%.2f,K,", sog_knots, sog_kph);
  } else {
    /* Speed (knots, km/hr) */
    NMEA_SENTENCE_PRINTF(",N,,K,");
  }

  /* Mode (note this is position mode not velocity mode)*/
  NMEA_SENTENCE_PRINTF("%c", mode);
  NMEA_SENTENCE_DONE();
}

/** Assemble an NMEA GPHDT message and send it out NMEA USARTs.
 * NMEA HDT contains Heading.
 *
 */
void nmea_gphdt(const msg_baseline_heading_t *sbp_baseline_heading) {
  NMEA_SENTENCE_START(40);
  NMEA_SENTENCE_PRINTF("$GPHDT,"); /* Command */
  if ((POSITION_MODE_MASK & sbp_baseline_heading->flags) == FIXED_POSITION) {
    NMEA_SENTENCE_PRINTF(
        "%.1f,T",
        (float)sbp_baseline_heading->heading /
            MSG_HEADING_SCALE_FACTOR); /* Heading only valid when fixed */
  } else {
    NMEA_SENTENCE_PRINTF(",T"); /* Heading only valid when fixed */
  }
  NMEA_SENTENCE_DONE();
}

/** Assemble an NMEA GPGLL message and send it out NMEA USARTs.
 * NMEA GLL contains Geographic Position Latitude/Longitude.
 *
 * \param sbp_pos_llh  Pointer to sbp pos llh struct.
 * \param sbp_msg_time Pointer to sbp gps time struct.
 * \param utc_time     Pointer to UTC time.
 */
void nmea_gpgll(const msg_pos_llh_t *sbp_pos_llh,
                const msg_gps_time_t *sbp_msg_time,
                const utc_tm *utc_time) {
  /* See the relevant comment for the similar code in nmea_gpgga() function
     for the reasoning behind (... * 1e8 / 1e8) trick */
  double lat = fabs(round(sbp_pos_llh->lat * 1e8) / 1e8);
  double lon = fabs(round(sbp_pos_llh->lon * 1e8) / 1e8);

  char lat_dir = sbp_pos_llh->lat < 0.0 ? 'S' : 'N';
  assert(lat <= UINT16_MAX);
  u16 lat_deg = (u16)lat; /* truncation towards zero */
  double lat_min = (lat - (double)lat_deg) * 60.0;

  char lon_dir = sbp_pos_llh->lon < 0.0 ? 'W' : 'E';
  assert(lon <= UINT16_MAX);
  u16 lon_deg = (u16)lon; /* truncation towards zero */
  double lon_min = (lon - (double)lon_deg) * 60.0;

  char status = get_nmea_status(sbp_pos_llh->flags);
  char mode = get_nmea_mode_indicator(sbp_pos_llh->flags);

  NMEA_SENTENCE_START(120);
  NMEA_SENTENCE_PRINTF("$GPGLL,"); /* Command */

  if ((sbp_pos_llh->flags & POSITION_MODE_MASK) != NO_POSITION) {
    NMEA_SENTENCE_PRINTF("%02u%010.7f,%c,%03u%010.7f,%c,", /* Lat/Lon */
                         lat_deg,
                         lat_min,
                         lat_dir,
                         lon_deg,
                         lon_min,
                         lon_dir);
  } else {
    NMEA_SENTENCE_PRINTF(",,,,"); /* Lat/Lon */
  }

  char utc[NMEA_TS_MAX_LEN];
  get_utc_time_string(
      sbp_msg_time, true, false, false, utc_time, utc, NMEA_TS_MAX_LEN);
  NMEA_SENTENCE_PRINTF("%s", utc);

  NMEA_SENTENCE_PRINTF("%c,%c", /* Status, Mode */
                       status,
                       mode);
  NMEA_SENTENCE_DONE();
}

/** Assemble an NMEA GPZDA message and send it out NMEA USARTs.
 * NMEA ZDA contains UTC Time and Date.
 *
 * \param sbp_msg_time Pointer to the current SBP GPS Time.
 * \param utc_time     Pointer to UTC time
 */
void nmea_gpzda(const msg_gps_time_t *sbp_msg_time, const utc_tm *utc_time) {
  NMEA_SENTENCE_START(40);
  NMEA_SENTENCE_PRINTF("$GPZDA,"); /* Command */

  char utc[NMEA_TS_MAX_LEN];
  get_utc_time_string(
      sbp_msg_time, true, true, false, utc_time, utc, NMEA_TS_MAX_LEN);
  NMEA_SENTENCE_PRINTF("%s", utc);

  NMEA_SENTENCE_PRINTF(","); /* Time zone */
  NMEA_SENTENCE_DONE();

} /* nmea_gpzda() */

static bool in_set(u8 prns[], u8 count, u8 prn) {
  for (u8 i = 0; i < count; i++) {
    if (prns[i] == prn) {
      return true;
    }
  }

  return false;
}

static void nmea_assemble_gsa(const msg_pos_llh_t *sbp_pos_llh,
                              const msg_dops_t *sbp_dops,
                              const u8 n_meas,
                              const navigation_measurement_t nav_meas[]) {
  assert(sbp_pos_llh);
  assert(sbp_dops);
  assert(nav_meas);

  u8 prns_gps[GSA_MAX_SV];
  u8 num_prns_gps = 0;
  u8 prns_glo[GSA_MAX_SV];
  u8 num_prns_glo = 0;

  bool fix = (NO_POSITION != (sbp_pos_llh->flags & POSITION_MODE_MASK));

  if (!fix) {
    /* No fix, no active SVs, send GSA messages and return */
    nmea_gsa(prns_gps, num_prns_gps, fix, sbp_dops, CONSTELLATION_GPS);
    nmea_gsa(prns_glo, num_prns_glo, fix, sbp_dops, CONSTELLATION_GLO);
    return;
  }

  /* Assemble list of currently active SVs */
  for (u8 i = 0; i < n_meas; i++) {
    const navigation_measurement_t info = nav_meas[i];
    if (IS_GPS(info.sid) && num_prns_gps < GSA_MAX_SV &&
        !in_set(prns_gps, num_prns_gps, info.sid.sat)) {
      prns_gps[num_prns_gps++] = info.sid.sat;
      continue;
    }

    if (enable_glonass && IS_GLO(info.sid) && num_prns_glo < GSA_MAX_SV &&
        !in_set(prns_glo, num_prns_glo, NMEA_SV_ID_GLO(info.sid.sat))) {
      prns_glo[num_prns_glo++] = NMEA_SV_ID_GLO(info.sid.sat);
      continue;
    }
  }

  /* Send GSA messages */
  nmea_gsa(prns_gps, num_prns_gps, fix, sbp_dops, CONSTELLATION_GPS);
  nmea_gsa(prns_glo, num_prns_glo, fix, sbp_dops, CONSTELLATION_GLO);
}

/** Generate and send periodic GPGSV and GLGSV.
 *
 * \param[in] n_used      size of ch_meas
 * \param[in] ch_meas     array of channel_measurement structs from tracked SVs
 */
void nmea_send_gsv(u8 n_used, const channel_measurement_t *ch_meas) {
  DO_EVERY(gpgsv_msg_rate, nmea_gsv(n_used, ch_meas, CONSTELLATION_GPS);
           nmea_gsv(n_used, ch_meas, CONSTELLATION_GLO););
}

bool send_nmea(u32 rate, u32 gps_tow_ms) {
  if (rate == 0 ) {
    return false;
  }
  if (gps_tow_ms == 0 ) {
    return false;
  }

  /* If the modulu of latest gps time estimate time with configured
   * output period is less than 1/2 the solution period we should send the NMEA message.
   * This way, we still send no_fix messages when receiver clock is drifting. */
  u32 soln_period_ms = (u32) 1/soln_freq_setting * 1e3;
  u32 output_period_ms = (u32) soln_period_ms * rate;
  if ((gps_tow_ms % output_period_ms) < (soln_period_ms / 2)) {
    return true;
  }
  return false;
}

/** Generate and send periodic NMEA GPRMC, GPGLL, GPVTG, GPZDA and GPGSA.
 * (but not GPGGA) messages.
 *
 * Called from solution thread.
 *
 * \param sbp_pos_llh  Pointer to sbp pos llh struct.
 * \param sbp_vel_ned  Pointer to sbp vel ned.
 * \param sbp_dops     Pointer to sbp dops.
 * \param sbp_msg_time Pointer to sbp msg time.
 * \param propagation_time time of base observation propagation
 * \param sender_id    NMEA sender id
 * \param utc_params   Pointer to UTC parameters
 * \param n_meas       nav_meas len
 * \param nav_meas     Valid navigation measurements
 */
void nmea_send_msgs(const msg_pos_llh_t *sbp_pos_llh,
                    const msg_vel_ned_t *sbp_vel_ned,
                    const msg_dops_t *sbp_dops,
                    const msg_gps_time_t *sbp_msg_time,
                    double propagation_time,
                    u8 sender_id,
                    const utc_params_t *utc_params,
                    const msg_baseline_heading_t *sbp_baseline_heading,
                    u8 n_meas,
                    const navigation_measurement_t nav_meas[]) {
  utc_tm utc_time;
  piksi_systime_t piksi_system_time;
  u64 piksi_time_ms;
  piksi_systime_get(&piksi_system_time);
  /* piksi_time_ms will be used to decimate output on time boundaries */
  piksi_time_ms = piksi_systime_to_ms(&piksi_system_time); 
  log_info("system time is %u", piksi_time_ms);
  /* prepare utc_tm structure with time rounded to NMEA precision 
   * Even when we have no GNSS fix, the tow is still populated
   * TOW is 0 before our first fix.*/
  if (sbp_msg_time->tow != 0 ) {
    gps_time_t t = {
        .wn = sbp_msg_time->wn,
        .tow = 1e-3 * sbp_msg_time->tow + 1e-9 * sbp_msg_time->ns_residual};
    piksi_time_ms = sbp_msg_time->tow;
    gps2utc(&t, &utc_time, utc_params);
    u16 second_frac = roundf(utc_time.second_frac * NMEA_UTC_S_FRAC_DIVISOR);
    if (second_frac == NMEA_UTC_S_FRAC_DIVISOR) {
      /* the UTC timestamp rounds up to next second, so recompute the UTC time
       * structure to normalize it and to handle leap second correctly */
      double dt = 1.0 - utc_time.second_frac;
      /* round up to the next representable floating point number */
      t.tow = nextafter(t.tow + dt, INFINITY);
      normalize_gps_time(&t);
      gps2utc(&t, &utc_time, utc_params);
      /* if the fractional part of the resulting new time stamp is not zero
       * then we computed wrong... */
      second_frac = roundf(utc_time.second_frac * NMEA_UTC_S_FRAC_DIVISOR);
      assert(second_frac == 0);
    }
  }
  if (sbp_pos_llh && sbp_msg_time && sbp_dops) {
    if (send_nmea(gpgga_msg_rate, piksi_time_ms)) {
      nmea_gpgga(sbp_pos_llh,
                 sbp_msg_time,
                 &utc_time,
                 sbp_dops,
                 propagation_time,
                 sender_id);
    }
  }
  if (sbp_baseline_heading && send_heading && sbp_msg_time) {
    if (send_nmea(gphdt_msg_rate, piksi_time_ms)) {
      nmea_gphdt(sbp_baseline_heading);
    }
  }
  if (sbp_vel_ned && sbp_pos_llh && sbp_msg_time) {
    if (send_nmea(gprmc_msg_rate, piksi_time_ms)) {
      nmea_gprmc(sbp_pos_llh, sbp_vel_ned, sbp_msg_time, &utc_time);
    }
  }
  if (sbp_pos_llh && sbp_msg_time) {
    if (send_nmea(gpgll_msg_rate, piksi_time_ms)) {
      nmea_gpgll(sbp_pos_llh, sbp_msg_time, &utc_time);
    }
  }
  if (sbp_vel_ned && sbp_pos_llh && sbp_msg_time) {
    if (send_nmea(gpvtg_msg_rate, piksi_time_ms)) {
      nmea_gpvtg(sbp_vel_ned, sbp_pos_llh);
    }
  }
  if (sbp_msg_time) {
    if (send_nmea(gpzda_msg_rate, piksi_time_ms)) {
      nmea_gpzda(sbp_msg_time, &utc_time);
    }
  }
  if (sbp_dops && sbp_pos_llh && sbp_msg_time) {
    if (send_nmea(gsa_msg_rate, piksi_time_ms)) {
      nmea_assemble_gsa(sbp_pos_llh, sbp_dops, n_meas, nav_meas);
    }
  }
}

/** Convert the SBP status flag into NMEA Status field.
 * Ref: NMEA-0183 version 2.30 pp.42,43
 *
 * \param flags        u8 sbp_pos_llh->flags
 */
char get_nmea_status(u8 flags) {
  switch (flags & POSITION_MODE_MASK) {
    case NO_POSITION:
      return 'V';
    case SPP_POSITION: /* autonomous mode */
    case DGNSS_POSITION:
    case FLOAT_POSITION:
    case FIXED_POSITION:
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
    case NO_POSITION:
      return 'N';
    case SPP_POSITION: /* autonomous mode */
      return 'A';
    case DGNSS_POSITION: /* differential mode */
    case FLOAT_POSITION:
    case FIXED_POSITION:
      return 'D';
    default:
      assert(!"Unsupported position type indicator");
      return 'N';
  }
}

/** Convert the SBP status flag into NMEA Quality Indicator field:
 * Ref: NMEA-0183 version 2.30 pp.30
 *
 * \param flags        u8 sbp_pos_llh->flags
 */
u8 get_nmea_quality_indicator(u8 flags) {
  switch (flags & POSITION_MODE_MASK) {
    case NO_POSITION:
      return NMEA_GGA_QI_INVALID;
    case SPP_POSITION:
      return NMEA_GGA_QI_GPS;
    case DGNSS_POSITION:
      return NMEA_GGA_QI_DGPS;
    case FLOAT_POSITION:
      return NMEA_GGA_QI_FLOAT;
    case FIXED_POSITION:
      return NMEA_GGA_QI_RTK;
    default:
      assert(!"Unsupported position type indicator");
      return NMEA_GGA_QI_INVALID;
  }
}

/** \} */

/** \} */
