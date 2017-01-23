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

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <libswiftnav/coord_system.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/time.h>
#include <libswiftnav/observation.h>

#include "board/nap/track_channel.h"
#include "track.h"
#include "nmea.h"
#include "sbp.h"
#include "settings.h"
#include "main.h"
#include "timing.h"
#include "io_support.h"

static u32 gpgsv_msg_rate = 10;
static u32 gprmc_msg_rate = 10;
static u32 gpvtg_msg_rate =  1;
static u32 gpgll_msg_rate = 10;
static u32 gpzda_msg_rate = 10;
static u32 gpgsa_msg_rate = 10;

/** \addtogroup io
 * \{ */

/** \defgroup nmea NMEA
 * Send messages in NMEA format.
 * \{ */

#define NMEA_SUFFIX_LEN 6  /* How much room to leave for the NMEA
                              checksum, CRLF + null termination,
                              i.e. "*%02X\r\n\0" */

/** Some helper macros for functions generating NMEA sentences. */

/** NMEA_SENTENCE_START: declare a buffer and set up some pointers
 * max_len = max possible length of the body of the message
 * (not including suffix)
 */
#define NMEA_SENTENCE_START(max_len) \
  char sentence_buf[max_len + NMEA_SUFFIX_LEN]; \
  char *sentence_bufp = sentence_buf; \
  char * const sentence_buf_end = sentence_buf + max_len;

/** NMEA_SENTENCE_PRINTF: use like printf, can use multiple times
    within a sentence. */
#define NMEA_SENTENCE_PRINTF(fmt, ...) do { \
    sentence_bufp += snprintf(sentence_bufp, sentence_buf_end - sentence_bufp, fmt, ##__VA_ARGS__); \
    if (sentence_bufp >= sentence_buf_end) \
      sentence_bufp = sentence_buf_end; \
    } while (0)

/** NMEA_SENTENCE_DONE: append checksum and dispatch.
 * \note According to section 5.3.1 of the NMEA 0183 spec, sentences are
 *       terminated with <CR><LF>. The sentence_buf is null_terminated.
 *       The call to nmea_output has been modified to remove the NULL.
 *       This will also affect all registered dispatchers
 */
#define NMEA_SENTENCE_DONE() do { \
    if (sentence_bufp == sentence_buf_end) \
      log_warn("NMEA %.6s cut off", sentence_buf); \
    nmea_append_checksum(sentence_buf, sizeof(sentence_buf)); \
    nmea_output(sentence_buf, sentence_bufp - sentence_buf + NMEA_SUFFIX_LEN-1); \
  } while (0)

/** Output NMEA sentence.

 * \param s The NMEA sentence to output.
 * \param size This is the C-string size, not including the null character
 */
static void nmea_output(char *s, size_t size)
{
  static MUTEX_DECL(send_mutex);
  chMtxLock(&send_mutex);

  io_support_write(SD_NMEA, (u8 *)s, size);

  chMtxUnlock(&send_mutex);
}

void nmea_setup(void)
{
  SETTING("nmea", "gpgsv_msg_rate", gpgsv_msg_rate, TYPE_INT);
  SETTING("nmea", "gprmc_msg_rate", gprmc_msg_rate, TYPE_INT);
  SETTING("nmea", "gpvtg_msg_rate", gpvtg_msg_rate, TYPE_INT);
  SETTING("nmea", "gpgll_msg_rate", gpgll_msg_rate, TYPE_INT);
  SETTING("nmea", "gpzda_msg_rate", gpzda_msg_rate, TYPE_INT);
  SETTING("nmea", "gpgsa_msg_rate", gpgsa_msg_rate, TYPE_INT);
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
static void nmea_append_checksum(char *s, size_t size)
{
  u8 sum = 0;
  char *p = s;

  /* '$' header not included in checksum calculation */
  if (*p == '$') {
    p++;
  }

  /* '*'  not included in checksum calculation */
  while (*p != '*' &&
         *p &&
         p + NMEA_SUFFIX_LEN < s + size) {
    sum ^= *p;
    p++;
  }

  sprintf(p, "*%02X\r\n", sum);
}

/** Assemble a NMEA GPGGA message and send it out NMEA USARTs.
 * NMEA GPGGA message contains Global Positioning System Fix Data.
 *
 * \param sbp_pos_llh       SBP LLH position messages
 * \param sbp_msg_time      SBP GPS Time message
 * \param sbp_dops          SBP DOP Message for this epoch
 * \param propagation_time  Age of differential corrections [s].
 * \param station_id        Differential reference station ID.
 */
void nmea_gpgga(const msg_pos_llh_t *sbp_pos_llh,
                const msg_gps_time_t *sbp_msg_time,
                const msg_dops_t *sbp_dops, double propagation_time,
                u8 station_id)
{
  time_t unix_t;
  struct tm t;

  gps_time_t current_time;

  if (sbp_msg_time->flags > 0) {
    current_time.wn = sbp_msg_time->wn;
    /* SBP msg time is in milliseconds */
    current_time.tow = sbp_msg_time->tow * 1e-3;

    unix_t = gps2time(&current_time);
    gmtime_r(&unix_t, &t);
  }

  double frac_s = fmod(current_time.tow, 1.0);

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
  u16 lat_deg = (u16) lat; /* truncation towards zero */
  double lat_min = (lat - (double) lat_deg) * 60.0;

  char lon_dir = sbp_pos_llh->lon < 0.0 ? 'W' : 'E';
  assert(lon <= UINT16_MAX);
  u16 lon_deg = (u16) lon; /* truncation towards zero */
  double lon_min = (lon - (double) lon_deg) * 60.0;

  u8 fix_type = 0;
  switch (sbp_pos_llh->flags) {
    case 0: fix_type = NMEA_GGA_FIX_INVALID; break;
    case 1: fix_type = NMEA_GGA_FIX_GPS; break;
    case 2: fix_type = NMEA_GGA_FIX_DGPS; break;
    case 3: fix_type = NMEA_GGA_FIX_FLOAT; break;
    case 4: fix_type = NMEA_GGA_FIX_RTK; break;
    default: fix_type = NMEA_GGA_FIX_INVALID;
  }

  NMEA_SENTENCE_START(120);
  NMEA_SENTENCE_PRINTF("$GPGGA,");

  if (sbp_msg_time->flags > 0) {
    NMEA_SENTENCE_PRINTF("%02d%02d%06.3f,",
                         t.tm_hour, t.tm_min, t.tm_sec + frac_s);
  } else {
    NMEA_SENTENCE_PRINTF(",");
  }

  if (fix_type != NMEA_GGA_FIX_INVALID) {
    NMEA_SENTENCE_PRINTF("%02u%010.7f,%c,%03u%010.7f,%c,",
                         lat_deg, lat_min, lat_dir, lon_deg, lon_min, lon_dir);
  } else {
    NMEA_SENTENCE_PRINTF(",,,,");
  }

  NMEA_SENTENCE_PRINTF("%01d,", fix_type);

  if (fix_type != NMEA_GGA_FIX_INVALID) {
    NMEA_SENTENCE_PRINTF("%02d,%.1f,%.2f,M,0.0,M,",
                         sbp_pos_llh->n_sats,
                         sbp_dops->hdop * 0.01,
                         sbp_pos_llh->height);
  } else {
    NMEA_SENTENCE_PRINTF(",,,M,,M,");
  }

  if ((fix_type == NMEA_GGA_FIX_DGPS) ||
      (fix_type == NMEA_GGA_FIX_FLOAT) ||
      (fix_type == NMEA_GGA_FIX_RTK)) {
    NMEA_SENTENCE_PRINTF("%.1f,%04d",
                         propagation_time,
                         station_id & 0x3FF); /* ID range is 0000 to 1023 */
  } else {
    NMEA_SENTENCE_PRINTF(",");
  }

  NMEA_SENTENCE_DONE();
}

/** Assemble a NMEA GPGSA message and send it out NMEA USARTs.
 * NMEA GPGSA message contains GNSS DOP and Active Satellites.
 *
 * \param prns      Array of PRNs to output.
 * \param num_prns  Number of valid PRNs in array.
 * \param sbp_dops  Pointer to SBP MSG DOP struct (PDOP, HDOP, VDOP).
 */
void nmea_gpgsa(const u8 *prns, u8 num_prns, const msg_pos_llh_t *sbp_pos_llh, const msg_dops_t *sbp_dops)
{
  char fix_mode =
    (sbp_pos_llh->flags == 0) ? '1' : '3';           /* Our fix is allways 3D */

  NMEA_SENTENCE_START(120);
  NMEA_SENTENCE_PRINTF("$GPGSA,A,%c,", fix_mode);    /* Always automatic mode */

  for (u8 i = 0; i < 12; i++) {
    if ((sbp_pos_llh->flags > 0) && (i < num_prns)) {
      NMEA_SENTENCE_PRINTF("%02d,", prns[i]);
    } else {
      NMEA_SENTENCE_PRINTF(",");
    }
  }

  if ((sbp_pos_llh->flags > 0) && sbp_dops) {
    NMEA_SENTENCE_PRINTF("%.1f,%.1f,%.1f",
                         sbp_dops->pdop * 0.01,
                         sbp_dops->hdop * 0.01,
                         sbp_dops->vdop * 0.01);
  } else {
    NMEA_SENTENCE_PRINTF(",,");
  }

  NMEA_SENTENCE_DONE();
}

/** Assemble a NMEA GPGSV message and send it out NMEA USARTs.
 * NMEA GPGSV message contains GNSS Satellites In View.
 *
 * \param nav_meas     Array of navigation_measurement structs.
 * \param sbp_pos_ecef Pointer to sbp pos ecef struct.
 */
void nmea_gpgsv(u8 n_used,
                const navigation_measurement_t *nav_meas,
                const msg_pos_ecef_t *sbp_pos_ecef)
{
  if (n_used == 0) {
    return;
  }

  u8 n_l1_used = 0;
  for (u8 i = 0; i < n_used; i++) {
    if (nav_meas[i].sid.code == CODE_GPS_L1CA) {
      n_l1_used++;
    }
  }

  u8 n_messages = (n_l1_used + 3) / 4;

  u8 n = 0;
  double az, el;

  for (u8 i = 0; i < n_messages; i++) {
    NMEA_SENTENCE_START(120);
    NMEA_SENTENCE_PRINTF("$GPGSV,%u,%u,%02u", n_messages, i+1, n_l1_used);

    for (u8 j = 0; j < 4 && n < n_used; n++) {
      if (nav_meas[n].sid.code == CODE_GPS_L1CA) {
        double pos_ecef[3];
        pos_ecef[0] = sbp_pos_ecef->x;
        pos_ecef[1] = sbp_pos_ecef->y;
        pos_ecef[2] = sbp_pos_ecef->z;
        wgsecef2azel(nav_meas[n].sat_pos, pos_ecef, &az, &el);
        NMEA_SENTENCE_PRINTF(",%02u,%02u,%03u,%02u",
          nav_meas[n].sid.sat,
          (u8)round(el * R2D),
          (u16)round(az * R2D),
          (u8)round(nav_meas[n].cn0)
          );
        j++; /* 4 sats per message no matter what */
      }
    }
    NMEA_SENTENCE_DONE();
  }

}

/** Assemble an NMEA GPRMC message and send it out NMEA USARTs.
 * NMEA RMC contains Recommended Minimum Specific GNSS Data.
 *
 * \param sbp_pos_llh  pointer to sbp pos llh struct
 * \param sbp_vel_ned  pointer to sbp vel ned struct
 * \param sbp_msg_time Pointer to sbp gps time struct
 */
void nmea_gprmc(const msg_pos_llh_t *sbp_pos_llh, const msg_vel_ned_t *sbp_vel_ned,
                const msg_gps_time_t *sbp_msg_time)
{
  time_t unix_t;
  struct tm t;

  gps_time_t current_time;
  if (sbp_msg_time->flags > 0) {
    current_time.wn = sbp_msg_time->wn;
    /* SBP msg time is in milliseconds */
    current_time.tow = sbp_msg_time->tow * 1e-3;

    unix_t = gps2time(&current_time);
    gmtime_r(&unix_t, &t);
  }

  double frac_s  = fmod(current_time.tow, 1.0);

  /* See the relevant comment for the similar code in nmea_gpgga() function
     for the reasoning behind (... * 1e8 / 1e8) trick */
  double lat     = fabs(round(sbp_pos_llh->lat * 1e8) / 1e8);
  double lon     = fabs(round(sbp_pos_llh->lon * 1e8) / 1e8);

  char   lat_dir = sbp_pos_llh->lat < 0.0 ? 'S' : 'N';
  assert(lat <= UINT16_MAX);
  u16    lat_deg = (u16)lat; /* truncation towards zero */
  double lat_min = (lat - (double)lat_deg) * 60.0;

  char   lon_dir = sbp_pos_llh->lon < 0.0 ? 'W' : 'E';
  assert(lon <= UINT16_MAX);
  u16    lon_deg = (u16)lon; /* truncation towards zero */
  double lon_min = (lon - (double)lon_deg) * 60.0;

  char mode = get_nmea_mode_indicator(sbp_pos_llh->flags);
  char status = get_nmea_status(sbp_pos_llh->flags);

  double x,y,z;
  x = sbp_vel_ned->n / 1000.0;
  y = sbp_vel_ned->e / 1000.0;
  z = sbp_vel_ned->d / 1000.0;
  double course = R2D * atan2(y,x);
  if (course < 0.0) {
    course += 360.0;
  }
  /* Conversion to magnitue knots */
  double vknots = MS2KNOTS(x,y,z);

  NMEA_SENTENCE_START(140);
  NMEA_SENTENCE_PRINTF(
    "$GPRMC,"                                  /* Command */
  );

  if (sbp_msg_time->flags > 0) {
    NMEA_SENTENCE_PRINTF(
      "%02d%02d%06.3f,",                       /* Time (UTC) */
      t.tm_hour, t.tm_min, t.tm_sec + frac_s
    );
  } else {
    NMEA_SENTENCE_PRINTF(
      ","                                      /* Time (UTC) */
    );
  }

  NMEA_SENTENCE_PRINTF(
    "%c,",                                     /* Status */
    status
  );

  if (sbp_pos_llh->flags > 0) {
    NMEA_SENTENCE_PRINTF(
      "%02u%010.7f,%c,%03u%010.7f,%c,",        /* Lat/Lon */
      lat_deg, lat_min, lat_dir, lon_deg, lon_min, lon_dir
    );
  } else {
    NMEA_SENTENCE_PRINTF(
      ",,,,"                                   /* Lat/Lon */
    );
  }

  if (sbp_vel_ned->flags > 0) {
    NMEA_SENTENCE_PRINTF(
      "%.2f,%05.1f,",                          /* Speed, Course */
      vknots, course
    );
  } else {
    NMEA_SENTENCE_PRINTF(
      ",,"                                     /* Speed, Course */
    );
  }

  if (sbp_msg_time->flags > 0) {
    NMEA_SENTENCE_PRINTF(
      "%02d%02d%02d,",                         /* Date Stamp */
      t.tm_mday, t.tm_mon + 1, t.tm_year % 100
    );
  } else {
    NMEA_SENTENCE_PRINTF(
      ","                                      /* Date Stamp */
    );
  }

  NMEA_SENTENCE_PRINTF(
    ",,"                                       /* Magnetic Variation */
    "%c",                                      /* Mode Indicator */
    mode
  );
  NMEA_SENTENCE_DONE();
}

/** Assemble an NMEA GPVTG message and send it out NMEA USARTs.
 * NMEA VTG contains Course Over Ground & Ground Speed.
 *
 * \param sbp_vel_ned Pointer to sbp vel ned struct.
 */
void nmea_gpvtg(const msg_vel_ned_t *sbp_vel_ned, const msg_pos_llh_t *sbp_pos_llh)
{
  double x,y,z;
  x = sbp_vel_ned->n / 1000.0;
  y = sbp_vel_ned->e / 1000.0;
  z = sbp_vel_ned->d / 1000.0;

  double course = R2D * atan2(y,x);
  if (course < 0.0) {
    course += 360.0;
  }

  /* Conversion to magnitude knots */
  double vknots = MS2KNOTS(x,y,z);
  /* Conversion to magnitude km/hr */
  double vkmhr = MS2KMHR(x,y,z);
  /* Position indicator is used based upon spec
     "Positioning system mode indicator" means we should
     see the same mode for pos and velocity messages
     in a particular epoch */

  char mode = get_nmea_mode_indicator(sbp_pos_llh->flags);

  NMEA_SENTENCE_START(120);
  NMEA_SENTENCE_PRINTF(
    "$GPVTG,"                           /* Command */
  );

  if (sbp_vel_ned->flags > 0) {
    NMEA_SENTENCE_PRINTF(
      "%05.1f,T,",                      /* Course */
      course
    );
  } else {
    NMEA_SENTENCE_PRINTF(
      ",T,"                             /* Course */
    );
  }

  NMEA_SENTENCE_PRINTF(
    ",M,"                               /* Magnetic Course (omitted) */
  );

  if (sbp_vel_ned->flags > 0) {
    NMEA_SENTENCE_PRINTF(
      "%.2f,N,%.2f,K,",                 /* Speed (knots, km/hr) */
      vknots, vkmhr
    );
  } else {
    NMEA_SENTENCE_PRINTF(
      ",N,,K,"                          /* Speed (knots, km/hr) */
    );
  }

  NMEA_SENTENCE_PRINTF(
    "%c",                               /* Mode (note this position mode not
                                                 velocity mode)*/
    mode
  );
  NMEA_SENTENCE_DONE();
}

/** Assemble an NMEA GPGLL message and send it out NMEA USARTs.
 * NMEA GLL contains Geographic Position Latitude/Longitude.
 *
 * \param sbp_pos_llh  Pointer to sbp pos llh struct.
 * \param sbp_msg_time Pointer to sbp gps time struct.
 */
void nmea_gpgll(const msg_pos_llh_t *sbp_pos_llh, const msg_gps_time_t *sbp_msg_time)
{
  time_t unix_t;
  struct tm t;

  gps_time_t current_time;
  if (sbp_msg_time->flags > 0) {
    current_time.wn = sbp_msg_time->wn;
    /* SBP msg time is in milliseconds */
    current_time.tow = sbp_msg_time->tow * 1e-3;

    unix_t = gps2time(&current_time);
    gmtime_r(&unix_t, &t);
  }

  double frac_s  = fmod(current_time.tow, 1.0);

  /* See the relevant comment for the similar code in nmea_gpgga() function
     for the reasoning behind (... * 1e8 / 1e8) trick */
  double lat     = fabs(round(sbp_pos_llh->lat * 1e8) / 1e8);
  double lon     = fabs(round(sbp_pos_llh->lon * 1e8) / 1e8);

  char   lat_dir = sbp_pos_llh->lat < 0.0 ? 'S' : 'N';
  assert(lat <= UINT16_MAX);
  u16    lat_deg = (u16)lat; /* truncation towards zero */
  double lat_min = (lat - (double)lat_deg) * 60.0;

  char   lon_dir = sbp_pos_llh->lon < 0.0 ? 'W' : 'E';
  assert(lon <= UINT16_MAX);
  u16    lon_deg = (u16)lon; /* truncation towards zero */
  double lon_min = (lon - (double)lon_deg) * 60.0;

  char status = get_nmea_status(sbp_pos_llh->flags);
  char mode = get_nmea_mode_indicator(sbp_pos_llh->flags);

  NMEA_SENTENCE_START(120);
  NMEA_SENTENCE_PRINTF(
    "$GPGLL,"                                     /* Command */
  );

  if (sbp_pos_llh->flags > 0) {
    NMEA_SENTENCE_PRINTF(
      "%02u%010.7f,%c,%03u%010.7f,%c,",           /* Lat/Lon */
      lat_deg, lat_min, lat_dir, lon_deg, lon_min, lon_dir
    );
  } else {
    NMEA_SENTENCE_PRINTF(
      ",,,,"                                      /* Lat/Lon */
    );
  }

  if (sbp_msg_time->flags > 0) {
    NMEA_SENTENCE_PRINTF(
      "%02d%02d%06.3f,",                          /* Time (UTC) */
      t.tm_hour, t.tm_min, t.tm_sec + frac_s
    );
  } else {
    NMEA_SENTENCE_PRINTF(
      ","
    );
  }
  NMEA_SENTENCE_PRINTF(
    "%c,%c",                                      /* Status, Mode */
    status, mode
  );
  NMEA_SENTENCE_DONE();
}

/** Assemble an NMEA GPZDA message and send it out NMEA USARTs.
 * NMEA ZDA contains UTC Time and Date.
 *
 * \param sbp_msg_time Pointer to the current SBP GPS Time.
 */
void nmea_gpzda(const msg_gps_time_t *sbp_msg_time)
{
  time_t unix_t;
  struct tm t;

  gps_time_t current_time;
  if(sbp_msg_time->flags > 0) {
    current_time.wn = sbp_msg_time->wn;
    /* SBP msg time is in milliseconds */
    current_time.tow = sbp_msg_time->tow * 1e-3;

    unix_t = gps2time(&current_time);
    gmtime_r(&unix_t, &t);
  }

  double frac_s = fmod(current_time.tow, 1.0);

  NMEA_SENTENCE_START(40);
  NMEA_SENTENCE_PRINTF(
    "$GPZDA,"                              /* Command */
  );

  if (sbp_msg_time->flags > 0) {
    NMEA_SENTENCE_PRINTF(
      "%02d%02d%06.3f,"                    /* Time (UTC) */
      "%02d,%02d,%d,",                     /* Date Stamp */
      t.tm_hour, t.tm_min, t.tm_sec + frac_s,
      t.tm_mday, t.tm_mon + 1, 1900 + t.tm_year
    );
  } else {
    NMEA_SENTENCE_PRINTF(
      ","                                  /* Time (UTC) */
      ",,,"                                /* Date Stamp */
    );
  }

  NMEA_SENTENCE_PRINTF(
    ","                                    /* Time zone */
  );
  NMEA_SENTENCE_DONE();

} /* nmea_gpzda() */


static void nmea_assemble_gpgsa(const msg_pos_llh_t *sbp_pos_llh, const msg_dops_t *sbp_dops)
{
  /* Assemble list of currently tracked GPS PRNs */
  u8 prns[nap_track_n_channels];
  u8 num_prns = 0;
  for (u32 i = 0; i < nap_track_n_channels; i++) {
    tracking_channel_info_t info;
    tracking_channel_get_values(i,
                                &info,  /* Generic info */
                                NULL,   /* Timers */
                                NULL,   /* Frequencies */
                                NULL,   /* Loop controller values */
                                NULL,   /* Misc values */
                                false); /* Reset stats */

    if (0 != (info.flags & TRACKING_CHANNEL_FLAG_ACTIVE)) {
      gnss_signal_t sid = info.sid;
      if (sid.code == CODE_GPS_L1CA ) {
        prns[num_prns++] = sid.sat;
      }
    }
  }
  /* Send GPGSA message */
  nmea_gpgsa(prns, num_prns, sbp_pos_llh, sbp_dops);
}

/** Generate and send periodic NMEA GPRMC, GPGLL, GPVTG, GPZDA, GPGSA and GPGSV.
 * (but not GPGGA) messages.
 *
 * Called from solution thread.
 *
 * \param sbp_pos_llh  Pointer to sbp pos llh struct.
 * \param sbp_pos_ecef Pointer to sbp pos ecef.
 * \param sbp_vel_ned  Pointer to sbp vel ned.
 * \param sbp_dops     Pointer to sbp dops.
 * \param sbp_msg_time Pointer to sbp msg time.
 * \param nav_meas     Array of n navigation_measurement structs.
 * \param sender_id    NMEA sender id
 * \param propagation_time time of base observation propagation
 */
void nmea_send_msgs(const msg_pos_llh_t *sbp_pos_llh, const msg_pos_ecef_t *sbp_pos_ecef,
                    const msg_vel_ned_t *sbp_vel_ned, const msg_dops_t *sbp_dops,
                    const msg_gps_time_t *sbp_msg_time, u8 n_used, const navigation_measurement_t *nav_meas,
                    double propagation_time, u8 sender_id)
{
  if (sbp_pos_llh && sbp_msg_time && sbp_dops) {
    nmea_gpgga(sbp_pos_llh, sbp_msg_time, sbp_dops, propagation_time, sender_id);
  }

  if (sbp_vel_ned && sbp_pos_llh && sbp_msg_time) {
    DO_EVERY(gprmc_msg_rate,
      nmea_gprmc(sbp_pos_llh, sbp_vel_ned, sbp_msg_time);
    );
  }
  if(sbp_pos_llh && sbp_msg_time) {
    DO_EVERY(gpgll_msg_rate,
             nmea_gpgll(sbp_pos_llh, sbp_msg_time););
  }
  if (sbp_vel_ned && sbp_pos_llh) {
    DO_EVERY(gpvtg_msg_rate,
      nmea_gpvtg(sbp_vel_ned, sbp_pos_llh);
    );
  }
  if (sbp_msg_time) {
    DO_EVERY(gpzda_msg_rate,
             nmea_gpzda(sbp_msg_time);
    );
  }
  if (sbp_dops && sbp_pos_llh) {
    DO_EVERY(gpgsa_msg_rate,
             nmea_assemble_gpgsa(sbp_pos_llh, sbp_dops);
    );
  }
  if(nav_meas && sbp_pos_ecef) {
    DO_EVERY(gpgsv_msg_rate,
             nmea_gpgsv(n_used, nav_meas, sbp_pos_ecef);
    );
  };
}

/** Convert the sbp status flag into NMEA Status field.
 * Ref: NMEA-0183 version 2.30 pp.42,43
 *
 * \param flags        u8 sbp_pos_llh->flags
 */
char get_nmea_status(u8 flags)
{
  switch (flags) {
  case 0:
    return 'V';
  case SPP_POSITION:   /* autonomous mode */
  case DGNSS_POSITION:
  case FLOAT_POSITION:
  case FIXED_POSITION:
    return 'A';
  default:
    assert(!"Unsupported position type indicator");
    return 'V';
  }
}

/** Convert the sbp status flag into NMEA Mode Indicator field:
 * Ref: NMEA-0183 version 2.30 pp.42,43
 *
 * \param flags        u8 sbp_pos_llh->flags
 */
char get_nmea_mode_indicator(u8 flags)
{
  switch (flags) {
  case 0:
    return 'N';
  case SPP_POSITION:   /* autonomous mode */
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

/** \} */

/** \} */
