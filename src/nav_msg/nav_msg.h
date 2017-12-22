/*
 * Copyright (C) 2010, 2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_NAV_MSG_H
#define SWIFTNAV_NAV_MSG_H

#include <libswiftnav/almanac.h>
#include <libswiftnav/common.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/ionosphere.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** GPS LNAV decode buffer size (448 bits) [32-bit words] */
#define NAV_MSG_SUBFRAME_WORDS_LEN 14
/** GPS LNAV decode buffer size (448 bits) [bits] */
#define NAV_MSG_SUBFRAME_BITS_LEN (NAV_MSG_SUBFRAME_WORDS_LEN * 32)

#define TOW_INVALID -1

#define BIT_POLARITY_NORMAL 0
#define BIT_POLARITY_INVERTED 1
#define BIT_POLARITY_UNKNOWN -1

/** Minimum GPS LNAV valid subframe number */
#define GPS_LNAV_SUBFRAME_MIN 1
/** Maximum GPS LNAV valid subframe number */
#define GPS_LNAV_SUBFRAME_MAX 5
#define GPS_LNAV_SUBFRAME_CNT \
  (GPS_LNAV_SUBFRAME_MAX - GPS_LNAV_SUBFRAME_MIN + 1)
/** Number of words we want to store */
#define GPS_LNAV_WORD_STORE_CNT 8

/** Special value to marking maximum subframe cache entry age [6 seconds] */
#define GPS_LNAV_SUBFRAME_AGE_INVALID ((u8)-1)
/** Special value to marking maximum subframe cache entry age [6 seconds] */
#define GPS_LNAV_SUBFRAME_AGE_MAX ((u8)100)

/**
 * GPS LNAV message decoder object.
 *
 * The object for decoding GPS LNAV messages for PRNs 1-32.
 *
 * \sa nav_msg_init
 * \sa nav_msg_update
 * \sa subframe_ready
 * \sa process_subframe
 */
typedef struct {
  /**< Decoder buffer (448 bits) */
  u32 subframe_bits[NAV_MSG_SUBFRAME_WORDS_LEN];
  /**< Current bit index in the buffer */
  u16 subframe_bit_index;
  /**< Buffer overrun flag (error) */
  bool overrun;
  /** subframe_start_index:
   * - 0 = no preamble found
   * - +x = preamble begins at bit index (x-1)
   * - -x = inverse preamble begins at (1-x)
   */
  s16 subframe_start_index;
  /**< Decoded subframe data */
  u32 frame_words[GPS_LNAV_SUBFRAME_CNT][GPS_LNAV_WORD_STORE_CNT];
  u8 frame_age[GPS_LNAV_SUBFRAME_CNT]; /**< Decoded subframe data age [6s]*/
  u8 next_subframe_id;                 /**< Next expected subframe id */
  s8 bit_polarity;                     /**< Bit polarity on decoding */
  u8 alert;                            /**< Alert flag */
} nav_msg_t;

typedef struct {
  ephemeris_t ephemeris;
  bool ephemeris_upd_flag;

  ionosphere_t iono;
  bool iono_corr_upd_flag;

  utc_params_t utc;
  bool utc_params_upd_flag;

  u32 gps_l2c_sv_capability;
  bool gps_l2c_sv_capability_upd_flag;

  u8 shi1;
  bool shi1_upd_flag;

  almanac_t almanac;
  bool almanac_upd_flag;

  gps_time_t almanac_time;
  bool almanac_time_upd_flag;

  u8 almanac_health[32];
  u32 almanac_health_upd_flags;

  bool invalid_control_or_data;
} gps_l1ca_decoded_data_t;

void nav_msg_init(nav_msg_t *n);
void nav_msg_clear_decoded(nav_msg_t *n);
s32 nav_msg_update(nav_msg_t *n, bool bit_val);
bool subframe_ready(nav_msg_t *n);
s8 process_subframe(nav_msg_t *n,
                    const me_gnss_signal_t mesid,
                    gps_l1ca_decoded_data_t *data);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* SWIFTNAV_NAV_MSG_H */
