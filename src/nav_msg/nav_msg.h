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

#include <stdbool.h>
#include <swiftnav/almanac.h>
#include <swiftnav/common.h>
#include <swiftnav/ephemeris.h>
#include <swiftnav/ionosphere.h>

#include "me_constants.h"
#include "signal_db/signal_db.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** GPS LNAV decode buffer size (480 bits) [32-bit words] */
#define NAV_MSG_SUBFRAME_WORDS_LEN (15)
/** GPS LNAV decode buffer size (480 bits) [bits] */
#define NAV_MSG_SUBFRAME_BITS_LEN (NAV_MSG_SUBFRAME_WORDS_LEN * 32)

#define TOW_INVALID (-1)
#define BUFFER_OVERRUN (-2)
#define BIT_INDEX_INVALID (-22)

#define BIT_POLARITY_NORMAL (0)
#define BIT_POLARITY_INVERTED (1)
#define BIT_POLARITY_UNKNOWN (-1)

#define GPS_L1CA_PREAMBLE_NORMAL (0x8B)
#define GPS_L1CA_PREAMBLE_INVERTED (0x74)
#define GPS_L1CA_PREAMBLE_LENGTH_BITS (8)

/** Multiplier to convert GPS truncated TOW into TOW.
 *  TOW is in units of [1.5 seconds] => multiplier of 1.5
 *  Truncated tow ignores 2 LSBs => multiplier of 4
 *  => Total multiplier of 1.5 * 4 = 6. */
#define GPS_TOW_MULTIPLIER (6)

/** Number of bits that needs to be decoded for polarity seek.
 *  Guarantees 2 last bits of previous Word 10 + TLM + HOW. */
#define BITS_DECODED_FOR_POLARITY (62)
/** Number of bits that needs to be decoded for subframe seek.
 *  Guarantees 2 last bits of previous Word 10
 *  + full subframe + next TLM + HOW. */
#define BITS_DECODED_FOR_SUBFRAME (362)

/** Minimum GPS LNAV valid subframe number */
#define GPS_LNAV_SUBFRAME_MIN (1)
/** Maximum GPS LNAV valid subframe number */
#define GPS_LNAV_SUBFRAME_MAX (5)
#define GPS_LNAV_SUBFRAME_CNT \
  (GPS_LNAV_SUBFRAME_MAX - GPS_LNAV_SUBFRAME_MIN + 1)
/** Number of words we want to store */
#define GPS_LNAV_WORD_STORE_CNT (8)

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
  /**< Number of bits decoded. */
  u16 bits_decoded;
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
  gps_time_t almanac_time;

  utc_params_t utc;

  ionosphere_t iono;

  almanac_t almanac;

  ephemeris_t ephemeris;

  u32 gps_l2c_sv_capability;
  u32 almanac_health_upd_flags;

  u8 shi_ephemeris;

  bool ephemeris_upd_flag;
  bool iono_corr_upd_flag;
  bool utc_params_upd_flag;
  bool gps_l2c_sv_capability_upd_flag;
  bool shi_ephemeris_upd_flag;
  bool almanac_upd_flag;
  bool almanac_time_upd_flag;
  bool invalid_control_or_data;

  u8 almanac_health[32];
} gps_l1ca_decoded_data_t;


u32 extract_word(const nav_msg_t *n, u16 bit_index, u8 n_bits, u8 invert);
s32 adjust_tow(u32 TOW_trunc);
void nav_msg_init(nav_msg_t *n);
void nav_msg_clear_decoded(nav_msg_t *n);
s32 nav_msg_update(nav_msg_t *n, bool bit_val);
u8 nav_parity(u32 *word);
bool subframe_ready(const nav_msg_t *n);
s8 process_subframe(nav_msg_t *n,
                    me_gnss_signal_t mesid,
                    gps_l1ca_decoded_data_t *data);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* SWIFTNAV_NAV_MSG_H */
