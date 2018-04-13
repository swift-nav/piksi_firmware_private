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

#ifndef SWIFTNAV_NAV_MSG_BDS_H
#define SWIFTNAV_NAV_MSG_BDS_H

#include <libswiftnav/almanac.h>
#include <libswiftnav/common.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/ionosphere.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** Minimum BDS valid subframe number */
#define BDS_SUBFRAME_MIN 1
/** Maximum BDS valid subframe number */
#define BDS_SUBFRAME_MAX 5
#define BDS_SUBFRAME_CNT (BDS_SUBFRAME_MAX - BDS_SUBFRAME_MIN + 1)
/** Number of words we want to store */
#define BDS_WORD_SUBFR 10

/** BDS D1 decode buffer size (352 bits) [11 x 32-bit words] */
#define BDS_NAV_MSG_SUBFRAME_WORDS_LEN (BDS_WORD_SUBFR + 1)
/** BDS D1 decode buffer size (352 bits) [bits] */
#define BDS_NAV_MSG_SUBFRAME_BITS_LEN (NAV_MSG_SUBFRAME_WORDS_LEN * 32)

/** Special value to marking maximum subframe cache entry age [6 seconds] */
#define BDS_SUBFRAME_AGE_INVALID ((u8)-1)
/** Special value to marking maximum subframe cache entry age [6 seconds] */
#define BDS_SUBFRAME_AGE_MAX ((u8)100)

#define BDS_WEEK_TO_GPS_WEEK 1356
#define BDS_SECOND_TO_GPS_SECOND 14

/** TODO: Check Beidou fit_interval definition */
#define BDS_FIT_INTERVAL_SECONDS (2 * HOUR_SECS)

/**
 * BDS D1 and D2 message decoder object.
 *
 * The object for decoding Beidou D1/D2 messages for PRNs 1-37.
 *
 * \sa nav_msg_init
 * \sa nav_msg_update
 * \sa subframe_ready
 * \sa process_subframe
 */
typedef struct {
  u8 prn;
  /**< Decoder buffer (330 bits) */
  u32 subframe_bits[BDS_NAV_MSG_SUBFRAME_WORDS_LEN];
  /**< Received bit counter */
  u16 bit_index;
  /**< Received bit counter for subframe start */
  u16 subfr_bit_index;
  /**< Has subframe sync */
  bool subfr_sync;
  /**< Polarity of the data */
  s8 bit_polarity;
  /**< Decoded subframe data */
  u32 page_words[BDS_WORD_SUBFR * BDS_SUBFRAME_MAX];
  /**< Successfully decoded words in page */
  u64 subfr_times[BDS_SUBFRAME_MAX];
  /**< Successfully decoded words in page */
  u64 goodwords_mask;
} nav_msg_bds_t;

typedef struct _bds_d1_decoded_data {
  u32 split_toe;
  u32 split_toe_mask;
  u32 aodc, aode;

  ephemeris_t ephemeris;
  bool ephemeris_upd_flag;

  ionosphere_t iono;
  bool iono_corr_upd_flag;

  utc_params_t utc;
  bool utc_params_upd_flag;

  bool invalid_control_or_data;
} bds_d1_decoded_data_t;

typedef struct _bds_d2_decoded_data {
  ephemeris_t ephemeris;
  bool ephemeris_upd_flag;

  ionosphere_t iono;
  bool iono_corr_upd_flag;

  utc_params_t utc;
  bool utc_params_upd_flag;

  bool invalid_control_or_data;
} bds_d2_decoded_data_t;

void bds_nav_msg_init(nav_msg_bds_t *n, u8 prn);
void bds_nav_msg_clear_decoded(nav_msg_bds_t *n);
bool bds_nav_msg_update(nav_msg_bds_t *n, bool bit_val);

s32 bds_d1_process_subframe(nav_msg_bds_t *n,
                            const me_gnss_signal_t mesid,
                            bds_d1_decoded_data_t *data);

s32 bds_d2_process_subframe(nav_msg_bds_t *n,
                            const me_gnss_signal_t mesid,
                            bds_d2_decoded_data_t *data);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* SWIFTNAV_NAV_MSG_BDS_H */
