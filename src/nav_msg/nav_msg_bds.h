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

#include <stdbool.h>
#include <swiftnav/almanac.h>
#include <swiftnav/common.h>
#include <swiftnav/ephemeris.h>
#include <swiftnav/ionosphere.h>

#include "nav_data_sync/nav_data_sync.h"
#include "signal_db/signal_db.h"

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
/** BDS D1 subframe length [seconds] */
#define BDS_D1_SUBFRAME_LEN_SECONDS (6)

/**
 * "The Open Service Signal in Space Navigation Data Comparison
 * of the Global Positioning System and the BeiDou Navigation Satellite System"
 * Shau-Shiun Jan and An-Lin Tao, 2014 August
 * https://www.ncbi.nlm.nih.gov/pmc/articles/PMC4178995/
 */
#define BDS_FIT_INTERVAL_SECONDS (3 * HOUR_SECS)

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
  me_gnss_signal_t mesid;
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
  /**< Decoded TOW [ms] */
  s32 TOW_ms;
  /**< SV health status */
  health_t health;
  /**< Decoded subframe data */
  u32 page_words[BDS_WORD_SUBFR * BDS_SUBFRAME_MAX];
  /**< Decoded subframe rx time */
  u64 subfr_times[BDS_SUBFRAME_MAX];
  /**< Successfully decoded words in page */
  u64 goodwords_mask;

  u16 bit_cnt; /**< For navbit data integrity checks */
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

/** BDS data decoding status */
typedef enum {
  BDS_DECODE_WAIT,       /**< Decoding in progress */
  BDS_DECODE_RESET,      /**< Decoding error or sensitivity mode */
  BDS_DECODE_POL_UPDATE, /**< Polarity decoded */
  BDS_DECODE_TOW_UPDATE, /**< TOW decoded */
  BDS_DECODE_EPH_UPDATE, /**< Ephemeris decoded */
} bds_decode_status_t;

u32 bch_crc_check(const u32 *subfr, const u8 size);
void bds_nav_msg_init(nav_msg_bds_t *n, const me_gnss_signal_t *mesid);
void bds_nav_msg_clear_decoded(nav_msg_bds_t *n);
bds_decode_status_t bds_data_decoding(nav_msg_bds_t *n, nav_bit_t nav_bit);
nav_data_sync_t construct_bds_data_sync(const nav_msg_bds_t *n,
                                        bds_decode_status_t status);
bool bds_nav_msg_update(nav_msg_bds_t *n);
bds_decode_status_t bds_d2_processing(nav_msg_bds_t *n,
                                      bds_d2_decoded_data_t *data);
bds_decode_status_t bds_d1_processing(nav_msg_bds_t *n,
                                      bds_d1_decoded_data_t *data);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* SWIFTNAV_NAV_MSG_BDS_H */
