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

#ifndef SWIFTNAV_NAV_MSG_GAL_H
#define SWIFTNAV_NAV_MSG_GAL_H

#include <libswiftnav/almanac.h>
#include <libswiftnav/common.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/ionosphere.h>

#include "nav_data_sync/nav_data_sync.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** Minimum GAL valid subframe number */
#define GAL_INAV_SUBFR_MIN 1
/** Maximum GAL valid subframe number */
#define GAL_INAV_SUBFR_MAX 24
#define GAL_INAV_SUBFR_CNT (GAL_INAV_SUBFR_MAX - GAL_INAV_SUBFR_MIN + 1)

/** I/NAV Number of pages in a subframe */
#define GAL_INAV_PAGES_SUBFR 15

/** Number of symbols in one Galileo I/NAV preamble */
#define GAL_INAV_SYNC_BITS 10
/** Number of symbols in one Galileo I/NAV page */
#define GAL_INAV_PAGE_SYMB 240
/** Number of bits in one Galileo I/NAV page */
#define GAL_INAV_PAGE_BIT (GAL_INAV_PAGE_SYMB / 2)
/** Equivalent number of bytes in one Galileo I/NAV page */
#define GAL_INAV_PAGE_BYTE ((GAL_INAV_PAGE_BIT + CHAR_BIT - 1) / CHAR_BIT)
/** Galileo CRC length in bits */
#define GAL_MSG_CRC_LENGTH 24

/** Number of bits in one Galileo I/NAV page content */
#define GAL_INAV_CONTENT_BIT 128
/** Number of Bytes in one Galileo I/NAV page content */
#define GAL_INAV_CONTENT_BYTE ((GAL_INAV_CONTENT_BIT + CHAR_BIT - 1) / CHAR_BIT)

/** Galileo I/NAV page tail bits */
#define GAL_INAV_TAIL_BIT 6

/** 2x250 symbols per I/NAV page */
#define GAL_INAV_DECODE_BUFF_SIZE \
  (2 * (GAL_INAV_SYNC_BITS + GAL_INAV_PAGE_SYMB))
/** 2x120 bits per I/NAV page */
#define GAL_INAV_PAGE_BYTES 30
/** 2x120 bits per I/NAV page */
#define GAL_INAV_PAGE_WORDS 8
/** GST week offset to GPS */
#define GAL_WEEK_TO_GPS_WEEK 1024

/** Galileo deinterleaver parameters */
#define GAL_INAV_INTERL_ROW 8
#define GAL_INAV_INTERL_COL 30

/** traceback length, usually 5*(k-1)*/
#define GAL_INAV_V27_HISTORY_LENGTH 240

/** Viterbi decoder reversed polynomial A */
#define GAL_INAV_V27_POLY_A 0x4F /* 0b01001111 - reversed 0171*/
/** Viterbi decoder reversed polynomial B */
#define GAL_INAV_V27_POLY_B 0x6D /* 0b01101101 - reversed 0133 */

/**
 * Galileo fit_interval definition
 * "Galileo Open Service: Service Definition Document"
 * Issue 1 Revision 0, 2016 December
 * Section 2.4.1
 * https://www.gsc-europa.eu/system/files/galileo_documents/Galileo-OS-SDD.pdf
 */
#define GAL_FIT_INTERVAL_SECONDS (4 * HOUR_SECS)

/**
 * Galileo message decoder object.
 *
 * The object for decoding Galileo symbols
 *
 * \sa nav_msg_init
 * \sa nav_msg_update
 * \sa subframe_ready
 * \sa process_subframe
 */
typedef struct {
  v27_t decoder;
  v27_decision_t decisions[GAL_INAV_V27_HISTORY_LENGTH];

  me_gnss_signal_t mesid;
  /**< Decoder buffer has 500 symbols for two half pages */
  u8 decoder_buffer[GAL_INAV_DECODE_BUFF_SIZE];
  /**< Each page is composed of 2x120 bit halves */
  u8 subframe_bits[GAL_INAV_PAGE_BYTES];
  /**< Oldest bit index */
  u16 bit_index;
  /* hold off before trying decoding again */
  u16 holdoff;

  /* last meaningful received data packet */
  u8 raw_content[GAL_INAV_CONTENT_BYTE];

  /* IODnav used to collect ephemeris from pages 1, 2, 3, 4 and 5 */
  u16 iod_nav[4];
  /* Ephemeris from pages 1, 2, 3, 4 and 5 */
  u8 raw_eph[5][GAL_INAV_CONTENT_BYTE];

  /* IODa used to collect almanac from different pages */
  u16 iod_alm[4];
  /* Almanac from pages 7, 8, 9 and 10 */
  u8 raw_alm[4][GAL_INAV_CONTENT_BYTE];

  /**< Polarity of the data */
  s8 bit_polarity;
  /**< Decoded TOW [ms] */
  s32 TOW_ms;
  /**< SV health status */
  health_t health;
} nav_msg_gal_inav_t;

typedef enum _inav_data_type_e {
  INAV_INCOMPLETE = -1,
  INAV_TOW = 0,
  INAV_EPH = 1,
  INAV_UTC = 6,
  INAV_ALM = 7,
  INAV_DUMMY = 63,
} inav_data_type_t;

typedef struct _gal_inav_decoded_t {
  u32 toe;
  u32 aodc, aode;

  ephemeris_t ephemeris;
  bool ephemeris_upd_flag;

  ionosphere_t iono;
  bool iono_corr_upd_flag;

  utc_params_t utc;
  bool utc_params_upd_flag;

  almanac_t alm[3];
} gal_inav_decoded_t;

/** GAL data decoding status */
typedef enum {
  GAL_DECODE_WAIT,         /**< Decoding in progress */
  GAL_DECODE_RESET,        /**< Decoding error or sensitivity mode */
  GAL_DECODE_TOW_UPDATE,   /**< TOW decoded */
  GAL_DECODE_EPH_UPDATE,   /**< Ephemeris decoded */
  GAL_DECODE_DUMMY_UPDATE, /**< Dummy message decoded */
} gal_decode_status_t;

void gal_inav_msg_init(nav_msg_gal_inav_t *n, const me_gnss_signal_t *mesid);
void gal_inav_msg_clear_decoded(nav_msg_gal_inav_t *n);
bool gal_inav_msg_update(nav_msg_gal_inav_t *n, s8 bit_val);

inav_data_type_t parse_inav_word(nav_msg_gal_inav_t *nav_msg,
                                 gal_inav_decoded_t *dd);
gal_decode_status_t gal_data_decoding(nav_msg_gal_inav_t *n, nav_bit_t nav_bit);
nav_data_sync_t construct_gal_data_sync(const nav_msg_gal_inav_t *n,
                                        gal_decode_status_t status);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* SWIFTNAV_NAV_MSG_GAL_H */
