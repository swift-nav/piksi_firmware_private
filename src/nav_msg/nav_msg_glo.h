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

#ifndef SWIFTNAV_NAV_MSG_GLO_H
#define SWIFTNAV_NAV_MSG_GLO_H

#include <stdbool.h>
#include <swiftnav/common.h>
#include <swiftnav/ephemeris.h>

#include "signal_db/signal_db.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** GLO time mark is 30 symbols of duration 10 millisec each */
#define GLO_TM_LEN_SYMBOLS 30 /** Length of GLO time mark in symbols */
#define GLO_TM_MASK ((1 << GLO_TM_LEN_SYMBOLS) - 1)
#define GLO_TM (0x3E375096)                /** Time mark in GLO nav string */
#define GLO_TM_INV (~GLO_TM & GLO_TM_MASK) /** Bitwise inverted time mark */

/** Length of GLO navigation string */
#define GLO_STR_LEN 85

/** Length of GLO navigation string in seconds */
#define GLO_STR_LEN_S 2
/** Length of GLO time mark seconds */
#define GLO_STR_TIME_MARK_LEN_S (0.3)

/* Collect 5 five strings (1-5) */
#define GLO_STRINGS_TO_COLLECT 5
/** Bitmap to indicate all five strings have been decoded */
#define GLO_STRINGS_NEEDED 0x1F
/** Bitmap to indicate all four strings for tow have been decoded */
#define GLO_STRINGS_NEEDED_FOR_TOW 0x1B

/** Zero-based indexes of strings which are skipped */
#define GLO_SKIP_STRING_NONE (0 - 1)
#define GLO_SKIP_STRING_3 (3 - 1)

/* States of receiver GLO bitstream */
typedef enum {
  SYNC_TM,     /**< Time mark search */
  GET_DATA_BIT /**< Data bit receive */
} glo_receive_machine;

/** GLO nav message status */
typedef enum {
  GLO_STRING_NOT_READY, /**< GLO string being collected */
  GLO_STRING_READY,     /**< GLO string ready for decoding */
  GLO_TIME_MARK_DECODED /**< GLO 300ms time mark was decoded */
} nav_msg_status_t;

/** GLO string decoding status */
typedef enum {
  GLO_STRING_DECODE_ERROR = -1, /**< Error during GLO string processing */
  GLO_STRING_DECODE_STRING,     /**< GLO string decoded (polarity) */
  GLO_STRING_DECODE_TOW,        /**< Strings for TOW decoded (TOW) */
  GLO_STRING_DECODE_EPH,        /**< GLO string decoding done (ephemeris) */
} string_decode_status_t;

/* Structure used for relative code removal.
 * See Figure 3.4. of GLO ICD 5.1. 2008. (english) */
typedef struct {
  u8 state;
} relcode_t;

/* The structure is used for GLO receive and decode bitstream */
typedef struct {
  /** buffer for one GLO string */
  glo_string_t string;
  /* Array to store string receive time in milliseconds, based on systicks */
  u32 string_receive_time_ms[GLO_STRINGS_TO_COLLECT];

  /** how many bits written into GLO string buffer*/
  u16 current_head_bit_index;

  /** GLO timestamp of the beginning of GLO frame.
      Used to compute the timestamp of the last data bit. */
  glo_time_t tk;

  /** TOE as a GLO epoch. h/m/s comes from the field tb in the GLO string 2 */
  glo_time_t toe;
  float tau_gps_s;     /**< correction to GPS time relative to GLO time [s] */
  u8 decoded_strings;  /**< bit field to indicate decoded strings 1-5 */
  gps_time_t gps_time; /**< GPS time of the last data bit [s] */
  glo_receive_machine state; /**< current state of receiver */
  u8 meander_bits_cnt : 2;   /**< counter for line code bits, MAX is 2 */
  u8 manchester : 2;         /**< 2 bits line code received */
  s8 bit_polarity;           /**< Bit polarity on decoding */
  relcode_t relcode;         /**< Structure used for relative code removal */
  ephemeris_t eph;           /**< GLO ephemeris placeholder.
                              *   GLO ephemeris can not be saved on NDB,
                              *   until glo_orbit_slot_id is decoded.
                              */
  me_gnss_signal_t mesid;    /**< decoding channel ME sid */

  u16 bit_cnt;         /**< For navbit data integrity checks */
  u8 age_of_data_days; /**< Age of the navigation data in days */
} nav_msg_glo_t;

void nav_msg_init_glo(nav_msg_glo_t *n, me_gnss_signal_t mesid);
bool timemark_glo_decoded(nav_msg_glo_t *n, bool symbol);
nav_msg_status_t get_data_bits_glo(nav_msg_glo_t *n, bool symbol);
string_decode_status_t process_string_glo(nav_msg_glo_t *n, u32 time_tag_ms);
nav_msg_status_t nav_msg_update_glo(nav_msg_glo_t *n, bool symbol);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* SWIFTNAV_NAV_MSG_GLO_H */
