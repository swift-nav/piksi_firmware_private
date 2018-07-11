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

#ifndef SWIFTNAV_CNAV_MSG_H
#define SWIFTNAV_CNAV_MSG_H

#include <libfec/fec.h>
#include <libswiftnav/bits.h>
#include <libswiftnav/common.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/edc.h>
#include <libswiftnav/gnss_time.h>

#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "me_constants.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** \addtogroup GPS_L2
 * \{ */
/** \addtogroup gps_cnav_decoder
 * \{ */

/*
 * GPS L2C message constants.
 */

/** GPS L2C preamble */
#define GPS_CNAV_PREAMBLE1 (0b10001011u)
/** Inverted GPS L2C preamble */
#define GPS_CNAV_PREAMBLE2 (0b01110100u)
/** GPS L2C preamble length in bits */
#define GPS_CNAV_PREAMBLE_LENGTH (8)
/** GPS LC2 CNAV CRC length in bits */
#define GPS_CNAV_MSG_CRC_LENGTH (24)
/** GPS L2C CNAV message payload length in bits */
#define GPS_CNAV_MSG_DATA_LENGTH (GPS_CNAV_MSG_LENGTH - GPS_CNAV_MSG_CRC_LENGTH)
/** GPS L2C CNAV message lock detector threshold */
#define GPS_CNAV_LOCK_MAX_CRC_FAILS (1)

/** Size of the Viterbi decoder history. */
#define GPS_L2_V27_HISTORY_LENGTH_BITS 64
/** Bits to accumulate before decoding starts. */
#define GPS_L2C_V27_INIT_BITS (32)
/** Bits to decode at a time. */
#define GPS_L2C_V27_DECODE_BITS (32)
/** Bits in decoder tail. We ignore them. */
#define GPS_L2C_V27_DELAY_BITS (32)
/** L2C convolutional encoder constraint length */
#define GPS_L2C_V27_CONSTRAINT_LENGTH (7)

/** The bit string of “1000000000000” will indicate that the group delay value
    is not available. Sign extended to s16*/
#define INVALID_GROUP_DELAY_VALUE ((s16)0xF000)

/** Group delay value scale factor 2^-35*/
#define GROUP_DELAY_SCALE C_1_2P35

typedef enum {
  CNAV_MSG_TYPE_10 = 10,
  CNAV_MSG_TYPE_11 = 11,
  CNAV_MSG_TYPE_30 = 30,
  CNAV_MSG_TYPE_32 = 32,
  CNAV_MSG_TYPE_33 = 33
} cnav_msg_type_t;

/**
 * GPS CNAV message type 30 data.
 *
 */
typedef struct {
  bool tgd_valid;
  s16 tgd; /**< seconds, scale factor 2^-35 */
  bool isc_l1ca_valid;
  s16 isc_l1ca; /**< seconds, scale factor 2^-35 */
  bool isc_l2c_valid;
  s16 isc_l2c; /**< seconds, scale factor 2^-35 */
} cnav_msg_type_30_t;

/**
 * GPS CNAV message type 33 data.
 *
 */
typedef struct {
  s16 a0;     /**< seconds, scale factor 2^-35 */
  s16 a1;     /**< s/s, scale factor 2^-51 */
  s8 a2;      /**< s/s^2, scale factor 2^-68 */
  s8 dt_ls;   /**< current or past leap second count */
  u16 tot;    /**< Time data reference time of week, scale factor 2^4 */
  u16 wn_ot;  /**< Time data reference week number */
  u16 wn_lsf; /**< Leap second reference week number */
  u8 dn;      /**< Leap second reference day number */
  s8 dt_lsf;  /**< current or future leap second count */
} cnav_msg_type_33_t;

/**
 * GPS CNAV message type 10 data.
 *
 */
typedef struct {
  bool l1_health;
  bool l2_health;
  bool l5_health;
} cnav_msg_type_10_t;

/**
 * GPS CNAV message container.
 *
 * @sa cnav_msg_decoder_add_symbol
 */
typedef struct {
  u8 prn;     /**< SV PRN. 0..31 */
  u8 msg_id;  /**< Message id. 0..31 */
  u32 tow;    /**< GPS ToW in 6-second units. Multiply to 6 to get seconds. */
  bool alert; /**< CNAV message alert flag */
  s8 bit_polarity; /**< Polarity of data bits */
  union {
    cnav_msg_type_30_t type_30;
    cnav_msg_type_10_t type_10;
    cnav_msg_type_33_t type_33;
  } data;
} cnav_msg_t;

/**
 * GPS CNAV decoder component.
 * This component controls symbol decoding string.
 *
 * @sa cnav_msg_decoder_t
 */
typedef struct {
  v27_t dec; /**< Viterbi block decoder object */
  v27_decision_t decisions[GPS_L2_V27_HISTORY_LENGTH_BITS];
  /**< Decision graph */
  unsigned char symbols[(GPS_L2C_V27_INIT_BITS + GPS_L2C_V27_DECODE_BITS) * 2];
  /**< Symbol buffer */
  size_t n_symbols; /**< Count of symbols in the symbol buffer */
  unsigned char decoded[GPS_L2C_V27_DECODE_BITS + GPS_L2C_V27_DELAY_BITS];
  /**< Decode buffer */
  size_t n_decoded;   /**< Number of bits in the decode buffer */
  bool preamble_seen; /**< When true, the decode buffer is aligned on
                       *   preamble. */
  bool invert;        /**< When true, indicates the bits are inverted */
  bool message_lock;  /**< When true, indicates the message boundary
                       *   is found. */
  bool crc_ok;        /**< Flag that the last message had good CRC */
  size_t n_crc_fail;  /**< Counter for CRC failures */
  bool init;          /**< Initial state flag. When true, initial bits
                       *   do not produce output. */
} cnav_v27_part_t;

/**
 * GPS CNAV message lock and decoder object.
 *
 * Decoder uses two Viterbi decoder objects to ensure the lock is acquired when
 * the input symbol phase is not known.
 */
typedef struct {
  cnav_v27_part_t part1; /**< Decoder for odd symbol pairs */
  cnav_v27_part_t part2; /**< Decoder for even symbol pairs */
} cnav_msg_decoder_t;

u32 _cnav_compute_crc(cnav_v27_part_t *part);
u32 _cnav_extract_crc(const cnav_v27_part_t *part);
void _cnav_rescan_preamble(cnav_v27_part_t *part);
const v27_poly_t *cnav_msg_decoder_get_poly(void);
void cnav_msg_decoder_init(cnav_msg_decoder_t *dec);
bool cnav_msg_decoder_add_symbol(cnav_msg_decoder_t *dec,
                                 unsigned char symbol,
                                 cnav_msg_t *msg,
                                 u32 *delay);
bool cnav_33_to_utc(const cnav_msg_type_33_t *msg, utc_params_t *u);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

/** \} */
/** \} */

#endif /* SWIFTNAV_CNAV_MSG_H */
