/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_SBAS_MSG_H
#define SWIFTNAV_SBAS_MSG_H

#include <libfec/fec.h>
#include <swiftnav/bits.h>
#include <swiftnav/common.h>
#include <swiftnav/constants.h>
#include <swiftnav/edc.h>
#include <swiftnav/signal.h>

#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "shm/shm.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** Size of the Viterbi decoder history. */
#define SBAS_V27_HISTORY_LENGTH_BITS 64
/** Bits to accumulate before decoding starts. */
#define SBAS_V27_INIT_BITS (32)
/** Bits to decode at a time. */
#define SBAS_V27_DECODE_BITS (32)
/** Bits in decoder tail. We ignore them. */
#define SBAS_V27_DELAY_BITS (32)
/** SBAS convolutional encoder constraint length */
#define SBAS_V27_CONSTRAINT_LENGTH (7)

/**
 * SBAS message container.
 *
 * @sa sbas_msg_decoder_add_symbol
 */
typedef struct {
  gnss_signal_t sid; /**< SV identifier */
  s32 tow_ms;        /**< GPS TOW [ms] */
  s16 wn;            /**< GPS Week Number */
  health_t health;   /**< Health status */
  s8 bit_polarity;   /**< Polarity of data bits */
} sbas_msg_t;

/**
 * SBAS decoder component.
 * This component controls symbol decoding string.
 *
 * @sa sbas_msg_decoder_t
 */
typedef struct {
  v27_t dec; /**< Viterbi block decoder object */
  v27_decision_t decisions[SBAS_V27_HISTORY_LENGTH_BITS];
  /**< Decision graph */
  unsigned char symbols[(SBAS_V27_INIT_BITS + SBAS_V27_DECODE_BITS) * 2];
  /**< Symbol buffer */
  size_t n_symbols; /**< Count of symbols in the symbol buffer */
  unsigned char decoded[SBAS_V27_DECODE_BITS + SBAS_V27_DELAY_BITS];
  /**< Decode buffer */
  size_t n_decoded;   /**< Number of bits in the decode buffer */
  bool preamble_seen; /**< When true, the decode buffer is aligned on
                       *   preamble. */
  u8 n_preamble;      /**< Number of next preamble to search */
  bool invert;        /**< When true, indicates the bits are inverted */
  bool message_lock;  /**< When true, indicates the message boundary
                       *   is found. */
  bool crc_ok;        /**< Flag that the last message had good CRC */
  size_t n_crc_fail;  /**< Counter for CRC failures */
  bool init;          /**< Initial state flag. When true, initial bits
                       *   do not produce output. */
} sbas_v27_part_t;

/**
 * SBAS message lock and decoder object.
 *
 * Decoder uses two Viterbi decoder objects to ensure the lock is acquired when
 * the input symbol phase is not known.
 */
typedef struct {
  sbas_v27_part_t part1; /**< Decoder for odd symbol pairs */
  sbas_v27_part_t part2; /**< Decoder for even symbol pairs */
} sbas_msg_decoder_t;

const v27_poly_t *sbas_msg_decoder_get_poly(void);
void sbas_msg_decoder_init(sbas_msg_decoder_t *dec);
bool sbas_msg_decoder_add_symbol(sbas_msg_decoder_t *dec,
                                 unsigned char symbol,
                                 sbas_msg_t *msg);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* SWIFTNAV_SBAS_MSG_H */
