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

#include "nav_msg/sbas_msg.h"
#include "nav_msg/nav_msg.h" /* For BIT_POLARITY_... constants */

#include <limits.h>
#include <string.h>

/*
 * Block Viterbi decoding parameters.
 */
/** Viterbi decoder reversed polynomial A */
#define SBAS_V27_POLY_A (0x4F) /* 0b01001111 - reversed 0171*/
/** Viterbi decoder reversed polynomial B */
#define SBAS_V27_POLY_B (0x6D) /* 0b01101101 - reversed 0133 */

/*
 * SBAS message constants.
 */

/** Number of SBAS preambles */
#define NUM_SBAS_PREAMBLES (3)
/** SBAS preamble 1 */
#define SBAS_PREAMBLE1 (0b01010011u)
/** Inverted SBAS preamble 1 */
#define SBAS_PREAMBLE1_INV (0b10101100u)
/** SBAS preamble 2 */
#define SBAS_PREAMBLE2 (0b10011010u)
/** Inverted SBAS preamble 2 */
#define SBAS_PREAMBLE2_INV (0b01100101u)
/** SBAS preamble 3 */
#define SBAS_PREAMBLE3 (0b11000110u)
/** Inverted SBAS preamble 3 */
#define SBAS_PREAMBLE3_INV (0b00111001u)
/** SBAS preamble length in bits */
#define SBAS_PREAMBLE_LENGTH (8)
/** SBAS CRC length in bits */
#define SBAS_MSG_CRC_LENGTH (24)
/** SBAS message payload length in bits */
#define SBAS_MSG_DATA_LENGTH (SBAS_MSG_LENGTH - SBAS_MSG_CRC_LENGTH)
/** SBAS message lock detector threshold */
#define SBAS_LOCK_MAX_CRC_FAILS (1)

/**
 * Computes CRC-24Q from a SBAS message buffer.
 * CRC-24Q is computed for 226 bits. For a purpose of 8-bit alignment, the
 * message is assumed to be prepended with four zero bits.
 *
 * \param[in] part Decoder component with payload
 *
 * \return CRC-24Q value.
 *
 * \private
 */
static u32 _sbas_compute_crc(sbas_v27_part_t *part) {
  u32 crc = crc24q_bits(0, part->decoded, SBAS_MSG_DATA_LENGTH, part->invert);

  return crc;
}

/**
 * Extracts CRC-24Q from a SBAS message buffer.
 * CRC-24Q value is the last 24 bits from 250 bits message buffer.
 *
 * \param[in] part Decoded component with payload.
 *
 * \return CRC24-Q value.
 *
 * \private
 */
static u32 _sbas_extract_crc(const sbas_v27_part_t *part) {
  u32 crc = getbitu(part->decoded, SBAS_MSG_DATA_LENGTH, SBAS_MSG_CRC_LENGTH);
  if (part->invert) {
    crc ^= 0xFFFFFF;
  }
  return crc;
}

/**
 * Helper to rescan for preamble in the received buffer.
 * Occasionally there could be a false lock on message contents if it the
 * preamble sequence is encountered in the message body. For this case, the
 * function performs for a scan for a preamble with a different offset:
 * - When found, the preamble octet is moved into the head of the buffer.
 * - When not found, only 7 bits are left in the buffer.
 *
 * \param[in,out] part Decoded component.
 *
 * \return None
 *
 * \private
 */
static void _sbas_rescan_preamble(sbas_v27_part_t *part) {
  part->preamble_seen = false;
  u8 preamble = 0;
  u8 preamble_inv = 0;
  if (0 == part->n_preamble || 1 == part->n_preamble) {
    preamble = SBAS_PREAMBLE1;
    preamble_inv = SBAS_PREAMBLE1_INV;
  } else if (2 == part->n_preamble) {
    preamble = SBAS_PREAMBLE2;
    preamble_inv = SBAS_PREAMBLE2_INV;
  } else if (3 == part->n_preamble) {
    preamble = SBAS_PREAMBLE3;
    preamble_inv = SBAS_PREAMBLE3_INV;
  } else {
    part->n_preamble = 0;
    log_warn("sbas premable index gone wild!");
  }

  if (part->n_decoded > SBAS_PREAMBLE_LENGTH + 1) {
    for (size_t i = 1, j = part->n_decoded - SBAS_PREAMBLE_LENGTH; i < j; ++i) {
      u32 c = getbitu(part->decoded, i, SBAS_PREAMBLE_LENGTH);
      if (preamble == c || preamble_inv == c) {
        part->preamble_seen = true;
        part->invert = (preamble_inv == c);
        /* We shift the accumulated bits to the beginning of the buffer */
        bitshl(part->decoded, sizeof(part->decoded), i);
        part->n_decoded -= i;
        part->n_preamble++;
        if (part->n_preamble > NUM_SBAS_PREAMBLES) {
          part->n_preamble = 1;
        }
        break;
      }
    }
  }
  if (!part->preamble_seen && part->n_decoded >= SBAS_PREAMBLE_LENGTH) {
    bitshl(part->decoded,
           sizeof(part->decoded),
           part->n_decoded - SBAS_PREAMBLE_LENGTH + 1);
    part->n_decoded = SBAS_PREAMBLE_LENGTH - 1;
  }
}

/**
 * Feed a symbol into Viterbi decoder instance.
 *
 * The method uses block Viterbi decoder. It first accumulates initial number of
 * symbols, and after that runs decoding every time the buffer is full. Only
 * some of the decoded symbols are used.
 *
 * \param[in,out] part Decoder object
 * \param[in]     s    Symbol (0x00 - Hard 0, 0xFF - Hard 1)
 *
 * \return None
 *
 * \private
 */
static void _sbas_add_symbol(sbas_v27_part_t *part, u8 s) {
  part->symbols[part->n_symbols++] = s;

  if (part->init) {
    /* Initial step - load more symbols without decoding. */
    if (part->n_symbols < (SBAS_V27_INIT_BITS + SBAS_V27_DECODE_BITS) * 2) {
      return;
    }
    part->init = false;
  } else if (part->n_symbols < SBAS_V27_DECODE_BITS * 2) {
    /* Wait until decoding block is accumulated */
    return;
  }

  /* Feed accumulated symbols into the buffer, reset the number of accumulated
   * symbols. */
  v27_update(&part->dec, part->symbols, part->n_symbols / 2);
  part->n_symbols = 0;

  /* Decode N+M bits, where:
   * - N - Number of bits to put into decoded buffer
   * - M - Number of bits in the tail to ignore.
   */
  unsigned char
      tmp_bits[(SBAS_V27_DECODE_BITS + SBAS_V27_DELAY_BITS + CHAR_BIT - 1) /
               CHAR_BIT];

  v27_chainback_likely(
      &part->dec, tmp_bits, SBAS_V27_DECODE_BITS + SBAS_V27_DELAY_BITS);

  /* Read decoded bits and add them to the decoded buffer */
  bitcopy(part->decoded, part->n_decoded, tmp_bits, 0, SBAS_V27_DECODE_BITS);
  part->n_decoded += SBAS_V27_DECODE_BITS;

  /* Depending on the decoder state, one of the following actions are
   * possible:
   * - If no message lock
   *   - If no preamble seen - look for preamble
   *   - If preamble seen - collect 250 bits
   *     - If 250 bits are collected - verify CRC
   *       - If CRC is OK - message lock is acquired
   *       - If CRC fails - rescan for preamble
   *         - If found - continue collecting 250 bits
   *         - If not found - continue preamble wait
   * - If message lock
   *   - If 250 bits collected, compute CRC
   *     - If CRC is OK, message can be decoded
   *     - If CRC is not OK, discard data
   */

  bool retry = true;
  while (retry) {
    retry = false;

    if (!part->preamble_seen) {
      /* Rescan for preamble if possible. The first bit is ignored. */
      _sbas_rescan_preamble(part);
    }
    if (part->preamble_seen && SBAS_MSG_LENGTH <= part->n_decoded) {
      /* We have collected 250 bits starting from message preamble. Now try
       * to compute CRC-24Q */
      u32 crc = _sbas_compute_crc(part);
      u32 crc2 = _sbas_extract_crc(part);

      if (part->message_lock) {
        /* We have message lock */
        part->crc_ok = (crc == crc2);
        if (part->crc_ok) {
          /* Reset message lock counter */
          part->n_crc_fail = 0;
        } else {
          /* Increment message lock counter */
          part->n_crc_fail++;
          if (part->n_crc_fail > SBAS_LOCK_MAX_CRC_FAILS) {
            /* CRC has failed too many times - drop the lock. */
            part->n_crc_fail = 0;
            part->message_lock = false;
            part->preamble_seen = false;
            part->n_preamble = 0;
            /* Try to find a new preamble, reuse data from buffer. */
            retry = true;
          }
        }
      } else if (crc == crc2) {
        /* CRC match - message can be decoded */
        part->message_lock = true;
        part->crc_ok = true;
        part->n_crc_fail = 0;
      } else {
        /* There is no message lock and the CRC check fails. Assume there is
         * false positive lock - rescan for preamble. */
        part->crc_ok = false;
        part->preamble_seen = false;
        part->n_preamble = 0;

        /* CRC mismatch - try to re-scan for preamble */
        retry = true;
      }
    } else {
      /* No preamble or preamble and less than 250 bits decoded */
    }
  }
}

/**
 * Invert message bits in the buffer.
 *
 * The method inverts bits of the decoded data.
 *
 * \param[in,out] part Decoder component with a message buffer.
 *
 * \return None
 */
static void _sbas_msg_invert(sbas_v27_part_t *part) {
  for (size_t i = 0; i < sizeof(part->decoded); i++) {
    part->decoded[i] ^= 0xFFu;
  }
}

/**
 * Performs SBAS message decoding.
 *
 * This function decoded SBAS message, if the following conditions are met:
 * - Buffer contains 250 bits.
 * - First 8 bits are matching direct or inverse preamble.
 * - Message data CRC matches one in the buffer.
 *
 * In case the message starts with inverted preamble, the data is inverted
 * before parsing.
 *
 * \param[in,out] part Decoder component.
 * \param[out]    msg  Container for a decoded message.
 * \param[out]    delay Delay of the message in symbols.
 *
 * \retval true The message has been decoded, and \a msg container is populated.
 * \retval false Not enough data or CRC is not correct.
 *
 * \private
 */
static bool _sbas_msg_decode(sbas_v27_part_t *part,
                             sbas_msg_t *msg,
                             u32 *delay) {
  bool res = false;
  if (SBAS_MSG_LENGTH <= part->n_decoded) {
    if (part->crc_ok) {
      /* CRC is OK */
      if (part->invert) {
        _sbas_msg_invert(part);
      }

      msg->bit_polarity =
          part->invert ? BIT_POLARITY_INVERTED : BIT_POLARITY_NORMAL;
      msg->prn = 1;
      msg->msg_id = getbitu(part->decoded, 8, 6);
      msg->tow = 0;
      msg->alert = 0;

      switch (msg->msg_id) {
        case 12:
          msg->tow = getbitu(part->decoded, 121, 20); /* Seconds */
          break;
        default:
          /* log_info_sid(construct_sid(CODE_SBAS_L1CA, msg->prn),
                          "Unsupported SBAS message type %d",
                          msg->msg_id); */
          break;
      }

      *delay = (part->n_decoded - SBAS_MSG_LENGTH + SBAS_V27_DELAY_BITS +
                SBAS_V27_CONSTRAINT_LENGTH - 1) *
                   2 +
               part->n_symbols;

      if (part->invert) {
        _sbas_msg_invert(part);
      }
      res = true;
    } else {
      /* CRC mismatch - no decoding */
    }
    bitshl(part->decoded, sizeof(part->decoded), SBAS_MSG_LENGTH);
    part->n_decoded -= SBAS_MSG_LENGTH;
  }

  return res;
}

/**
 * Initialize SBAS decoder.
 *
 * SBAS decoder contains two Viterbi decoders that are used to estimate bit and
 * message boundary.
 *
 * \param[out] dec Decoder structure.
 *
 * \return None
 */
void sbas_msg_decoder_init(sbas_msg_decoder_t *dec) {
  memset(dec, 0, sizeof(*dec));
  v27_init(&dec->part1.dec,
           dec->part1.decisions,
           SBAS_V27_HISTORY_LENGTH_BITS,
           sbas_msg_decoder_get_poly(),
           0);
  v27_init(&dec->part2.dec,
           dec->part2.decisions,
           SBAS_V27_HISTORY_LENGTH_BITS,
           sbas_msg_decoder_get_poly(),
           0);
  dec->part1.init = true;
  dec->part2.init = true;
  _sbas_add_symbol(&dec->part2, 0x80);
}
/**
 * Adds a received symbol to decoder.
 *
 * The method feeds the symbol into the decoder. In case there is a sufficient
 * information to produce a message, the message is decoded and symbol delay is
 * reported.
 * The time of the last input symbol can be computed from the message ToW and
 * delay by the formulae:
 * \code
 * symbolTime_ms = msg->tow * SECS_MS + *pdelay * SBAS_L1CA_SYMBOL_LENGTH_MS
 * \endcode
 *
 * \param[in,out] dec    Decoder object.
 * \param[in]     symbol Symbol value probability, where 0x00 - 100% of 0,
 *                       0xFF - 100% of 1.
 * \param[out]    msg    Buffer for decoded message. The message is available
 *                       only when message lock is acquired and CRC is correct.
 * \param[out]    pdelay Delay of message generation in symbols.
 *
 * \retval true  The message has been decoded. ToW parameter is available.
 * \retval false More data is required.
 */
bool sbas_msg_decoder_add_symbol(sbas_msg_decoder_t *dec,
                                 u8 symbol,
                                 sbas_msg_t *msg,
                                 u32 *pdelay) {
  _sbas_add_symbol(&dec->part1, symbol);
  _sbas_add_symbol(&dec->part2, symbol);

  if (dec->part1.message_lock) {
    /* Flush data in decoder. */
    dec->part2.n_decoded = 0;
    dec->part2.n_symbols = 0;
    return _sbas_msg_decode(&dec->part1, msg, pdelay);
  }
  if (dec->part2.message_lock) {
    /* Flush data in decoder. */
    dec->part1.n_decoded = 0;
    dec->part1.n_symbols = 0;
    return _sbas_msg_decode(&dec->part2, msg, pdelay);
  }

  return false;
}

/**
 * Provides a singleton polynomial object.
 *
 * The method constructs and returns polynomial object for SBAS message
 * decoding. The same polynomial can be used also for other message handling.
 *
 * The object is initialized on the first call. The method is thread-safe.
 *
 * @return Pointer to polynomial object for SBAS message decoding.
 */
const v27_poly_t *sbas_msg_decoder_get_poly(void) {
  static v27_poly_t instance;
  static bool initialized = false;

  if (!initialized) {
    /* Coefficients for polynomial object */
    const signed char coeffs[2] = {SBAS_V27_POLY_A, SBAS_V27_POLY_B};

    /* Racing condition handling: the data can be potential initialized more
     * than once in case multiple threads request concurrent access. However,
     * nature of the v27_poly_init() function and data alignment ensure that
     * the data returned from the earlier finished call is consistent and can
     * be used even when re-initialization is happening.
     *
     * Other possible approaches are:
     * - Replace late initialization with an explicit call.
     * - Use POSIX synchronization objects like pthread_once_t.
     */
    v27_poly_init(&instance, coeffs);
    initialized = true;
  }
  return &instance;
}
