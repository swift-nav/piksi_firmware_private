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
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <libswiftnav/bits.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/ionosphere.h>
#include <libswiftnav/l2c_capability.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/shm.h>

#include "nav_msg/nav_msg.h"

/** Bad parity subframe mask for subframes 3-10 */
#define GPS_LNAV_BAD_DATA_SFRAME_MASK UINT8_C(0xFF)
/* Detection of bad control or invalid data sequence.
 * We expect a sequence of 01 or 10 bit pairs for words 3-10 with the exception
 * that the last two bits in work 10 can be 00 or 11, and we don't check them */
/** 30 bit mask for words 3-9 when detecting 1/0 sequence */
#define GPS_LNAV_BAD_DATA_MASK1 UINT32_C(0x3FFFFFFF)
/** 28 bit mask for word 10 when detecting 1/0 sequence */
#define GPS_LNAV_BAD_DATA_MASK2 UINT32_C(0x3FFFFFFC)
/** Sequence of 15 0/1 bit pairs (30 bits) */
#define GPS_LNAV_ZERO_ONE_BITS1 UINT32_C(0x15555555)
/** Sequence of 15 1/0 bit pairs (30 bits) */
#define GPS_LNAV_ONE_ZERO_BITS1 UINT32_C(0x2AAAAAAA)
/** Sequence of 14 0/1 bit pairs (28 bits MSB) */
#define GPS_LNAV_ZERO_ONE_BITS2 UINT32_C(0x15555554)
/** Sequence of 14 1/0 bit pairs (28 bits MSB) */
#define GPS_LNAV_ONE_ZERO_BITS2 UINT32_C(0x2AAAAAA8)
/** Total number of words matching 01 or 10 bit pattern */
#define GPS_LNAV_BAD_DATA_WORDS 8

/**
 * Subframe data check status
 */
typedef enum {
  GPS_LNAV_SF_STATUS_OK,
  GPS_LNAV_SF_STATUS_ERROR,
  GPS_LNAV_SF_STATUS_INVALID,
} gps_lnav_sf_status_t;

static u8 nav_parity(u32 *word);

/**
 * Initializes GPS LNAV message decoder.
 *
 * Initializes GPS LNAV message decoder for PRNs 1-32.
 *
 * \param n GPS LNAV message decoder object
 */
void nav_msg_init(nav_msg_t *n) {
  /* Initialize the necessary parts of the nav message state structure. */
  memset(n, 0, sizeof(*n));
  n->next_subframe_id = 0; /* The very first subframe number is not known  */
  n->bit_polarity = BIT_POLARITY_UNKNOWN;
  for (unsigned i = 0; i < GPS_LNAV_SUBFRAME_CNT; ++i) {
    n->frame_age[i] = GPS_LNAV_SUBFRAME_AGE_INVALID;
  }
}

void nav_msg_clear_decoded(nav_msg_t *n) {
  memset(n->frame_words, 0, sizeof(n->frame_words));
  for (u8 i = 0; i < GPS_LNAV_SUBFRAME_CNT; ++i) {
    n->frame_age[i] = GPS_LNAV_SUBFRAME_AGE_INVALID;
  }
}

/**
 * Extract bits from a buffered subframe.
 *
 * \param[in] n         Decoder object
 * \param[in] bit_index First bit index.
 * \param[in] n_bits    Total number of bits to extract (<=32)
 * \param[in] invert    Flag if the data to be inverted
 *
 * \return Extracted bits
 */
static u32 extract_word(nav_msg_t *n, u16 bit_index, u8 n_bits, u8 invert) {
  assert(n_bits > 0 && n_bits <= 32);

  /* Extract a word of n_bits length (n_bits <= 32) at position bit_index into
   * the subframe. Takes account of the offset stored in n, and the circular
   * nature of the n->subframe_bits buffer. */

  /* Offset for the start of the subframe in the buffer. */
  if (n->subframe_start_index > 0) {
    bit_index += n->subframe_start_index; /* Standard. */
    bit_index--;
  } else if (n->subframe_start_index < 0) {
    bit_index -= n->subframe_start_index; /* Bits are inverse! */
    bit_index--;
    invert = !invert;
  }

  /* Wrap if necessary. */
  if (bit_index >= NAV_MSG_SUBFRAME_BITS_LEN) {
    bit_index -= NAV_MSG_SUBFRAME_BITS_LEN;
  }

  u8 bix_hi = bit_index >> 5;
  u8 bix_lo = bit_index & 0x1F;
  u32 word = n->subframe_bits[bix_hi] << bix_lo;

  if (0 != bix_lo) {
    if (++bix_hi == NAV_MSG_SUBFRAME_WORDS_LEN) {
      bix_hi = 0;
    }
    word |= n->subframe_bits[bix_hi] >> (32 - bix_lo);
  }

  if (invert) {
    word = ~word;
  }

  return word >> (32 - n_bits);
}

/** Navigation message decoding update.
 * Called once per nav bit interval. Performs the necessary steps to
 * store the nav bits and decode them.
 *
 * Also extracts and returns the GPS time of week each time a new subframe is
 * received.
 *
 * \param n Nav message decode state struct
 * \param bit_val State of the nav bit to process
 *
 * \return The GPS time of week in milliseconds of the current code phase
 *         rollover, or `TOW_INVALID` (-1) if unknown
 */
s32 nav_msg_update(nav_msg_t *n, bool bit_val) {
  s32 TOW_ms = TOW_INVALID;

  /* The following is offset by 27 to allow the handover word to be
   * overwritten.  This is not a problem as it's handled below and
   * isn't needed by the ephemeris decoder.
   */
  u16 last_subframe_bit_index = ABS(n->subframe_start_index) + 27;
  last_subframe_bit_index %= NAV_MSG_SUBFRAME_BITS_LEN;
  if (n->subframe_start_index &&
      (n->subframe_bit_index == last_subframe_bit_index)) {
    /* Subframe buffer is full: the nav message decoder has missed it's
     * deadline.  Clobbering the buffer can result in invalid nav data
     * being used.
     */
    n->overrun = true;
    return -2;
  }

  if (n->subframe_bit_index >= NAV_MSG_SUBFRAME_BITS_LEN) {
    log_error("subframe bit index gone wild %d", (int)n->subframe_bit_index);
    return -22;
  }

  if (bit_val) {
    n->subframe_bits[n->subframe_bit_index >> 5] |=
        1u << (31 - (n->subframe_bit_index & 0x1F));
  } else {
    /* Integrated correlation is negative, so bit is 0. */
    n->subframe_bits[n->subframe_bit_index >> 5] &=
        ~(1u << (31 - (n->subframe_bit_index & 0x1F)));
  }

  n->subframe_bit_index++;
  if (n->subframe_bit_index == NAV_MSG_SUBFRAME_BITS_LEN) {
    n->subframe_bit_index = 0;
  }

  /* Yo dawg, are we still looking for the preamble? */
  if (!n->subframe_start_index) {
/* We're going to look for the preamble at a time 360 nav bits ago,
 * then again 60 nav bits ago. */
#define SUBFRAME_START_BUFFER_OFFSET (NAV_MSG_SUBFRAME_BITS_LEN - 360)

    /* Check whether there's a preamble at the start of the circular
     * subframe_bits buffer. */
    u8 preamble_candidate = extract_word(
        n, n->subframe_bit_index + SUBFRAME_START_BUFFER_OFFSET, 8, 0);

    if (preamble_candidate == 0x8B) {
      n->subframe_start_index =
          n->subframe_bit_index + SUBFRAME_START_BUFFER_OFFSET + 1;
    } else if (preamble_candidate == 0x74) {
      n->subframe_start_index =
          -(n->subframe_bit_index + SUBFRAME_START_BUFFER_OFFSET + 1);
    }

    if (n->subframe_start_index) {
      // Looks like we found a preamble, but let's confirm.
      if (extract_word(n, 300, 8, 0) == 0x8B) {
        // There's another preamble in the following subframe.  Looks good so
        // far.
        // Extract the TOW:
        unsigned int TOW_trunc =
            extract_word(n, 30, 17, extract_word(n, 29, 1, 0));
        /* (bit 29 is D30* for the second word, where the TOW resides) */
        if (TOW_trunc < 7 * 24 * 60 * 10) {
          /* TOW in valid range */
          TOW_trunc++;  // Increment it, to see what we expect at the start of
                        // the next subframe
          if (TOW_trunc == 7 * 24 * 60 * 10) {  // Handle end of week rollover
            TOW_trunc = 0;
          }

          if (TOW_trunc ==
              extract_word(n, 330, 17, extract_word(n, 329, 1, 0))) {
            /* Check parity */
            /* Extract word 2 and the last two parity bits of word 1
               of the full subframe in our buffer. */
            u32 sf_word2 = extract_word(n, 28, 32, 0);
            if (nav_parity(&sf_word2)) {
              n->subframe_start_index = 0;
              return TOW_INVALID;
            }
            /* Extract word 2 and the last two parity bits of word 1
               of the partial subframe in our buffer. */
            sf_word2 = extract_word(n, 300 + 28, 32, 0);
            if (nav_parity(&sf_word2)) {
              n->subframe_start_index = 0;
              return TOW_INVALID;
            }
            // We got two appropriately spaced preambles, and two matching TOW
            // counts.  Pretty certain now.
            // The TOW in the message is for the start of the NEXT subframe.
            // That is, 240 nav bits' time from now, since we are 60 nav bits
            // into the second subframe that we recorded.
            if (TOW_trunc == 0) {
              /* end-of-week special case */
              TOW_ms = WEEK_MS - (300 - 60) * 20;
            } else {
              TOW_ms = TOW_trunc * 6000 - (300 - 60) * 20;
            }
            n->bit_polarity = (n->subframe_start_index > 0)
                                  ? BIT_POLARITY_NORMAL
                                  : BIT_POLARITY_INVERTED;
          }
        }
      }
      /* If we didn't find a matching pair of preambles + TOWs,
       * this offset can't be right. Move on. */
      if (TOW_ms < 0) {
        n->subframe_start_index = 0;
      }
    }
  }

  return TOW_ms;
}

/* Tests the parity of a L1 C/A NAV message word.
 * Inverts the data bits if necessary, and checks the parity.
 * Expects a word where MSB = D29*, bit 30 = D30*, bit 29 = D1, ... LSB = D30.
 *
 * \note This function may modify the value of `word`.
 *
 * References:
 *   -# ICD-GPS-200E Table 20-XIV
 *
 * \param word Pointer to word to check. Note, if D30* is set then the data
 *             bits in this word will be inverted in place.
 * \return 0 if the parity is correct,
 *         otherwise returns the number of the first incorrect parity bit.
 */
static u8 nav_parity(u32 *word) {
  if (*word & 1u << 30) { /* Inspect D30* */
    *word ^= 0x3FFFFFC0;  /* D30* = 1, invert all the data bits! */
  }

  /* Check D25 */
  if (parity(*word & 0xBB1F34A0 /* 0b10111011000111110011010010100000 */)) {
    return 25;
  }
  /* Check D26 */
  if (parity(*word & 0x5D8F9A50 /* 0b01011101100011111001101001010000 */)) {
    return 26;
  }
  /* Check D27 */
  if (parity(*word & 0xAEC7CD08 /* 0b10101110110001111100110100001000 */)) {
    return 27;
  }
  /* Check D28 */
  if (parity(*word & 0x5763E684 /* 0b01010111011000111110011010000100 */)) {
    return 28;
  }
  /* Check D29 */
  if (parity(*word & 0x6BB1F342 /* 0b01101011101100011111001101000010 */)) {
    return 29;
  }
  /* Check D30 */
  if (parity(*word & 0x8B7A89C1 /* 0b10001011011110101000100111000001 */)) {
    return 30;
  }

  return 0;
}

bool subframe_ready(nav_msg_t *n) { return (n->subframe_start_index != 0); }

/**
 * Internal helper for increase cached subframe data age.
 *
 * The method increases the age of cached subframe entries by one until it
 * exceeds #GPS_LNAV_SUBFRAME_AGE_MAX. In the latter case, the entry age is set
 * to #GPS_LNAV_SUBFRAME_AGE_INVALID.
 *
 * \param[in,out] n     GPS LNAV message decoder data
 *
 * \return None
 */
static void age_subframe_data(nav_msg_t *n) {
  for (unsigned sf = 0; sf < GPS_LNAV_SUBFRAME_CNT; ++sf) {
    if (n->frame_age[sf] < GPS_LNAV_SUBFRAME_AGE_MAX) {
      /* Increase age as it is under the limit */
      n->frame_age[sf]++;
    } else {
      /* Mark subframe as too old. */
      n->frame_age[sf] = GPS_LNAV_SUBFRAME_AGE_INVALID;
    }
  }
}

/**
 * Updates subframe counter and subframe ages.
 *
 * \param[in,out] n GPS LNAV message decoder data
 *
 * \return None
 */
static void prepare_for_next_subframe(nav_msg_t *n) {
  /* Mark the subframe as processed */
  n->subframe_start_index = 0;

  /* Compute the next subframe number if known */
  if (0 != n->next_subframe_id &&
      ++n->next_subframe_id > GPS_LNAV_SUBFRAME_MAX) {
    n->next_subframe_id = GPS_LNAV_SUBFRAME_MIN;

    /* Age all subframe entries */
    age_subframe_data(n);
  }
}

/**
 * Verifies integrity of received data and moves it into decoding buffer.
 *
 * This method is called after all subframe data is received and can be used
 * for decoding. If all the subframe words are valid, the data is copied into
 * decoding buffer and age is set to 0.
 *
 * \param[in,out] n     GPS LNAV message decoder data
 * \param[in]     mesid ME signal identifier
 * \param[in]     sf_id GNSS subframe identifier [1..5]
 *
 * \retval GPS_LNAV_SF_STATUS_OK      Subframe data seems to be valid
 * \retval GPS_LNAV_SF_STATUS_ERROR   Data can't be recovered
 * \retval GPS_LNAV_SF_STATUS_INVALID Invalid control or data element sequence
 *                                    detected
 */
static gps_lnav_sf_status_t fill_in_sf_data(nav_msg_t *n,
                                            const me_gnss_signal_t mesid,
                                            u8 sf_id) {
  gps_lnav_sf_status_t res = GPS_LNAV_SF_STATUS_OK;

  if (sf_id >= GPS_LNAV_SUBFRAME_MIN && sf_id <= GPS_LNAV_SUBFRAME_MAX) {
    u8 sf_idx = sf_id - GPS_LNAV_SUBFRAME_MIN;

    u8 subframe_invalid_mask = 0;
    u32 subframe_words[8];
    for (u32 w = 0; w < 8; w++) { /* For words 3..10 */
      /* Get the bits */
      u32 tmp;
      subframe_words[w] = tmp = extract_word(n, 30 * (w + 2) - 2, 32, 0);
      /* MSBs are D29* and D30*.  LSBs are D1...D30 */
      if (0 != nav_parity(&subframe_words[w])) {
        subframe_invalid_mask |= UINT8_C(1) << (7 - w);
        subframe_words[w] = tmp;
      }
    }
    if (0 != subframe_invalid_mask) {
      /* Check parity and invert bits if D30* */
      log_info_mesid(mesid,
                     "subframe %" PRIu8
                     " parity mismatch "
                     "(3-10 error words mask: 0x%02" PRIX8 ")",
                     sf_id,
                     subframe_invalid_mask);
      res = GPS_LNAV_SF_STATUS_ERROR;

      if (GPS_LNAV_BAD_DATA_SFRAME_MASK == subframe_invalid_mask) {
        /* Subframes 3-10 are invalid... Check for 1/0 pattern */
        u32 count = 0;
        u32 bits;
        for (u32 w = 0; w < 7; w++) {
          bits = subframe_words[w] & GPS_LNAV_BAD_DATA_MASK1;
          if (GPS_LNAV_ZERO_ONE_BITS1 == bits ||
              GPS_LNAV_ONE_ZERO_BITS1 == bits) {
            count++;
          }
        }
        bits = subframe_words[7] & GPS_LNAV_BAD_DATA_MASK2;
        if (GPS_LNAV_ZERO_ONE_BITS2 == bits ||
            GPS_LNAV_ONE_ZERO_BITS2 == bits) {
          count++;
        }

        if (GPS_LNAV_BAD_DATA_WORDS == count) {
          log_debug_mesid(mesid, "invalid control or data element detected");
          res = GPS_LNAV_SF_STATUS_INVALID;
        }
      }
    } else {
      /* Replace cached data with updated one, and set age to 0 */
      memcpy(n->frame_words[sf_idx], subframe_words, sizeof(subframe_words));
      n->frame_age[sf_idx] = 0;
      log_debug_mesid(mesid, "subframe %" PRIu8 " parity OK", sf_id);
    }
  } else {
    res = GPS_LNAV_SF_STATUS_ERROR;
  }

  return res;
}

/**
 * Helper method to check if the ephemeris can be constructed from a cached
 * data.
 *
 * The method verifies that there are cached entries for subframes 1, 2 and 3,
 * and they have the same IOD.
 *
 * \param[in] n     GPS LNAV message decoder data
 * \param[in] mesid ME signal identifier
 *
 * \retval true  Cached entries belong to the same IOD.
 * \retval false Cached entries belong to different IOD, or data is incomplete.
 */
static bool can_combine_subframes123(const nav_msg_t *n,
                                     const me_gnss_signal_t mesid) {
  bool ok = true;
  bool age0 = false;
  bool age1p = false;

  for (u32 sf = 0; sf < 3; ++sf) {
    if (n->frame_age[sf] > GPS_LNAV_SUBFRAME_AGE_MAX) {
      ok = false;
      log_debug_mesid(
          mesid, "Missing subframe %" PRIu32 " for ephemeris decoding", sf + 1);
      if (!DEBUG) {
        break;
      }
    } else if (0 == n->frame_age[sf]) {
      age0 = true;
    } else {
      age1p = true;
    }
  }

  if (ok) {
    if (!age0) {
      /* No new data received. Do not recombine */
      ok = false;
      log_debug_mesid(mesid, "No new data to decode");
    } else if (!age1p) {
      /* All three subframes are new. Can combine, no further actions */
      log_debug_mesid(mesid, "New ephemeris data to decode");
    } else {
      /* Some new and old subframe combinations, log info */
      u8 iode_sf1 = n->frame_words[1 - 1][8 - 3] >> (30 - 8) & 0xFF;
      u8 iode_sf2 = n->frame_words[2 - 1][3 - 3] >> (30 - 8) & 0xFF;
      u8 iode_sf3 = n->frame_words[3 - 1][10 - 3] >> (30 - 8) & 0xFF;

      if (iode_sf1 == iode_sf2 && iode_sf2 == iode_sf3) {
        /* IODE matches, can combine */
        log_debug_mesid(mesid,
                        "Combining subframes 1-3:"
                        " AGE=%" PRIu8 "/%" PRIu8 "/%" PRIu8 " IODE=%" PRIu8,
                        n->frame_age[0],
                        n->frame_age[1],
                        n->frame_age[2],
                        iode_sf1);
      } else {
        /* IODE mismatch, can't combine */
        ok = false;
        log_debug_mesid(mesid,
                        "Can't combine subframes 1-3:"
                        " AGE=%" PRIu8 "/%" PRIu8 "/%" PRIu8 " IODE=%" PRIu8
                        "/%" PRIu8 "/%" PRIu8,
                        n->frame_age[0],
                        n->frame_age[1],
                        n->frame_age[2],
                        iode_sf1,
                        iode_sf2,
                        iode_sf3);
      }
    }
  } else {
    /* Not OK */
  }

  return ok;
}

/**
 * Decodes ephemeris if sufficient information is available.
 *
 * \param[in]  n        GPS LNAV message decoder data
 * \param[in]  mesid    ME signal identifier
 * \param[in]  sf_word2 Word 2 contents from the current subframe
 * \param[out] data     Destination container for ephemeris data
 *
 * \retval 1   Ephemeris has been decoded.
 * \retval -1  Ephemeris has not been decoded.
 *
 * \sa can_combine_subframes123
 */
static s8 try_ephemeris_decoding(const nav_msg_t *n,
                                 const me_gnss_signal_t mesid,
                                 u32 sf_word2,
                                 gps_l1ca_decoded_data_t *data) {
  s8 res = -1;
  if (can_combine_subframes123(n, mesid)) {
    /*
     * HOW contains TOW for the _next_ subframe transmission start
     */
    s32 tow_6s = sf_word2 >> (30 - 17) & 0x1ffff;
    s32 tot_tow = TOW_UNKNOWN; /* ToW in seconds */
    if (0 == tow_6s) {
      /* Week rollover after the current subframe */
      tot_tow = WEEK_SECS - 6;
    } else if (tow_6s < WEEK_SECS / 6) {
      /* ToW is in valid range per ICD, compute ToW in seconds for the current
       * subframe */
      tot_tow = (tow_6s - 1) * 6;
    } else {
      log_info_mesid(mesid, "Invalid ToW for ephemeris");
    }

    if (TOW_UNKNOWN != tot_tow) {
      /* Now let's actually decode the ephemeris... */
      data->ephemeris.sid = mesid2sid(mesid, GLO_ORBIT_SLOT_UNKNOWN);
      decode_ephemeris(n->frame_words, &data->ephemeris, tot_tow);
      data->ephemeris_upd_flag = true;
      res = 1;
    }
  }

  return res;
}

/**
 * Decodes data after subframe 1 arrival.
 *
 * \param[in]  n        GPS LNAV message decoder data
 * \param[in]  mesid    ME signal identifier
 * \param[in]  sf_word2 Word 2 contents from the current subframe
 * \param[out] data     Destination container for ephemeris data
 *
 * \retval 1   Ephemeris or SHM data have been decoded.
 * \retval -1  No data have been decoded.
 */
static s8 decode_subframe1(const nav_msg_t *n,
                           const me_gnss_signal_t mesid,
                           u32 sf_word2,
                           gps_l1ca_decoded_data_t *data) {
  s8 res = -1;

  res = try_ephemeris_decoding(n, mesid, sf_word2, data);

  if (0 == n->frame_age[0]) {
    /* Decode health information as soon as we get it and even if other
     * subframes forming the ephemeris won't be received */
    shm_gps_decode_shi1(n->frame_words[0][3 - 3], &data->shi1);
    data->shi1_upd_flag = true;
    res = 1;
  }

  return res;
}

/**
 * Decodes data after subframes 2 and 3 arrival.
 *
 * \param[in,out] n        GPS LNAV message decoder data
 * \param[in]     mesid    ME signal identifier
 * \param[in]     sf_word2 Word 2 contents from the current subframe
 * \param[out]    data     Destination container for ephemeris data
 *
 * \retval 1   Ephemeris data have been decoded.
 * \retval -1  No data have been decoded.
 */
static s8 decode_subframe23(const nav_msg_t *n,
                            const me_gnss_signal_t mesid,
                            u32 sf_word2,
                            gps_l1ca_decoded_data_t *data) {
  return try_ephemeris_decoding(n, mesid, sf_word2, data);
}

/**
 * Decodes data after subframe 4 arrival.
 *
 * The method decodes almanacs for PRNs 1 to 32, almanac week reference,
 * extra SV health information, iono parameters, L2C capabilities.
 *
 * \param[in]  age   Age of subframe data.
 * \param[in]  words Subframe words 3-10.
 * \param[out] data  Destination container decoded data.
 *
 * \retval 1   Some data have been decoded.
 * \retval -1  No data have been decoded.
 */
static s8 decode_subframe45(u8 age,
                            const u32 words[8],
                            gps_l1ca_decoded_data_t *data) {
  s8 res = -1;
  u8 data_id = words[3 - 3] >> (30 - 2) & 0x3;

  /* Check Word 3 bits 0..1 (61..62) for Data ID, see IS-200H, pg. 109 */
  /* Data ID for GPS block II and III is "01" which is the only currently
   * used. ID "00" is for the long obsolete block I */
  if (0 == age && GPS_LNAV_ALM_DATA_ID_BLOCK_II == data_id) {
    /* Check Word 3 bits 2..7 (63..69) for SV ID, see IS-200H, pg. 84 */
    u8 sv_id = words[3 - 3] >> (30 - 8) & 0x3f;

    /* IS-GPS-200H / 20.3.3.5.1 Content of Subframes 4 and 5.
     * Subframe 4: pages 2, 3, 4, 5, 7, 8, 9 and 10 contain almanac data for
     * SV 25 through 32 respectively.
     *
     * So, compute SV ID if page is within the range.
     */
    if (sv_id >= GPS_LNAV_ALM_MIN_PRN && sv_id <= GPS_LNAV_ALM_MAX_PRN) {
      almanac_decode(words, &data->almanac);
      data->almanac_upd_flag = data->almanac.valid;
      res = 1;
    }

    /* Get Words 3-8 from 25th page (SV config bits)
     * Page 25 has SV ID 63, see IS-200H, pg. 109-110 */
    if (GPS_LNAV_ALM_SVID_CAPABILITIES == sv_id) {
      decode_l2c_capability(words, &data->gps_l2c_sv_capability);
      data->gps_l2c_sv_capability_upd_flag = true;
      res = 1;
    }

    /* Extra SV health info */
    if (GPS_LNAV_ALM_SVID_HEALTH_4 == sv_id ||
        GPS_LNAV_ALM_SVID_HEALTH_5 == sv_id) {
      almanac_health_t health;
      if (almanac_decode_health(words, &health)) {
        data->almanac_health_upd_flags = health.health_bits_valid;
        assert(sizeof(data->almanac_health) == sizeof(health.health_bits));
        memcpy(data->almanac_health,
               health.health_bits,
               sizeof(data->almanac_health));
        res = 1;
      }
    }

    /* Check Word 3 bits 2..7 (63..69) for page 18,
     * which contains ionospheric and UTC data
     * Page 18 has SV ID 56, see IS-200H, pg. 109-110 */
    if (GPS_LNAV_ALM_SVID_IONO == sv_id) {
      /* decode ionospheric correction data */
      data->iono_corr_upd_flag = decode_iono_parameters(words, &data->iono);
      /* decode UTC correction parameters and leap second info */
      data->utc_params_upd_flag = decode_utc_parameters(words, &data->utc);
      res = 1;
    }

    if (GPS_LNAV_ALM_SVID_WEEK == sv_id) {
      almanac_ref_week_t ref_week;

      /* Almanac reference week */
      if (almanac_decode_week(words, &ref_week)) {
        data->almanac_time.tow = ref_week.toa;
        data->almanac_time.wn = ref_week.wna;
        data->almanac_time_upd_flag = true;
        res = 1;
      }
    }
  }

  return res;
}

s8 process_subframe(nav_msg_t *n,
                    const me_gnss_signal_t mesid,
                    gps_l1ca_decoded_data_t *data) {
  assert(data != NULL);
  memset(data, 0, sizeof(*data));

  /* Check parity and parse out the ephemeris from the most recently received
   * subframe */

  /* First things first - check the parity, and invert bits if necessary.
   * process the data, skipping the first word, TLM, and starting with HOW
   */

  /* Complain if buffer overrun */
  if (n->overrun) {
    log_error_mesid(mesid, "nav_msg subframe buffer overrun!");
    n->overrun = false;
  }

  /* Extract word 2, and the last two parity bits of word 1 */
  u32 sf_word2 = extract_word(n, 28, 32, 0);
  if (nav_parity(&sf_word2)) {
    log_info_mesid(mesid,
                   "subframe %" PRIu8 " parity mismatch (word 2)",
                   n->next_subframe_id);
    prepare_for_next_subframe(n);
    return -2;
  }

  /* Alert flag, bit 18 */
  n->alert = sf_word2 >> (30 - 18) & 0x01;
  if (n->alert) {
    log_warn_mesid(mesid, "alert flag set! Ignoring satellite.");
  }

  /* Which of 5 possible subframes is it? bits 20-22 */
  u8 sf_id = sf_word2 >> (30 - 22) & 0x07;

  if (sf_id >= GPS_LNAV_SUBFRAME_MIN && sf_id <= GPS_LNAV_SUBFRAME_MAX) {
    /* SF id is valid */
    if (0 == n->next_subframe_id) {
      /* The very first subframe we found */
      n->next_subframe_id = sf_id;
    } else if (n->next_subframe_id != sf_id) {
      log_info_mesid(mesid,
                     "Expecting sub-frame %" PRIu8 " but got %" PRIu8,
                     n->next_subframe_id,
                     sf_id);
      /* The very first subframe we found */
      n->next_subframe_id = sf_id;
      if (sf_id < n->next_subframe_id) {
        /* Age subframes */
        age_subframe_data(n);
      }
    }
  } else {
    /* SF id is invalid */
    if (0 != n->next_subframe_id) {
      log_info_mesid(mesid,
                     "Invalid subframe ID received; expected %" PRIu8,
                     n->next_subframe_id);
    } else {
      log_info_mesid(mesid, "Invalid subframe ID received");
    }
    prepare_for_next_subframe(n);
    return -4;
  }

  s8 res = -1;

  /* Extract bits and check parity, update data age */
  gps_lnav_sf_status_t sf_status = fill_in_sf_data(n, mesid, sf_id);
  switch (sf_status) {
    case GPS_LNAV_SF_STATUS_OK:
      switch (sf_id) {
        case 1:
          /* Ephemeris and SHM data decoding */
          res = decode_subframe1(n, mesid, sf_word2, data);
          break;

        case 2:
        case 3:
          /* Ephemeris decoding */
          res = decode_subframe23(n, mesid, sf_word2, data);
          break;

        case 4:
          /* Decode almanac, L2C capabilities and iono data. */
          res = decode_subframe45(
              n->frame_age[4 - 1], n->frame_words[4 - 1], data);
          break;

        case 5:
          /* Decode almanac and almanac time data. */
          res = decode_subframe45(
              n->frame_age[5 - 1], n->frame_words[5 - 1], data);
          break;

        default:
          /* Unexpected subframe number. Possible error: invalid subframe ids
           * are
           * handled above. */
          assert(!"Unexpected subframe number");
          break;
      }
      break;
    case GPS_LNAV_SF_STATUS_ERROR:
      /* Unrecoverable parity errors */
      break;
    case GPS_LNAV_SF_STATUS_INVALID:
      /* Detected a sequence of 0/1 in words 3-10 */
      data->invalid_control_or_data = true;
      res = 1;
      break;
    default:
      assert(!"Invalid subframe status flag");
  }

  /* Update subframe counter */
  prepare_for_next_subframe(n);

  return res;
}
