/*
 * Copyright (C) 2017 Swift Navigation Inc.
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
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <libfec/fec.h>
#include <libswiftnav/bits.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/edc.h>
#include <libswiftnav/ionosphere.h>
#include <libswiftnav/logging.h>

#include "main/main.h"
#include "nav_msg/nav_msg.h"
#include "nav_msg/nav_msg_gal.h"
#include "timing/timing.h"

#define GAL_INAV_PREAMBLE_MASK 0x03ff
#define GAL_INAV_PREAMBLE 0x0160
#define GAL_INAV_PREAMBLE_INV 0x029f

#define GAL_INAV_IOD_INVALID (0xFFFF)

static const u8 gal_inav_sync_bits[GAL_INAV_SYNC_BITS] = {
    0x00, 0xFF, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00};
static const u8 gal_inav_sync_bits_inv[GAL_INAV_SYNC_BITS] = {
    0xFF, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static v27_poly_t poly;

#if defined GAL_INAV_FEC_HARD
#define GAL_INAV_WORD_BITMASK 0x3fffffff
static void deint8x30bits(u8 out[static GAL_INAV_INTERL_COL],
                          const u32 in[static GAL_INAV_INTERL_ROW]);
static void inav_buffer_1bit_pushr(u32 buff[static GAL_INAV_DECODE_BUFF_SIZE],
                                   const bool bitval);
#endif /* GAL_INAV_FEC_HARD */
static void deint8x30(u8 out[static GAL_INAV_PAGE_SYMB],
                      const nav_msg_gal_inav_t *nav_msg,
                      u16 offset);
static void inav_buffer_store(nav_msg_gal_inav_t *nav_msg, const u8 val);
static void symb_buffer_copy_saturate(u8 *out,
                                      const nav_msg_gal_inav_t *nav_msg,
                                      const u16 offset,
                                      const u16 size);
static void flip_odd(u8 inout[static GAL_INAV_PAGE_SYMB], const u16 size);
static u32 gal_inav_compute_crc(const u8 even[static GAL_INAV_PAGE_BYTE],
                                const u8 odd[static GAL_INAV_PAGE_BYTE],
                                const int inv);
static u32 gal_inav_extract_crc(const u8 odd[static GAL_INAV_PAGE_BYTE],
                                int inv);
static void extract_inav_content(u8 content[static GAL_INAV_CONTENT_BYTE],
                                 const u8 even[static GAL_INAV_PAGE_BYTE],
                                 const u8 odd[static GAL_INAV_PAGE_BYTE]);
static bool eph_complete(nav_msg_gal_inav_t *nav_msg);
static bool alm_complete(nav_msg_gal_inav_t *nav_msg);

static float sisa_map(u32 sisa);
static void parse_inav_eph(nav_msg_gal_inav_t *nav_msg, gal_inav_decoded_t *dd);

/**
 * Initializes Galileo message decoder.
 *
 * \param[in] n   GAL message decoder object
 * \param[in] prn Galileo PRN id
 */
void gal_inav_msg_init(nav_msg_gal_inav_t *n, u8 prn) {
  const s8 coeffs[2] = {GAL_INAV_V27_POLY_A, GAL_INAV_V27_POLY_B};
  v27_poly_init(&poly, coeffs);

  /* Initialize the necessary parts of the nav message state structure. */
  memset(n, 0, sizeof(*n));
  n->prn = prn;

  n->iod_nav[0] = GAL_INAV_IOD_INVALID;
  n->iod_nav[1] = GAL_INAV_IOD_INVALID;
  n->iod_nav[2] = GAL_INAV_IOD_INVALID;
  n->iod_nav[3] = GAL_INAV_IOD_INVALID;

  n->iod_alm[0] = GAL_INAV_IOD_INVALID;
  n->iod_alm[1] = GAL_INAV_IOD_INVALID;
  n->iod_alm[2] = GAL_INAV_IOD_INVALID;
  n->iod_alm[3] = GAL_INAV_IOD_INVALID;
}

/**
 * Re-initializes Galileo message decoder.
 *
 * \param n Galileo message decoder object
 */
void gal_inav_msg_clear_decoded(nav_msg_gal_inav_t *n) { (void)n; }

/** Navigation message decoding update.
 * Called once per nav bit interval. Performs the necessary steps to
 * store the nav bits and decode them.
 *
 * \param n Nav message decode state struct
 * \param bit_val State of the nav bit to process
 *
 * \return true if a two I/NAV pages completed with this bit
 */
bool gal_inav_msg_update(nav_msg_gal_inav_t *n, s8 bit_val) {
  assert(n);

  /* this implementation uses a SOFT-decision decoder,
   * but right now the dumb 32->8 bit conversion
   * without an AGC generates always a symbol with magnitude 1 */
  u8 symb = bit_val + C_2P7; /* maps -1 into 127 and +1 into 129 */
  inav_buffer_store(n, symb);

  /* once 500 symbols are decoded correctly this function can
   * "sleep" for another 500 */
  if (n->holdoff > 0) {
    n->holdoff--;
    return false;
  }

  u8 preamb0[GAL_INAV_SYNC_BITS];
  u8 preamb1[GAL_INAV_SYNC_BITS];
  /* compare HARD preambles between them */
  u16 offset = 0;
  symb_buffer_copy_saturate(preamb0, n, offset, GAL_INAV_SYNC_BITS);
  offset = GAL_INAV_SYNC_BITS + GAL_INAV_PAGE_SYMB;
  symb_buffer_copy_saturate(preamb1, n, offset, GAL_INAV_SYNC_BITS);
  int preambles_differ = memcmp(preamb0, preamb1, GAL_INAV_SYNC_BITS);
  if (preambles_differ) {
    return false;
  }
  /* compare HARD preambles with Galileo expected ones */
  int differ0 = memcmp(preamb0, gal_inav_sync_bits, GAL_INAV_SYNC_BITS);
  int differ1 = memcmp(preamb0, gal_inav_sync_bits_inv, GAL_INAV_SYNC_BITS);
  if (differ0 && differ1) {
    /* preambles neither match straight nor flipped */
    return false;
  }

  /* at this point we have two candidate preambles
   * and need to check data content */
  u8 candidate_symbols[GAL_INAV_PAGE_SYMB]; /* 240 symbols of candidate half
                                               page */

  /* de-interleave the first 240 bits */
  deint8x30(candidate_symbols, n, GAL_INAV_SYNC_BITS);
  /* work around NOT port in Galileo interleaver */
  flip_odd(candidate_symbols, GAL_INAV_PAGE_SYMB);

  u8 candidate_even_bits[15]; /* 120 bits of candidate EVEN half page */
  v27_init(&(n->decoder), n->decisions, GAL_INAV_V27_HISTORY_LENGTH, &poly, 0);
  v27_update(&(n->decoder), candidate_symbols, GAL_INAV_PAGE_BIT);
  v27_chainback_fixed(&(n->decoder),
                      candidate_even_bits,
                      GAL_INAV_PAGE_BIT - GAL_INAV_TAIL_BIT,
                      0x0);

  /* check if the first is an even page */
  u8 even_flag = (candidate_even_bits[0] >> 7) & 0x1;
  if (0 != even_flag) {
    return false;
  }

  /* de-interleave the second 240 bits */
  deint8x30(candidate_symbols,
            n,
            GAL_INAV_SYNC_BITS + GAL_INAV_PAGE_SYMB + GAL_INAV_SYNC_BITS);
  /* work around NOT port in Galileo interleaver */
  flip_odd(candidate_symbols, GAL_INAV_PAGE_SYMB);

  u8 candidate_odd_bits[15]; /* 120 bits of candidate ODD half page */
  v27_init(&(n->decoder), n->decisions, GAL_INAV_V27_HISTORY_LENGTH, &poly, 0);
  v27_update(&(n->decoder), candidate_symbols, GAL_INAV_PAGE_BIT);
  v27_chainback_fixed(&(n->decoder),
                      candidate_odd_bits,
                      GAL_INAV_PAGE_BIT - GAL_INAV_TAIL_BIT,
                      0x0);

  u8 odd_flag = (candidate_odd_bits[0] >> 7) & 0x1;
  /* check if the second is an odd page */
  if (1 != odd_flag) {
    return false;
  }

  u32 crc_calc =
      gal_inav_compute_crc(candidate_even_bits, candidate_odd_bits, differ0);
  u32 crc_extract = gal_inav_extract_crc(candidate_odd_bits, differ0);

  bool crc_ok = (crc_calc == crc_extract);
  if (!crc_ok) {
    log_info("E%02d crc_calc %08" PRIx32 " crc_extract %08" PRIx32,
             n->prn,
             crc_calc,
             crc_extract);
    return false;
  }

  /* at this point CRC matches: we have a good word */
  n->holdoff = 2 * (GAL_INAV_SYNC_BITS + GAL_INAV_PAGE_SYMB) - 1;

  /*char str[128];
  char bytestr[4];
  memset(str, 0, 128);
  for (u8 i=0; i<15; i++) {
    sprintf(bytestr, "%02x", candidate_even_bits[i]);
    strcat(str, bytestr);
  }
  strcat(str, " ");
  for (u8 i=0; i<15; i++) {
    sprintf(bytestr, "%02x", candidate_odd_bits[i]);
    strcat(str, bytestr);
  }
  log_info("E%02d %s", n->prn, str);
   */

  bool alert =
      getbitu(candidate_even_bits, 1, 1) || getbitu(candidate_odd_bits, 1, 1);
  if (alert) {
    /* do not process an alert page further */
    DO_EACH_MS(10 * SECS_MS,
               log_info("E%02" PRIu8 " received an alert page", n->prn););
    return false;
  }

  extract_inav_content(n->raw_content, candidate_even_bits, candidate_odd_bits);

  //  inav_content_type type = parse_inav_word(n, content);
  //  if (INAV_INCOMPLETE != type) {
  //    return true;
  //  }
  return true;
}

static void parse_inav_eph(nav_msg_gal_inav_t *nav_msg,
                           gal_inav_decoded_t *dd) {
  ephemeris_t *eph = &(dd->ephemeris);
  ephemeris_kepler_t *kep = &(dd->ephemeris.kepler);
  /* word type 1 */
  u32 toe = getbitu(nav_msg->raw_eph[0], 16, 14);
  eph->toe.tow = toe * 60.0;
  u32 m0 = getbitu(nav_msg->raw_eph[0], 30, 32);
  kep->m0 = BITS_SIGN_EXTEND_32(32, m0) * C_1_2P31 * GPS_PI;
  u32 ecc = getbitu(nav_msg->raw_eph[0], 62, 32);
  kep->ecc = ecc * C_1_2P33;
  u32 sqrta = getbitu(nav_msg->raw_eph[0], 94, 32);
  kep->sqrta = sqrta * C_1_2P19;
  /* word type 2 */
  u32 omega0 = getbitu(nav_msg->raw_eph[1], 0, 32);
  kep->omega0 = BITS_SIGN_EXTEND_32(32, omega0) * C_1_2P31 * GPS_PI;
  u32 i0 = getbitu(nav_msg->raw_eph[1], 32, 32);
  kep->inc = BITS_SIGN_EXTEND_32(32, i0) * C_1_2P31 * GPS_PI;
  u32 omega = getbitu(nav_msg->raw_eph[1], 64, 32);
  kep->w = BITS_SIGN_EXTEND_32(32, omega) * C_1_2P31 * GPS_PI;
  u32 idot = getbitu(nav_msg->raw_eph[1], 96, 14);
  kep->inc_dot = BITS_SIGN_EXTEND_32(14, idot) * C_1_2P43 * GPS_PI;
  /* word type 3 */
  u32 omegadot = getbitu(nav_msg->raw_eph[2], 0, 24);
  kep->omegadot = BITS_SIGN_EXTEND_32(24, omegadot) * C_1_2P43 * GPS_PI;
  u32 deltan = getbitu(nav_msg->raw_eph[2], 24, 16);
  kep->dn = BITS_SIGN_EXTEND_32(16, deltan) * C_1_2P43 * GPS_PI;
  u32 cuc = getbitu(nav_msg->raw_eph[2], 40, 16);
  kep->cuc = BITS_SIGN_EXTEND_32(16, cuc) * C_1_2P29;
  u32 cus = getbitu(nav_msg->raw_eph[2], 56, 16);
  kep->cus = BITS_SIGN_EXTEND_32(16, cus) * C_1_2P29;
  u32 crc = getbitu(nav_msg->raw_eph[2], 72, 16);
  kep->crc = BITS_SIGN_EXTEND_32(16, crc) * C_1_2P5;
  u32 crs = getbitu(nav_msg->raw_eph[2], 88, 16);
  kep->crs = BITS_SIGN_EXTEND_32(16, crs) * C_1_2P5;
  u32 sisa = getbitu(nav_msg->raw_eph[2], 104, 8);
  eph->ura = sisa_map(sisa);
  /* word type 4 */
  u32 sat = getbitu(nav_msg->raw_eph[3], 0, 6);
  eph->sid.sat = sat;
  u32 cic = getbitu(nav_msg->raw_eph[3], 6, 16);
  kep->cic = BITS_SIGN_EXTEND_32(16, cic) * C_1_2P29;
  u32 cis = getbitu(nav_msg->raw_eph[3], 22, 16);
  kep->cis = BITS_SIGN_EXTEND_32(16, cis) * C_1_2P29;
  u32 toc = getbitu(nav_msg->raw_eph[3], 38, 14);
  kep->toc.tow = toc * 60.0;
  u32 af0 = getbitu(nav_msg->raw_eph[3], 52, 31);
  kep->af0 = BITS_SIGN_EXTEND_32(31, af0) * C_1_2P34;
  u32 af1 = getbitu(nav_msg->raw_eph[3], 83, 21);
  kep->af1 = BITS_SIGN_EXTEND_32(21, af1) * C_1_2P46;
  u32 af2 = getbitu(nav_msg->raw_eph[3], 104, 6);
  kep->af2 = BITS_SIGN_EXTEND_32(6, af2) * C_1_2P59;
}

inav_content_type parse_inav_word(nav_msg_gal_inav_t *nav_msg,
                                  gal_inav_decoded_t *dd) {
  assert(nav_msg);
  u8 *content = nav_msg->raw_content;

  u8 word_type = getbitu(content, 0, 6);
  if (0 == word_type) {
    u32 tflag = getbitu(content, 6, 2);
    if (tflag != 0b10) return INAV_INCOMPLETE;
    return INAV_TOWONLY;
  }

  if (1 == word_type) {
    nav_msg->iod_nav[0] = getbitu(content, 6, 10);
    memcpy(nav_msg->raw_eph[0], content, GAL_INAV_CONTENT_BYTE);
    if (!eph_complete(nav_msg)) {
      return INAV_INCOMPLETE;
    }
    parse_inav_eph(nav_msg, dd);
    return INAV_EPH;
  }

  if (2 == word_type) {
    memcpy(nav_msg->raw_eph[1], content, GAL_INAV_CONTENT_BYTE);
    if (!eph_complete(nav_msg)) {
      return INAV_INCOMPLETE;
    }
    parse_inav_eph(nav_msg, dd);
    return INAV_EPH;
  }

  if (3 == word_type) {
    memcpy(nav_msg->raw_eph[2], content, GAL_INAV_CONTENT_BYTE);
    if (!eph_complete(nav_msg)) {
      return INAV_INCOMPLETE;
    }
    parse_inav_eph(nav_msg, dd);
    return INAV_EPH;
  }

  if (4 == word_type) {
    memcpy(nav_msg->raw_eph[3], content, GAL_INAV_CONTENT_BYTE);
    if (!eph_complete(nav_msg)) {
      return INAV_INCOMPLETE;
    }
    parse_inav_eph(nav_msg, dd);
    return INAV_EPH;
  }

  if (5 == word_type) {
    return INAV_TOW;
  }

  if (6 == word_type) {
    return INAV_UTC;
  }

  if (7 == word_type) {
    memcpy(nav_msg->raw_alm0, content, GAL_INAV_CONTENT_BYTE);
    return alm_complete(nav_msg);
  }

  if (8 == word_type) {
    memcpy(nav_msg->raw_alm1, content, GAL_INAV_CONTENT_BYTE);
    return alm_complete(nav_msg);
  }

  if (9 == word_type) {
    memcpy(nav_msg->raw_alm2, content, GAL_INAV_CONTENT_BYTE);
    return alm_complete(nav_msg);
  }

  if (10 == word_type) {
    memcpy(nav_msg->raw_alm3, content, GAL_INAV_CONTENT_BYTE);
    return alm_complete(nav_msg);
  }

  return INAV_INCOMPLETE;
}

/* copies the oldest `size` received symbols starting from `offset`
 * into an output array saturating them, thus converting those smaller
 * than 128 to 0 and those bigger than 127 to 255 */
static void symb_buffer_copy_saturate(u8 *out,
                                      const nav_msg_gal_inav_t *nav_msg,
                                      const u16 offset,
                                      const u16 size) {
  if (NULL == out) return;
  if (NULL == nav_msg) return;
  if (0 == size) return;

  const u8 *buff = nav_msg->decoder_buffer;
  for (u8 k = 0; k < size; k++) {
    u16 idx = (nav_msg->bit_index + offset + k) % GAL_INAV_DECODE_BUFF_SIZE;
    out[k] = (buff[idx] < 128) ? 0x00 : 0xFF;
  }
}

/** Shifts bytes properly in the array */
static void inav_buffer_store(nav_msg_gal_inav_t *nav_msg, const u8 val) {
  if (NULL == nav_msg) return;
  nav_msg->decoder_buffer[nav_msg->bit_index] = val;
  nav_msg->bit_index = (nav_msg->bit_index + 1) % GAL_INAV_DECODE_BUFF_SIZE;
}

/** Deinterleaves bytes stored in 8 rows and 30 columns */
static void deint8x30(u8 out[static GAL_INAV_PAGE_SYMB],
                      const nav_msg_gal_inav_t *nav_msg,
                      u16 offset) {
  /* bitwise de-interleaver */
  for (u8 row = 0; row < GAL_INAV_INTERL_ROW; row++) {
    for (u8 col = 0; col < GAL_INAV_INTERL_COL; col++) {
      u16 dsti = col * GAL_INAV_INTERL_ROW + row;
      u16 srci = nav_msg->bit_index + offset + row * GAL_INAV_INTERL_COL + col;
      out[dsti] = nav_msg->decoder_buffer[srci % GAL_INAV_DECODE_BUFF_SIZE];
    }
  }
}

/** Deinterleaves bytes stored in 8 rows and 30 columns */
static void flip_odd(u8 inout[static GAL_INAV_PAGE_SYMB], const u16 size) {
  /* work around the NOT port on the Galileo convolutional encoder */
  for (u8 k = 1; k < size; k += 2) {
    inout[k] = 0xFF - inout[k];
  }
}

static u32 gal_inav_compute_crc(const u8 even[static GAL_INAV_PAGE_BYTE],
                                const u8 odd[static GAL_INAV_PAGE_BYTE],
                                const int inv) {
  bool invert = inv ? true : false;

  u8 temp[32];
  /* copy the first 120 bit */
  memcpy(temp, even, GAL_INAV_PAGE_BYTE);
  /* as the destination is not Byte-aligned, need to use the slow get/setbitu */
  u32 tmpw = getbitu(odd, 0, 32);
  /* overwrite the tail bits of the even page */
  setbitu(temp, 114, 32, tmpw); /* 114 - 145 */
  tmpw = getbitu(odd, 32, 32);
  setbitu(temp, 114 + 32, 32, tmpw); /* 146 - 177 */
  tmpw = getbitu(odd, 64, 32);
  setbitu(temp, 114 + 64, 32, tmpw); /* 178 - 209 */

  u32 crc = crc24q_bits(0, temp, 196, invert);

  return crc;
}

static u32 gal_inav_extract_crc(const u8 odd[static GAL_INAV_PAGE_BYTE],
                                int inv) {
  bool invert = inv ? true : false;
  u32 crc = getbitu(odd, 82, 24);

  if (invert) {
    crc ^= 0xFFFFFF;
  }
  return crc;
}

static void extract_inav_content(u8 content[static GAL_INAV_CONTENT_BYTE],
                                 const u8 even[static GAL_INAV_PAGE_BYTE],
                                 const u8 odd[static GAL_INAV_PAGE_BYTE]) {
  u32 tmpw = getbitu(even, 2, 32);
  setbitu(content, 0, 32, tmpw); /* 0 - 31 */
  tmpw = getbitu(even, 34, 32);
  setbitu(content, 32, 32, tmpw); /* 32 - 63 */
  tmpw = getbitu(even, 66, 32);
  setbitu(content, 64, 32, tmpw); /* 64 - 95 */
  tmpw = getbitu(even, 98, 14);
  setbitu(content, 96, 14, tmpw); /* 96 - 111 */
  tmpw = getbitu(odd, 2, 16);
  setbitu(content, 112, 16, tmpw); /* 112 - 127 */
}

static bool eph_complete(nav_msg_gal_inav_t *nav_msg) {
  if ((nav_msg->iod_nav[0] == nav_msg->iod_nav[1]) &&
      (nav_msg->iod_nav[0] == nav_msg->iod_nav[2]) &&
      (nav_msg->iod_nav[0] == nav_msg->iod_nav[3])) {
    return true;
  }
  return false;
}

static bool alm_complete(nav_msg_gal_inav_t *nav_msg) {
  if ((nav_msg->iod_alm[0] == nav_msg->iod_alm[1]) &&
      (nav_msg->iod_alm[0] == nav_msg->iod_alm[2]) &&
      (nav_msg->iod_alm[0] == nav_msg->iod_alm[3])) {
    return true;
  }
  return false;
}

static float sisa_map(u32 sisa) {
  sisa &= 0xff;
  if (sisa < 50) {
    return sisa * 0.01f;
  } else if (sisa < 75) {
    return 0.5f + (sisa - 50) * 0.02f;
  } else if (sisa < 100) {
    return 1.0f + (sisa - 75) * 0.04f;
  } else if (sisa < 126) {
    return 2.0f + (sisa - 100) * 0.16f;
  }
  return INVALID_GPS_URA_VALUE;
}

#if defined GAL_INAV_FEC_HARD
/** Shifts bits properly in the bit array */
static void inav_buffer_1bit_pushr(u32 buff[static GAL_INAV_DECODE_BUFF_SIZE],
                                   const bool bitval) {
  u8 k;

  assert(buff);

  /* buff[0] contains the first preamble candidate */
  buff[0] = ((buff[0] << 1) | ((buff[1] >> 29) & 0x1)) & GAL_INAV_PREAMBLE_MASK;

  /* buff[1-8] contain the first i/nav stream candidate */
  for (k = 0; k < (GAL_INAV_PAGE_WORDS - 1); k++) {
    buff[1 + k] = ((buff[1 + k] << 1) | ((buff[2 + k] >> 29) & 0x1)) &
                  GAL_INAV_WORD_BITMASK;
  }
  /* buff[8] gets its LSB from the second preamble candidate */
  buff[GAL_INAV_PAGE_WORDS] = ((buff[GAL_INAV_PAGE_WORDS] << 1) |
                               ((buff[1 + GAL_INAV_PAGE_WORDS] >> 9) & 0x1)) &
                              GAL_INAV_WORD_BITMASK;

  /* buff[9] contains the second preamble candidate */
  buff[1 + GAL_INAV_PAGE_WORDS] =
      ((buff[1 + GAL_INAV_PAGE_WORDS] << 1) |
       ((buff[2 + GAL_INAV_PAGE_WORDS] >> 29) & 0x1)) &
      GAL_INAV_PREAMBLE_MASK;

  /* buff[10-17] contain the second i/nav stream candidate */
  for (k = 0; k < (GAL_INAV_PAGE_WORDS - 1); k++) {
    buff[2 + GAL_INAV_PAGE_WORDS + k] =
        ((buff[2 + GAL_INAV_PAGE_WORDS + k] << 1) |
         ((buff[3 + GAL_INAV_PAGE_WORDS + k] >> 29) & 0x1)) &
        GAL_INAV_WORD_BITMASK;
  }
  /* buff[17] gets its LSB from added bit */
  buff[1 + 2 * GAL_INAV_PAGE_WORDS] =
      ((buff[1 + 2 * GAL_INAV_PAGE_WORDS] << 1) | (bitval & 1)) &
      GAL_INAV_WORD_BITMASK;
}

/** deint - deinterleave every second bit */
static void deint8x30bits(u8 out[static GAL_INAV_INTERL_COL],
                          const u32 in[static GAL_INAV_INTERL_ROW]) {
  memset(out, 0, GAL_INAV_INTERL_COL * sizeof(u8));
  /* bitwise de-interleaver */
  for (u8 row = 0; row < GAL_INAV_INTERL_ROW; row++) {
    for (u8 col = 0; col < GAL_INAV_INTERL_COL; col++) {
      out[col] <<= 1;
      out[col] |= ((in[row] >> (GAL_INAV_INTERL_COL - 1 - col)) & 0x1);
    }
  }
}
#endif /* GAL_INAV_FEC_HARD */
