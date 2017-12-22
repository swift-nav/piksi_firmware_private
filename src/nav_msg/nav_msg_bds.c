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
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <libswiftnav/bits.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/ionosphere.h>
#include <libswiftnav/logging.h>

#include "nav_msg/nav_msg_bds.h"

/* `11 1000 1001 0` 11 bit modified Barker code from BDS ICD v2.1 English p.23
 */
#define BDS_PREAMBLE_MASK (0x3ff80000)
#define BDS_PREAMBLE (0x38900000)
#define BDS_PREAMBLE_INV (0x07680000)
#define BDS_WORD_BITMASK (0x3fffffff)
/* this bit mask does not work for n==0 and n==32 */
#define BITMASK(n) ((1U << n) - 1)

static void dw30_1bit_pushr(u32 *words, u8 numel, bool bitval);
static void pack_buffer(nav_msg_bds_t *n);
static u32 deint(const u32 word);

static void process_d1_fraid1(nav_msg_bds_t *n,
                              const me_gnss_signal_t mesid,
                              bds_d1_decoded_data_t *data);

static void process_d1_fraid2(nav_msg_bds_t *n,
                              const me_gnss_signal_t mesid,
                              bds_d1_decoded_data_t *data);

static void process_d1_fraid3(nav_msg_bds_t *n,
                              const me_gnss_signal_t mesid,
                              bds_d1_decoded_data_t *data);

static void process_d1_common_alm(nav_msg_bds_t *n,
                                  const me_gnss_signal_t mesid,
                                  bds_d1_decoded_data_t *data);

static void process_d1_fraid4(nav_msg_bds_t *n,
                              const me_gnss_signal_t mesid,
                              bds_d1_decoded_data_t *data);

static void process_d1_fraid5(nav_msg_bds_t *n,
                              const me_gnss_signal_t mesid,
                              bds_d1_decoded_data_t *data);

/**
 * Initializes BDS message decoder.
 *
 * \param[in] n   BDS message decoder object
 * \param[in] prn Beidou PRN id
 */
void bds_nav_msg_init(nav_msg_bds_t *n, u8 prn) {
  /* Initialize the necessary parts of the nav message state structure. */
  memset(n, 0, sizeof(*n));
  n->prn = prn;
}

/**
 * Re-initializes BDS message decoder.
 *
 * \param n BDS message decoder object
 */
void bds_nav_msg_clear_decoded(nav_msg_bds_t *n) {
  memset(n->frame_words, 0, sizeof(n->frame_words));
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
 * \return true if a new subframe started with this bit
 */
bool bds_nav_msg_update(nav_msg_bds_t *n, bool bit_val) {
  /* add new bit to buffer */
  n->bit_index++;
  dw30_1bit_pushr(n->subframe_bits, BDS_NAV_MSG_SUBFRAME_WORDS_LEN, bit_val);

  u32 pream_candidate_prev = (n->subframe_bits[0]) & BDS_PREAMBLE_MASK;
  u32 pream_candidate_last = (n->subframe_bits[10]) & BDS_PREAMBLE_MASK;

  if (false == n->subfr_sync) { /* not aligned with subframe */
    /* check preamble on TLM */
    if ((BDS_PREAMBLE != pream_candidate_prev) &&
        (BDS_PREAMBLE_INV != pream_candidate_prev)) {
      return false;
    }
    /* check if it repeats (including polarity) on second TLM */
    if (pream_candidate_prev != pream_candidate_last) {
      log_info("C%02d prev %" PRIx32 " last %" PRIx32,
               n->prn,
               pream_candidate_prev,
               pream_candidate_last);
      return false;
    }
    n->subfr_sync = true;
    n->subfr_bit_index = n->bit_index;
    n->bit_polarity = (BDS_PREAMBLE == pream_candidate_prev)
                          ? BDS_BIT_POLARITY_NORMAL
                          : BDS_BIT_POLARITY_INVERTED;
    pack_buffer(n);
    return true;

  } else { /* aligned with subframe */
    if (n->bit_index !=
        (u16)(n->subfr_bit_index + 300)) { /* it's not the subframe start */
      return false;
    } else { /* should be the subframe start */
      /* check if it repeats (including polarity) on second TLM */
      if (pream_candidate_prev != pream_candidate_last) {
        /* reset everything */
        n->subfr_sync = false;
        n->bit_polarity = BDS_BIT_POLARITY_UNKNOWN;
        log_info("C%02d lost sync prev %" PRIx32 " last %" PRIx32,
                 n->prn,
                 pream_candidate_prev,
                 pream_candidate_last);
        return false;
      }
      /* subframe start confirmed */
      n->subfr_bit_index = n->bit_index;
      pack_buffer(n);
      return true;
    }
  }
  return false;
}

/** D1 parsing
 *
 * \param n     Nav message decode state struct
 * \param mesid Signal ID
 * \param data  Target for data decoding
 *
 * \return TOW in milliseconds
 */
s32 bds_d1_process_subframe(nav_msg_bds_t *n,
                            const me_gnss_signal_t mesid,
                            bds_d1_decoded_data_t *data) {
  (void)mesid;
  (void)data;

  s32 TOWms = (((n->frame_words[0]) >> 4) & 0xffU) << 12;
  TOWms |= (((n->frame_words[1]) >> 18) & 0x3FFU);
  if (TOWms > 86400) {
    return BDS_TOW_INVALID;
  }

  u8 fraid = ((n->frame_words[0]) >> 12) & 0x7U;
  if ((fraid < 1) || (fraid > 5)) {
    return BDS_TOW_INVALID;
  }

  switch (fraid) {
    case 1:
      process_d1_fraid1(n, mesid, data);
      break;

    case 2:
      process_d1_fraid2(n, mesid, data);
      break;

    case 3:
      process_d1_fraid3(n, mesid, data);
      break;

    case 4:
      process_d1_fraid4(n, mesid, data);
      break;

    case 5:
      process_d1_fraid5(n, mesid, data);
      break;

    default:
      return BDS_TOW_INVALID;
      break;
  }

  return TOWms * 1000;
}

/** D2 parsing
 *
 * \param n     Nav message decode state struct
 * \param mesid Signal ID
 * \param data  Target for data decoding
 *
 * \return TOW in milliseconds
 */
s32 bds_d2_process_subframe(nav_msg_bds_t *n,
                            const me_gnss_signal_t mesid,
                            bds_d2_decoded_data_t *data) {
  (void)mesid;
  (void)data;

  s32 TOWms = (((n->frame_words[0]) >> 4) & 0xffU) << 12;
  TOWms |= (((n->frame_words[1]) >> 18) & 0x3FFU);
  if (TOWms > 86400) {
    return BDS_TOW_INVALID;
  }

  return TOWms * 1000;
}

/** Shifts bits properly in the bit array */
static void dw30_1bit_pushr(u32 *words, u8 numel, bool bitval) {
  u8 k;

  assert(words);
  assert(numel);

  for (k = 0; k < (numel - 1U); k++) {
    words[k] <<= 1;
    words[k] |= (words[k + 1] >> 29) & 0x1;
    words[k] &= BDS_WORD_BITMASK;
  }
  words[k] <<= 1;
  words[k] |= bitval;
  words[k] &= BDS_WORD_BITMASK;
}

/** Deinterleaves a word, from http://programming.sirrida.de/calcperm.php */
static u32 deint(const u32 x) {
  return (x & 0x20000181) | ((x & 0x08000020) << 1) | ((x & 0x02000008) << 2) |
         ((x & 0x00800002) << 3) | ((x & 0x00200000) << 4) |
         ((x & 0x00080000) << 5) | ((x & 0x00020000) << 6) |
         ((x & 0x00008000) << 7) | ((x & 0x00002000) << 8) |
         ((x & 0x00000800) << 9) | ((x & 0x00000200) << 10) |
         ((x & 0x10000000) >> 10) | ((x & 0x04000000) >> 9) |
         ((x & 0x01000000) >> 8) | ((x & 0x00400000) >> 7) |
         ((x & 0x00100000) >> 6) | ((x & 0x00040000) >> 5) |
         ((x & 0x00010000) >> 4) | ((x & 0x00004040) >> 3) |
         ((x & 0x00001010) >> 2) | ((x & 0x00000404) >> 1);
}

/** Deinterleaves signal in subframe structure */
static void pack_buffer(nav_msg_bds_t *n) {
  u32 tmp = 0;
  bool flip = (BDS_BIT_POLARITY_INVERTED == (n->bit_polarity)) ? true : false;
  for (u8 k = 0; k < BDS_WORD_SUBFR; k++) {
    tmp = n->subframe_bits[k];
    tmp = flip ? (tmp ^ BDS_WORD_BITMASK) : tmp;
    n->frame_words[k] = (k > 0) ? deint(tmp) : tmp;
  }
}

static void process_d1_fraid1(nav_msg_bds_t *n,
                              const me_gnss_signal_t mesid,
                              bds_d1_decoded_data_t *data) {
  (void)n;
  (void)mesid;
  (void)data;

  /*
  u8 sath1 = (((n->frame_words[1]) >> 17) & 0x1);
  u8 aodc = (((n->frame_words[1]) >> 12) & 0x1f);
  u8 urai = (((n->frame_words[1]) >>  8) & 0xf);
  u16 weekno = (((n->frame_words[2]) >> 17) & 0x1fff);
  u32 toc = (((n->frame_words[2]) >>  8) & 0x1ff) <<  8;
  toc    |= (((n->frame_words[3]) >> 22) & 0xff);
  u16 tgd1 = (((n->frame_words[3]) >> 12) & 0x3ff);
  u16 tgd2 = (((n->frame_words[3]) >>  8) & 0xf) << 6;
  tgd2    |= (((n->frame_words[4]) >> 24) & 0x3f);
  u8 alpha[4];
  alpha[0] = (((n->frame_words[4]) >> 16) & 0xff);
  alpha[1] = (((n->frame_words[4]) >>  8) & 0xff);
  alpha[2] = (((n->frame_words[5]) >> 22) & 0xff);
  alpha[3] = (((n->frame_words[5]) >> 14) & 0xff);
  u8 beta[4];
  beta[0]  = (((n->frame_words[5]) >>  8) & 0x3f) << 2;
  beta[0] |= (((n->frame_words[6]) >> 28) & 0x3);
  beta[1]  = (((n->frame_words[6]) >> 20) & 0xff);
  beta[2]  = (((n->frame_words[6]) >> 12) & 0xff);
  beta[3]  = (((n->frame_words[6]) >>  8) & 0xf) << 4;
  beta[3] |= (((n->frame_words[7]) >> 26) & 0xf);
  u32 a[3];
  a[2]  = (((n->frame_words[7]) >> 15) & 0x7ff);
  a[0]  = (((n->frame_words[7]) >>  8) & 0x7f) << 17;
  a[0] |= (((n->frame_words[8]) >> 13) & 0x7ffff);
  a[1]  = (((n->frame_words[8]) >>  8) & 0x1f) << 17;
  a[1] |= (((n->frame_words[9]) >> 13) & 0x7ffff);
  u8 aode = (((n->frame_words[9]) >>  8) & 0x1f);
  */
}

static void process_d1_fraid2(nav_msg_bds_t *n,
                              const me_gnss_signal_t mesid,
                              bds_d1_decoded_data_t *data) {
  (void)n;
  (void)mesid;
  (void)data;

  /*
  u32 deltan = (((n->frame_words[1]) >>  8) & 0x3ff) <<  6;
  deltan    |= (((n->frame_words[2]) >> 24) & 0x3f);
  u32 cuc = (((n->frame_words[2]) >>  8) & 0xffff) <<  2;
  cuc    |= (((n->frame_words[3]) >> 28) & 0x3);
  u32 m0 = (((n->frame_words[3]) >>  8) & 0xfffff) << 12;
  m0    |= (((n->frame_words[4]) >> 18) & 0xfff);
  u32 ecc = (((n->frame_words[4]) >>  8) & 0x3ff) << 22;
  ecc    |= (((n->frame_words[5]) >>  8) & 0x3fffff);
  u32 cus = (((n->frame_words[6]) >> 12) & 0x3ffff);
  u32 crc = (((n->frame_words[6]) >>  8) & 0xf) << 14;
  crc    |= (((n->frame_words[7]) >> 16) & 0x3fff);
  u32 crs = (((n->frame_words[7]) >>  8) & 0xff) << 10;
  crs    |= (((n->frame_words[8]) >> 20) & 0x3ff);
  u32 sqrta = (((n->frame_words[8]) >>  8) & 0xfff) << 20;
  sqrta    |= (((n->frame_words[9]) >> 10) & 0xfffff);
  u32 toe_msb = (((n->frame_words[9]) >>  8) & 0x3);
  */
}

static void process_d1_fraid3(nav_msg_bds_t *n,
                              const me_gnss_signal_t mesid,
                              bds_d1_decoded_data_t *data) {
  (void)n;
  (void)mesid;
  (void)data;

  /*
  u32 toe_lsb = (((n->frame_words[1]) >>  8) & 0x3ff) <<  5;
  toe_lsb    |= (((n->frame_words[2]) >> 25) & 0x1f);
  u32 i0 = (((n->frame_words[2]) >>  8) & 0x1ffff) << 15;
  i0    |= (((n->frame_words[3]) >> 15) & 0x7fff);
  u32 cic = (((n->frame_words[3]) >>  8) & 0x7f) << 11;
  cic    |= (((n->frame_words[4]) >> 19) & 0x7ff);
  u32 omegadot = (((n->frame_words[4]) >>  8) & 0x7ff) << 13;
  omegadot    |= (((n->frame_words[5]) >> 17) & 0x1fff);
  u32 cis = (((n->frame_words[5]) >>  8) & 0x1ff) <<  9;
  cis    |= (((n->frame_words[6]) >> 21) & 0x1ff);
  u32 idot = (((n->frame_words[6]) >>  8) & 0x1fff) <<  1;
  idot    |= (((n->frame_words[7]) >> 29) & 0x1);
  u32 omegazero = (((n->frame_words[7]) >>  8) & 0x1fffff) << 11;
  omegazero    |= (((n->frame_words[8]) >> 19) & 0x7ff);
  u32 omega = (((n->frame_words[8]) >>  8) & 0x7ff) << 21;
  omega    |= (((n->frame_words[9]) >>  9) & 0x1fffff);
  */
}

static void process_d1_common_alm(nav_msg_bds_t *n,
                                  const me_gnss_signal_t mesid,
                                  bds_d1_decoded_data_t *data) {
  (void)n;
  (void)mesid;
  (void)data;
}

static void process_d1_fraid4(nav_msg_bds_t *n,
                              const me_gnss_signal_t mesid,
                              bds_d1_decoded_data_t *data) {
  (void)n;
  (void)mesid;
  (void)data;

  u32 pnum = (((n->frame_words[1]) >> 10) & 0x7f);

  if ((pnum == 0) || (pnum > 24)) return;

  process_d1_common_alm(n, mesid, data);
}

static void process_d1_fraid5(nav_msg_bds_t *n,
                              const me_gnss_signal_t mesid,
                              bds_d1_decoded_data_t *data) {
  (void)n;
  (void)mesid;
  (void)data;

  u32 pnum = (((n->frame_words[1]) >> 10) & 0x7f);

  if ((pnum == 0) || (pnum > 24)) return;

  if (pnum <= 6) {
    process_d1_common_alm(n, mesid, data);
  }

  if (pnum == 7) {
  } else if (pnum == 8) {
  } else if (pnum == 9) {
  } else if (pnum == 10) {
  } else {
  }
}
