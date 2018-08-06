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
#include <string.h>

#include <libswiftnav/bits.h>

#include "ephemeris/ephemeris.h"
#include "nav_msg/nav_msg_bds.h"
#include "timing/timing.h"
#include "track/track_decode.h"

/* `11 1000 1001 0` 11 bit modified Barker code from BDS ICD v2.1 English p.23
 */
#define BDS_PREAMBLE_MASK (0x3ff80000)
#define BDS_PREAMBLE (0x38900000)
#define BDS_PREAMBLE_INV (0x07680000)
#define BDS_WORD_BITMASK (0x3fffffff)
/* this bit mask does not work for n==0 and n==32 */
#define BITMASK(n) ((1U << n) - 1)
#define BDS_WORD_SUBFR_MASK BITMASK(BDS_WORD_SUBFR)

static const u16 bch_table[16] = {[0b0000] = 0b000000000000000,
                                  [0b0001] = 0b000000000000001,
                                  [0b0010] = 0b000000000000010,
                                  [0b0011] = 0b000000000010000,
                                  [0b0100] = 0b000000000000100,
                                  [0b0101] = 0b000000100000000,
                                  [0b0110] = 0b000000000100000,
                                  [0b0111] = 0b000010000000000,
                                  [0b1000] = 0b000000000001000,
                                  [0b1001] = 0b100000000000000,
                                  [0b1010] = 0b000001000000000,
                                  [0b1011] = 0b000000010000000,
                                  [0b1100] = 0b000000001000000,
                                  [0b1101] = 0b010000000000000,
                                  [0b1110] = 0b000100000000000,
                                  [0b1111] = 0b001000000000000};

static const float bds_ura_table[16] = {[0] = 2.0f,
                                        [1] = 2.8f,
                                        [2] = 4.0f,
                                        [3] = 5.7f,
                                        [4] = 8.0f,
                                        [5] = 11.3f,
                                        [6] = 16.0f,
                                        [7] = 32.0f,
                                        [8] = 64.0f,
                                        [9] = 128.0f,
                                        [10] = 256.0f,
                                        [11] = 512.0f,
                                        [12] = 1024.0f,
                                        [13] = 2048.0f,
                                        [14] = 4096.0f,
                                        [15] = INVALID_URA_VALUE};

static bool subframes123_from_same_frame(const nav_msg_bds_t *n);
static void dw30_1bit_pushr(u32 *words, u8 numel, bool bitval);
static void pack_buffer(nav_msg_bds_t *n);
//~ static void dump_navmsg(const nav_msg_bds_t *n, const u8 subfr);
static void deint(u32 *hi, u32 *lo, const u32 dw);
static bool bch1511(u32 *pdw);

static void process_d1_fraid1(const nav_msg_bds_t *n,
                              bds_d1_decoded_data_t *data);

static void process_d1_fraid2(const nav_msg_bds_t *n,
                              bds_d1_decoded_data_t *data);

static void process_d1_fraid3(const nav_msg_bds_t *n,
                              bds_d1_decoded_data_t *data);

static void process_d1_common_alm(nav_msg_bds_t *n,
                                  bds_d1_decoded_data_t *data);

static void process_d1_fraid4(nav_msg_bds_t *n, bds_d1_decoded_data_t *data);

static void process_d1_fraid5(nav_msg_bds_t *n, bds_d1_decoded_data_t *data);

/**
 * Initializes BDS message decoder.
 *
 * \param[in] n     BDS message decoder object
 * \param[in] mesid Signal ID
 */
void bds_nav_msg_init(nav_msg_bds_t *n, const me_gnss_signal_t *mesid) {
  /* Initialize the necessary parts of the nav message state structure. */
  memset(n, 0, sizeof(*n));
  n->bit_polarity = BIT_POLARITY_UNKNOWN;
  n->mesid = *mesid;
}

/**
 * Re-initializes BDS message decoder.
 *
 * \param n BDS message decoder object
 */
void bds_nav_msg_clear_decoded(nav_msg_bds_t *n) {
  memset(n->page_words, 0, sizeof(n->page_words));
}

static void bds_eph_debug(const nav_msg_bds_t *n,
                          const bds_d1_decoded_data_t *data,
                          s32 TOW_s) {
  utc_tm date;
  const ephemeris_t *e = &(data->ephemeris);
  const ephemeris_kepler_t *k = &(data->ephemeris.kepler);
  make_utc_tm(&(k->toc), &date);
  log_debug_mesid(n->mesid,
                  "%4" PRIu16 " %2" PRIu8 " %2" PRIu8 " %2" PRIu8 " %2" PRIu8
                  " %2" PRIu8 "%19.11E%19.11E%19.11E  ",
                  date.year,
                  date.month,
                  date.month_day,
                  date.hour,
                  date.minute,
                  date.second_int,
                  k->af0,
                  k->af1,
                  k->af2);
  log_debug("    %19.11E%19.11E%19.11E%19.11E  ",
            (double)k->iode,
            k->crs,
            k->dn,
            k->m0);
  log_debug(
      "    %19.11E%19.11E%19.11E%19.11E  ", k->cuc, k->ecc, k->cus, k->sqrta);
  log_debug("    %19.11E%19.11E%19.11E%19.11E  ",
            (double)e->toe.tow,
            k->cic,
            k->omega0,
            k->cis);
  log_debug(
      "    %19.11E%19.11E%19.11E%19.11E  ", k->inc, k->crc, k->w, k->omegadot);
  log_debug("    %19.11E%19.11E%19.11E%19.11E  ",
            k->inc_dot,
            0.0,
            (double)e->toe.wn - BDS_WEEK_TO_GPS_WEEK,
            0.0);
  log_debug("    %19.11E%19.11E%19.11E%19.11E  ",
            e->ura,
            (double)e->health_bits,
            k->tgd_bds_s[0],
            k->tgd_bds_s[1]);
  log_debug("    %19.11E%19.11E ", rint(TOW_s), (double)k->iodc);
}

static void bds_eph_store(const nav_msg_bds_t *n, bds_d1_decoded_data_t *data) {
  ephemeris_t *e = &(data->ephemeris);
  ephemeris_kepler_t *k = &(data->ephemeris.kepler);
  ionosphere_t *iono = &(data->iono);

  add_secs(&e->toe, BDS_SECOND_TO_GPS_SECOND);
  add_secs(&k->toc, BDS_SECOND_TO_GPS_SECOND);
  add_secs(&iono->toa, BDS_SECOND_TO_GPS_SECOND);
  /* Always mark BDS ephemeris as if it was coming from B1. */
  e->sid.code = CODE_BDS2_B1;
  e->fit_interval = BDS_FIT_INTERVAL_SECONDS;
  e->valid = 1;
  shm_bds_set_shi(e->sid.sat, e->health_bits);
  eph_new_status_t r = ephemeris_new(e);
  if (EPH_NEW_OK != r) {
    log_warn_mesid(n->mesid,
                   "Error in BDS d1nav ephemeris processing. "
                   "Eph status: %" PRIu8 " ",
                   (u8)r);
  }
}

/** Process BDS D2 navigation data
 *
 * Extracts available TOW, polarity and SV health.
 *
 * \param n Nav message decode state struct
 * \param data BDS D2 data structure
 *
 * \return bds_decode_status_t
 */
bds_decode_status_t bds_d2_processing(nav_msg_bds_t *n,
                                      bds_d2_decoded_data_t *data) {
  /* TODO BDS: Save BDS D2 ephemeris */
  (void)data;
  s32 TOW_s = (((n->page_words[0]) >> 4) & 0xffU) << 12;
  TOW_s |= (((n->page_words[1]) >> 18) & 0x3FFU);
  if (TOW_s > WEEK_SECS) {
    return BDS_DECODE_RESET;
  }
  /* TODO BDS: Check BDS D2 TOW validity */
  n->TOW_ms = TOW_s * 1000 - 60;
  return BDS_DECODE_TOW_UPDATE;
}

/** Process BDS D1 navigation data
 *
 * Extracts available TOW, polarity and SV health.
 * Also saves new BDS ephemeris.
 *
 * \param n Nav message decode state struct
 * \param data BDS D1 data structure
 *
 * \return bds_decode_status_t
 */
bds_decode_status_t bds_d1_processing(nav_msg_bds_t *n,
                                      bds_d1_decoded_data_t *data) {
  u8 subfr = 1;
  for (u8 s = 2; s <= BDS_SUBFRAME_MAX; s++) {
    if ((n->subfr_times[subfr - 1]) < (n->subfr_times[s - 1])) {
      subfr = s;
    }
  }
  u32 *subfr_words = &(n->page_words[(subfr - 1) * BDS_WORD_SUBFR]);
  s32 TOW_s = (((subfr_words[0]) >> 4) & 0xff) << 12;
  TOW_s |= ((subfr_words[1]) >> 18) & 0xfff;
  if (TOW_s > WEEK_SECS) {
    return BDS_DECODE_RESET;
  }

  TOW_s += BDS_SECOND_TO_GPS_SECOND;
  if (TOW_s >= WEEK_SECS) {
    TOW_s -= WEEK_SECS;
  }
  /* Current time is 330 bits from TOW. */
  n->TOW_ms = TOW_s * SECS_MS + BDS2_B11_D1NAV_SYMBOL_LENGTH_MS * 330;

  if (0x3ffULL == ((n->goodwords_mask >> 10) & 0x3ffULL)) {
    process_d1_fraid4(n, data);
  }

  if (0x3ffULL == ((n->goodwords_mask) & 0x3ffULL)) {
    process_d1_fraid5(n, data);
  }

  if (!subframes123_from_same_frame(n)) {
    return BDS_DECODE_TOW_UPDATE;
  }

  if (0x3fffffffULL == ((n->goodwords_mask >> 20) & 0x3fffffffULL)) {
    process_d1_fraid1(n, data);
    process_d1_fraid2(n, data);
    process_d1_fraid3(n, data);
    /* debug information */
    bds_eph_debug(n, data, TOW_s);
    bds_eph_store(n, data);
    n->goodwords_mask = 0;
    n->health = shm_ephe_healthy(&data->ephemeris, n->mesid.code)
                    ? SV_HEALTHY
                    : SV_UNHEALTHY;
    return BDS_DECODE_EPH_UPDATE;
  }

  return BDS_DECODE_TOW_UPDATE;
}

/** BDS navigation message decoding update.
 * Called once per nav bit interval.
 *
 * Extracts BDS D1 & D2 data for tracker sync.
 *
 * \param n       Nav message decode state struct
 * \param nav_bit Struct containing nav_bit data
 *
 * \return bds_decode_status_t Decoder status
 */
bds_decode_status_t bds_data_decoding(nav_msg_bds_t *n, nav_bit_t nav_bit) {
  bds_d1_decoded_data_t dd_d1nav;
  bds_d2_decoded_data_t dd_d2nav;
  memset(&dd_d1nav, 0, sizeof(bds_d1_decoded_data_t));
  memset(&dd_d2nav, 0, sizeof(bds_d2_decoded_data_t));

  /* Don't decode data while in sensitivity mode. */
  if ((0 == nav_bit.data) || (nav_bit.cnt != n->bit_cnt)) {
    me_gnss_signal_t tmp_mesid = n->mesid;
    bds_nav_msg_init(n, &tmp_mesid);
    n->bit_cnt = nav_bit.cnt + 1;
    return BDS_DECODE_RESET;
  }
  n->bit_cnt++;

  bool bit_val = nav_bit.data > 0;
  bool tlm_rx = bds_nav_msg_update(n, bit_val);
  if (!tlm_rx) {
    return BDS_DECODE_WAIT;
  }

  bds_decode_status_t status;
  if (bds_d2nav(n->mesid)) {
    status = bds_d2_processing(n, &dd_d2nav);
  } else {
    status = bds_d1_processing(n, &dd_d1nav);
  }
  return status;
}

/** Fill in BDS sync data to tracker.
 *
 * Sets sync flags based on decoder status.
 *
 * \param n            Nav message decode state struct
 * \param status       Decoder status
 *
 * \return nav_data_sync_t Struct for tracker synchronization
 */
nav_data_sync_t construct_bds_data_sync(const nav_msg_bds_t *n,
                                        bds_decode_status_t status) {
  nav_data_sync_t from_decoder;
  memset(&from_decoder, 0, sizeof(from_decoder));
  from_decoder.TOW_ms = n->TOW_ms;
  from_decoder.bit_polarity = n->bit_polarity;
  from_decoder.health = n->health;

  switch (status) {
    case BDS_DECODE_TOW_UPDATE:
      from_decoder.sync_flags = SYNC_POL | SYNC_TOW;
      break;
    case BDS_DECODE_EPH_UPDATE:
      from_decoder.sync_flags = SYNC_POL | SYNC_TOW | SYNC_EPH;
      break;
    case BDS_DECODE_WAIT:
    case BDS_DECODE_RESET:
    default:
      from_decoder.sync_flags = SYNC_NONE;
      break;
  }
  return from_decoder;
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
 * \return true if first word of a new subframe ended with this bit
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
      log_debug_mesid(n->mesid,
                      "prev %" PRIx32 " last %" PRIx32,
                      pream_candidate_prev,
                      pream_candidate_last);
      return false;
    }
    /* check that there are no bit errors */
    if (!crc_check(n)) {
      return false;
    }
    /* subframe start found */
    n->subfr_sync = true;
    n->subfr_bit_index = n->bit_index;
    n->bit_polarity = (BDS_PREAMBLE == pream_candidate_prev)
                          ? BIT_POLARITY_NORMAL
                          : BIT_POLARITY_INVERTED;
    pack_buffer(n);
    return true;

  } else { /* aligned with subframe */
    if (n->bit_index !=
        (u16)(n->subfr_bit_index + 300)) { /* it's not the subframe start */
      return false;
    } else { /* should be the subframe start */
      /* check if it repeats (including polarity) on second TLM */
      if (pream_candidate_prev != pream_candidate_last) {
        /* reset subframe sync and polarity */
        n->subfr_sync = false;
        n->bit_polarity = BIT_POLARITY_UNKNOWN;
        log_debug_mesid(n->mesid,
                        "lost sync prev %" PRIx32 " last %" PRIx32,
                        pream_candidate_prev,
                        pream_candidate_last);
        return false;
      }
      /* check that there are no bit errors */
      if (!crc_check(n)) {
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

/*
static void dump_navmsg(const nav_msg_bds_t *n, const u8 subfr) {
  char bitstream[256];
  char tempstr[64];
  const u32 *subfr_words = &(n->page_words[(subfr-1)*BDS_WORD_SUBFR]);
  u32 tow = (((subfr_words[0] >> 4) << 12) |
             ((subfr_words[1] >> 18) & 0xfffU)) &
            0xfffffU;
  sprintf(bitstream, " 3 %02d %6" PRIu32 "  ", n->mesid.sat, tow);
  for (u8 k = 0; k < BDS_WORD_SUBFR; k++) {
    sprintf(tempstr, "%08" PRIx32 " ", subfr_words[k]);
    strcat(bitstream, tempstr);
  }
  log_info("%s", bitstream);
}
*/

static bool subframes123_from_same_frame(const nav_msg_bds_t *n) {
  /* If subframe 1 is newer than subframe 2, or
   * if subframe 2 is newer than subframe 3,
   * then wait. */
  u64 rx_diff1 = n->subfr_times[1] - n->subfr_times[0];
  u64 rx_diff2 = n->subfr_times[2] - n->subfr_times[1];
  /* Age threshold is one subframe + 1 second,
   * to allow millisecond delays in decoder. */
  u64 age_threshold = (BDS_D1_SUBFRAME_LEN_SECONDS + 1) * SECS_MS;

  if ((rx_diff1 >= age_threshold) || (rx_diff2 >= age_threshold)) {
    return false;
  }
  return true;
}

/** Shifts bits properly in the bit array */
static void dw30_1bit_pushr(u32 *words, u8 numel, bool bitval) {
  u8 k;

  ASSERT(words);
  ASSERT(numel);

  for (k = 0; k < (numel - 1U); k++) {
    words[k] <<= 1;
    words[k] |= (words[k + 1] >> 29) & 0x1;
    words[k] &= BDS_WORD_BITMASK;
  }
  words[k] <<= 1;
  words[k] |= bitval;
  words[k] &= BDS_WORD_BITMASK;
}

/** morton1 - extract even bits */
static u32 morton(u32 x) {
  x = x & 0x55555555;
  x = (x | (x >> 1)) & 0x33333333;
  x = (x | (x >> 2)) & 0x0F0F0F0F;
  x = (x | (x >> 4)) & 0x00FF00FF;
  x = (x | (x >> 8)) & 0x0000FFFF;
  return x;
}

/** deint - deinterleave every second bit */
static void deint(u32 *hi, u32 *lo, const u32 dw) {
  *hi = morton(dw >> 1);
  *lo = morton(dw);
}

/** BCH1511 decoder
 * Note: works as CRC and single bit error correction, assuming one trusts it
 * */
static bool bch1511(u32 *pdw) {
  u16 in = (*pdw) & 0x7fff;
  u8 crc = 0, bit = 0;
  for (s8 k = 14; k >= 0; k--) {
    bit = ((crc >> 3) & 0x1);
    crc ^= bit;
    crc <<= 1;
    crc |= (bit ^ ((in >> k) & 0x1));
  }
  crc &= 0xf;
  (*pdw) = (in ^ bch_table[crc]); /* 1-bit error correction */
  return (0 == crc);              /* crc pass/fail */
}

/** BCH(15,11) check on all received bits */
bool crc_check(nav_msg_bds_t *n) {
  u32 good_words = 0;
  for (u8 k = 0; k < BDS_WORD_SUBFR; k++) {
    u32 hi, lo;
    bool good;
    if (0 == k) {
      hi = (n->subframe_bits[k] >> 15) & 0x7fff;
      lo = (n->subframe_bits[k]) & 0x7fff;
      good = bch1511(&lo);
    } else {
      deint(&hi, &lo, n->subframe_bits[k]);
      good = bch1511(&hi) && bch1511(&lo);
    }
    if (good) {
      good_words |= (1 << k);
    }
    n->subframe_bits[k] = (hi << 15) | lo;
    /* with this ^ the deinterleave is done, but location of 4+4 bit CRC
     * in the 30-bits is still wrong */
  }
  /* check if all words passed the CRC check */
  if (good_words != BDS_WORD_SUBFR_MASK) {
    log_debug_mesid(n->mesid, "good_words %08" PRIx32, good_words);
    return false;
  }
  return true;
}

/** pack CRC bits at the end as 11a+11b+4a+4b */
static u32 packdw(const u32 dw) {
  return (((dw >> 19) & 0x7ff) << 19) | (((dw >> 4) & 0x7ff) << 8) |
         (((dw >> 15) & 0x00f) << 4) | (((dw)&0x00f));
}

/** Deinterleaves signal in subframe structure */
static void pack_buffer(nav_msg_bds_t *n) {
  u32 tmp = n->subframe_bits[0];
  bool flip = (BIT_POLARITY_INVERTED == (n->bit_polarity)) ? true : false;
  tmp = flip ? (tmp ^ BDS_WORD_BITMASK) : tmp;
  u8 subfr = (tmp >> 12) & 0x7;
  if ((subfr < 1) || (subfr > 5)) {
    log_warn_mesid(n->mesid, "subframe %" PRIu8 "error", subfr);
    return;
  }
  for (u8 k = 0; k < BDS_WORD_SUBFR; k++) {
    tmp = n->subframe_bits[k];
    tmp = flip ? (tmp ^ BDS_WORD_BITMASK) : tmp;
    n->page_words[(subfr - 1) * BDS_WORD_SUBFR + k] =
        (0 == k) ? tmp : packdw(tmp);
  }
  /* store correctly decoded words into mask */
  n->goodwords_mask |= (0x3ffULL << (10 * (5 - subfr)));
  /* store subframe rx time */
  n->subfr_times[subfr - 1] = timing_getms();
  /* debug message to verify decoder output */
  //~ dump_navmsg(n, subfr);
}

static void process_d1_fraid1(const nav_msg_bds_t *n,
                              bds_d1_decoded_data_t *data) {
  ephemeris_t *e = &(data->ephemeris);
  ephemeris_kepler_t *k = &(data->ephemeris.kepler);
  ionosphere_t *i = &(data->iono);

  u8 sath1 = (((n->page_words[1]) >> 17) & 0x1);
  u8 aodc = (((n->page_words[1]) >> 12) & 0x1f);
  u8 urai = (((n->page_words[1]) >> 8) & 0xf);
  u16 weekno = (((n->page_words[2]) >> 17) & 0x1fff);
  u32 toc = (((n->page_words[2]) >> 8) & 0x1ff) << 8;
  toc |= (((n->page_words[3]) >> 22) & 0xff);
  u16 tgd1 = (((n->page_words[3]) >> 12) & 0x3ff);
  u16 tgd2 = (((n->page_words[3]) >> 8) & 0xf) << 6;
  tgd2 |= (((n->page_words[4]) >> 24) & 0x3f);
  s8 alpha[4];
  alpha[0] = (((n->page_words[4]) >> 16) & 0xff);
  alpha[1] = (((n->page_words[4]) >> 8) & 0xff);
  alpha[2] = (((n->page_words[5]) >> 22) & 0xff);
  alpha[3] = (((n->page_words[5]) >> 14) & 0xff);
  s8 beta[4];
  beta[0] = (((n->page_words[5]) >> 8) & 0x3f) << 2;
  beta[0] |= (((n->page_words[6]) >> 28) & 0x3);
  beta[1] = (((n->page_words[6]) >> 20) & 0xff);
  beta[2] = (((n->page_words[6]) >> 12) & 0xff);
  beta[3] = (((n->page_words[6]) >> 8) & 0xf) << 4;
  beta[3] |= (((n->page_words[7]) >> 26) & 0xf);
  u32 a[3];
  a[2] = (((n->page_words[7]) >> 15) & 0x7ff);
  a[0] = (((n->page_words[7]) >> 8) & 0x7f) << 17;
  a[0] |= (((n->page_words[8]) >> 13) & 0x1ffff);
  a[1] = (((n->page_words[8]) >> 8) & 0x1f) << 17;
  a[1] |= (((n->page_words[9]) >> 13) & 0x1ffff);
  u8 aode = (((n->page_words[9]) >> 8) & 0x1f);

  /* Beidou specific data */
  data->aodc = aodc;
  data->aode = aode;
  /* Ephemeris params */
  e->sid = mesid2sid(n->mesid, GLO_ORBIT_SLOT_UNKNOWN);
  e->health_bits = sath1;
  e->ura = bds_ura_table[urai];
  e->toe.wn = BDS_WEEK_TO_GPS_WEEK + weekno;
  /* Keplerian params */
  k->tgd_bds_s[0] = BITS_SIGN_EXTEND_32(10, tgd1) * 1e-10;
  k->tgd_bds_s[1] = BITS_SIGN_EXTEND_32(10, tgd2) * 1e-10;
  k->toc.wn = e->toe.wn;
  k->toc.tow = (double)toc * C_2P3;
  k->af0 = BITS_SIGN_EXTEND_32(24, a[0]) * C_1_2P33;
  k->af1 = BITS_SIGN_EXTEND_32(22, a[1]) * C_1_2P50;
  k->af2 = BITS_SIGN_EXTEND_32(11, a[2]) * C_1_2P66;
  /* RTCM recommendation, BDS IODC = mod(toc / 720, 240)
   * Note scale factor effect, (toc * 8) / 720 -> (toc / 90) */
  k->iodc = (toc / 90) % 240;

  /* IONO params */
  i->toa.wn = e->toe.wn;
  i->a0 = (double)(alpha[0] * C_1_2P30);
  i->a1 = (double)(alpha[1] * C_1_2P27);
  i->a2 = (double)(alpha[2] * C_1_2P24);
  i->a3 = (double)(alpha[3] * C_1_2P24);
  i->b0 = (double)(beta[0] * C_2P11);
  i->b1 = (double)(beta[1] * C_2P14);
  i->b2 = (double)(beta[2] * C_2P16);
  i->b3 = (double)(beta[3] * C_2P16);
}

static void process_d1_fraid2(const nav_msg_bds_t *n,
                              bds_d1_decoded_data_t *data) {
  ephemeris_kepler_t *k = &(data->ephemeris.kepler);

  u32 deltan = (((n->page_words[11]) >> 8) & 0x3ff) << 6;
  deltan |= (((n->page_words[12]) >> 24) & 0x3f);
  u32 cuc = (((n->page_words[12]) >> 8) & 0xffff) << 2;
  cuc |= (((n->page_words[13]) >> 28) & 0x3);
  u32 m0 = (((n->page_words[13]) >> 8) & 0xfffff) << 12;
  m0 |= (((n->page_words[14]) >> 18) & 0xfff);
  u32 ecc = (((n->page_words[14]) >> 8) & 0x3ff) << 22;
  ecc |= (((n->page_words[15]) >> 8) & 0x3fffff);
  u32 cus = (((n->page_words[16]) >> 12) & 0x3ffff);
  u32 crc = (((n->page_words[16]) >> 8) & 0xf) << 14;
  crc |= (((n->page_words[17]) >> 16) & 0x3fff);
  u32 crs = (((n->page_words[17]) >> 8) & 0xff) << 10;
  crs |= (((n->page_words[18]) >> 20) & 0x3ff);
  u32 sqrta = (((n->page_words[18]) >> 8) & 0xfff) << 20;
  sqrta |= (((n->page_words[19]) >> 10) & 0xfffff);
  u32 toe_msb = (((n->page_words[19]) >> 8) & 0x3);

  /* Beidou specific data */
  data->split_toe &= ~(0x3 << 15);
  data->split_toe |= (toe_msb << 15);
  data->split_toe_mask |= (0x3 << 15);
  /* Keplerian params */
  k->dn = BITS_SIGN_EXTEND_32(16, deltan) * C_1_2P43 * GPS_PI;
  k->cuc = BITS_SIGN_EXTEND_32(18, cuc) * C_1_2P31;
  k->m0 = BITS_SIGN_EXTEND_32(32, m0) * C_1_2P31 * GPS_PI;
  k->ecc = ecc * C_1_2P33;
  k->cus = BITS_SIGN_EXTEND_32(18, cus) * C_1_2P31;
  k->crc = BITS_SIGN_EXTEND_32(18, crc) * C_1_2P6;
  k->crs = BITS_SIGN_EXTEND_32(18, crs) * C_1_2P6;
  k->sqrta = sqrta * C_1_2P19;
}

static void process_d1_fraid3(const nav_msg_bds_t *n,
                              bds_d1_decoded_data_t *data) {
  ephemeris_t *e = &(data->ephemeris);
  ephemeris_kepler_t *k = &(data->ephemeris.kepler);
  double new_toe;

  u32 toe_lsb = (((n->page_words[21]) >> 8) & 0x3ff) << 5;
  toe_lsb |= (((n->page_words[22]) >> 25) & 0x1f);
  u32 i0 = (((n->page_words[22]) >> 8) & 0x1ffff) << 15;
  i0 |= (((n->page_words[23]) >> 15) & 0x7fff);
  u32 cic = (((n->page_words[23]) >> 8) & 0x7f) << 11;
  cic |= (((n->page_words[24]) >> 19) & 0x7ff);
  u32 omegadot = (((n->page_words[24]) >> 8) & 0x7ff) << 13;
  omegadot |= (((n->page_words[25]) >> 17) & 0x1fff);
  u32 cis = (((n->page_words[25]) >> 8) & 0x1ff) << 9;
  cis |= (((n->page_words[26]) >> 21) & 0x1ff);
  u32 idot = (((n->page_words[26]) >> 8) & 0x1fff) << 1;
  idot |= (((n->page_words[27]) >> 29) & 0x1);
  u32 omegazero = (((n->page_words[27]) >> 8) & 0x1fffff) << 11;
  omegazero |= (((n->page_words[28]) >> 19) & 0x7ff);
  u32 omega = (((n->page_words[28]) >> 8) & 0x7ff) << 21;
  omega |= (((n->page_words[29]) >> 9) & 0x1fffff);

  /* Beidou specific data */
  data->split_toe &= ~(0x7ff);
  data->split_toe |= toe_lsb;
  data->split_toe_mask |= (0x7ff);
  /* Ephemeris params */
  if (data->split_toe_mask) {
    new_toe = data->split_toe * C_2P3;
    if (e->toe.tow != new_toe) {
      data->split_toe_mask = 0;
    }
    e->toe.tow = new_toe;
    /* RTCM recommendation, BDS IODE = mod(toe / 720, 240)
     * Note scale factor effect, (toe * 8) / 720 -> (toe / 90) */
    k->iode = (data->split_toe / 90) % 240;
  }
  /* Keplerian params */
  k->inc = BITS_SIGN_EXTEND_32(32, i0) * C_1_2P31 * GPS_PI;
  k->cic = BITS_SIGN_EXTEND_32(18, cic) * C_1_2P31;
  k->omegadot = BITS_SIGN_EXTEND_32(24, omegadot) * C_1_2P43 * GPS_PI;
  k->cis = BITS_SIGN_EXTEND_32(18, cis) * C_1_2P31;
  k->inc_dot = BITS_SIGN_EXTEND_32(14, idot) * C_1_2P43 * GPS_PI;
  k->omega0 = BITS_SIGN_EXTEND_32(32, omegazero) * C_1_2P31 * GPS_PI;
  k->w = BITS_SIGN_EXTEND_32(32, omega) * C_1_2P31 * GPS_PI;
}

static void process_d1_common_alm(nav_msg_bds_t *n,
                                  bds_d1_decoded_data_t *data) {
  (void)n;
  (void)data;
}

static void process_d1_fraid4(nav_msg_bds_t *n, bds_d1_decoded_data_t *data) {
  u32 pnum = (((n->page_words[31]) >> 10) & 0x7f);

  if ((pnum == 0) || (pnum > 24)) return;

  process_d1_common_alm(n, data);
}

static void process_d1_fraid5(nav_msg_bds_t *n, bds_d1_decoded_data_t *data) {
  u32 pnum = (((n->page_words[41]) >> 10) & 0x7f);

  if ((pnum == 0) || (pnum > 24)) return;

  if (pnum <= 6) {
    process_d1_common_alm(n, data);
  }

  if (pnum == 7) {
  } else if (pnum == 8) {
  } else if (pnum == 9) {
  } else if (pnum == 10) {
  } else {
  }
}
