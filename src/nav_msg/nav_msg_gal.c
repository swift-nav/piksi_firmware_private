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
#include <string.h>

#include <libfec/fec.h>
#include <libswiftnav/bits.h>
#include <libswiftnav/edc.h>

#include "ephemeris/ephemeris.h"
#include "nav_msg/nav_msg.h"
#include "nav_msg/nav_msg_gal.h"

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

static float sisa_map(u8 sisa);
static void parse_inav_eph(const nav_msg_gal_inav_t *nav_msg,
                           gal_inav_decoded_t *dd,
                           const gps_time_t *t_dec);
static void parse_inav_alm3(nav_msg_gal_inav_t *nav_msg,
                            gal_inav_decoded_t *dd);
static u32 parse_inav_utc(const u8 content[GAL_INAV_CONTENT_BYTE],
                          gal_inav_decoded_t *dd);
static void parse_inav_bgd(const u8 content[GAL_INAV_CONTENT_BYTE],
                           gal_inav_decoded_t *dd);
static void parse_inav_health6(const u8 content[GAL_INAV_CONTENT_BYTE],
                               gal_inav_decoded_t *dd);
static gps_time_t parse_inav_w5tow(const u8 content[GAL_INAV_CONTENT_BYTE]);
static gps_time_t parse_inav_w0tow(const u8 content[GAL_INAV_CONTENT_BYTE]);

/**
 * Initializes Galileo message decoder.
 *
 * \param[in] n   GAL message decoder object
 * \param[in] prn Galileo PRN id
 */
void gal_inav_msg_init(nav_msg_gal_inav_t *n, const me_gnss_signal_t *mesid) {
  const s8 coeffs[2] = {GAL_INAV_V27_POLY_A, GAL_INAV_V27_POLY_B};
  v27_poly_init(&poly, coeffs);

  /* Initialize the necessary parts of the nav message state structure. */
  memset(n, 0, sizeof(*n));
  n->mesid = *mesid;

  n->iod_nav[0] = GAL_INAV_IOD_INVALID;
  n->iod_nav[1] = GAL_INAV_IOD_INVALID;
  n->iod_nav[2] = GAL_INAV_IOD_INVALID;
  n->iod_nav[3] = GAL_INAV_IOD_INVALID;

  n->iod_alm[0] = GAL_INAV_IOD_INVALID;
  n->iod_alm[1] = GAL_INAV_IOD_INVALID;
  n->iod_alm[2] = GAL_INAV_IOD_INVALID;
  n->iod_alm[3] = GAL_INAV_IOD_INVALID;

  n->bit_polarity = BIT_POLARITY_UNKNOWN;
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
  ASSERT(n);

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
  v27_chainback_fixed(
      &(n->decoder), candidate_even_bits, GAL_INAV_PAGE_BIT, 0x0);

  /* check if the first is an even page */
  u8 even_flag = getbitu(candidate_even_bits, 6, 1);
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
  v27_chainback_fixed(
      &(n->decoder), candidate_odd_bits, GAL_INAV_PAGE_BIT, 0x0);

  u8 odd_flag = getbitu(candidate_odd_bits, 6, 1);
  /* check if the second is an odd page */
  if (1 != odd_flag) {
    return false;
  }

  u32 crc_calc =
      gal_inav_compute_crc(candidate_even_bits, candidate_odd_bits, differ0);
  u32 crc_extract = gal_inav_extract_crc(candidate_odd_bits, differ0);

  bool crc_ok = (crc_calc == crc_extract);
  if (!crc_ok) {
    log_info_mesid(n->mesid,
                   "crc_calc %08" PRIx32 " crc_extract %08" PRIx32,
                   crc_calc,
                   crc_extract);
    return false;
  }

  /* at this point CRC matches: we have a good word */

  /* if different from the flipped, we are in phase */
  n->bit_polarity = differ1 ? BIT_POLARITY_NORMAL : BIT_POLARITY_INVERTED;
  if (differ0) {
    log_warn_mesid(n->mesid, "inverted preamble");
  }

  /* don't try anything for another 500 symbols */
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
  log_info_mesid(n->mesid, "%s", str);
   */

  bool alert =
      getbitu(candidate_even_bits, 7, 1) || getbitu(candidate_odd_bits, 7, 1);
  if (alert) {
    /* do not process an alert page further */
    DO_EACH_MS(10 * SECS_MS,
               log_info_mesid(n->mesid, "received an alert page"););
    return false;
  }

  extract_inav_content(n->raw_content, candidate_even_bits, candidate_odd_bits);

  return true;
}

static void gal_eph_debug(const nav_msg_gal_inav_t *n,
                          const gal_inav_decoded_t *data,
                          const gps_time_t *t) {
  const ephemeris_t *e = &(data->ephemeris);
  const ephemeris_kepler_t *k = &(data->ephemeris.kepler);
  utc_tm date;
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
            1.0,
            (double)e->toe.wn,
            0.0);
  log_debug("    %19.11E%19.11E%19.11E%19.11E  ",
            e->ura,
            (double)e->health_bits,
            k->tgd_gal_s[0],
            k->tgd_gal_s[1]);
  log_debug("    %19.11E%19.11E ", rint(t->tow), 0.0);
}

static void gal_eph_store(const nav_msg_gal_inav_t *n,
                          gal_inav_decoded_t *data,
                          const gps_time_t *t) {
  gal_eph_debug(n, data, t);
  ephemeris_t *e = &(data->ephemeris);
  /* Always mark GAL ephemeris as if it was coming from E1. */
  e->sid.code = CODE_GAL_E1B;
  e->valid = 1;
  shm_gal_set_shi(e->sid.sat, e->health_bits);
  eph_new_status_t estat = ephemeris_new(e);
  if (EPH_NEW_OK != estat) {
    log_warn_mesid(n->mesid,
                   "Error in GAL INAV ephemeris processing. "
                   "Eph status: %" PRIu8 " ",
                   (u8)estat);
  }
}

inav_data_type_t parse_inav_word(nav_msg_gal_inav_t *nav_msg,
                                 gal_inav_decoded_t *dd) {
  ASSERT(nav_msg);
  u8 *content = nav_msg->raw_content;
  gps_time_t t_dec = GPS_TIME_UNKNOWN;

  u8 word_type = getbitu(content, 0, 6);
  if (0 == word_type) {
    u32 tflag = getbitu(content, 6, 2);
    if (tflag != 0b10) return INAV_INCOMPLETE;
    t_dec = parse_inav_w0tow(content);
    log_debug_mesid(nav_msg->mesid, "WN %d TOW %.3f", t_dec.wn, t_dec.tow);
    nav_msg->TOW_ms = (s32)rint(t_dec.tow * 1000) + 2000;
    return INAV_TOW;
  }

  if (1 == word_type) {
    nav_msg->iod_nav[0] = getbitu(content, 6, 10);
    memcpy(nav_msg->raw_eph[0], content, GAL_INAV_CONTENT_BYTE);
    return INAV_INCOMPLETE;
  }

  if (2 == word_type) {
    nav_msg->iod_nav[1] = getbitu(content, 6, 10);
    memcpy(nav_msg->raw_eph[1], content, GAL_INAV_CONTENT_BYTE);
    return INAV_INCOMPLETE;
  }

  if (3 == word_type) {
    nav_msg->iod_nav[2] = getbitu(content, 6, 10);
    memcpy(nav_msg->raw_eph[2], content, GAL_INAV_CONTENT_BYTE);
    return INAV_INCOMPLETE;
  }

  if (4 == word_type) {
    nav_msg->iod_nav[3] = getbitu(content, 6, 10);
    memcpy(nav_msg->raw_eph[3], content, GAL_INAV_CONTENT_BYTE);
    return INAV_INCOMPLETE;
  }

  if (5 == word_type) {
    memcpy(nav_msg->raw_eph[4], content, GAL_INAV_CONTENT_BYTE);
    if (!eph_complete(nav_msg)) {
      return INAV_INCOMPLETE;
    }
    parse_inav_bgd(content, dd);
    parse_inav_health6(content, dd);
    t_dec = parse_inav_w5tow(content);
    parse_inav_eph(nav_msg, dd, &t_dec);
    gal_eph_store(nav_msg, dd, &t_dec);
    nav_msg->TOW_ms = (s32)rint(t_dec.tow * 1000) + 2000;
    nav_msg->health = shm_ephe_healthy(&dd->ephemeris, nav_msg->mesid.code)
                          ? SV_HEALTHY
                          : SV_UNHEALTHY;
    return INAV_EPH;
  }

  if (6 == word_type) {
    t_dec.wn = WN_UNKNOWN;
    t_dec.tow = (double)parse_inav_utc(content, dd);
    log_debug_mesid(nav_msg->mesid, "WN %d TOW %.3f", t_dec.wn, t_dec.tow);
    nav_msg->TOW_ms = (s32)rint(t_dec.tow * 1000) + 2000;
    return INAV_UTC;
  }

  if (7 == word_type) {
    nav_msg->iod_alm[0] = getbitu(content, 6, 4);
    memcpy(nav_msg->raw_alm[0], content, GAL_INAV_CONTENT_BYTE);
    if (!alm_complete(nav_msg)) {
      return INAV_INCOMPLETE;
    }
    parse_inav_alm3(nav_msg, dd);
    return INAV_ALM;
  }

  if (8 == word_type) {
    nav_msg->iod_alm[1] = getbitu(content, 6, 4);
    memcpy(nav_msg->raw_alm[1], content, GAL_INAV_CONTENT_BYTE);
    if (!alm_complete(nav_msg)) {
      return INAV_INCOMPLETE;
    }
    parse_inav_alm3(nav_msg, dd);
    return INAV_ALM;
  }

  if (9 == word_type) {
    nav_msg->iod_alm[2] = getbitu(content, 6, 4);
    memcpy(nav_msg->raw_alm[2], content, GAL_INAV_CONTENT_BYTE);
    if (!alm_complete(nav_msg)) {
      return INAV_INCOMPLETE;
    }
    parse_inav_alm3(nav_msg, dd);
    return INAV_ALM;
  }

  if (10 == word_type) {
    nav_msg->iod_alm[3] = getbitu(content, 6, 4);
    memcpy(nav_msg->raw_alm[3], content, GAL_INAV_CONTENT_BYTE);
    if (!alm_complete(nav_msg)) {
      return INAV_INCOMPLETE;
    }
    parse_inav_alm3(nav_msg, dd);
    return INAV_ALM;
  }

  if (63 == word_type) {
    log_debug_mesid(nav_msg->mesid, "Dummy msg received from GAL SV");
    nav_msg->health = SV_UNHEALTHY;
    return INAV_DUMMY;
  }

  return INAV_INCOMPLETE;
}

/** GAL navigation message decoding update.
 * Called once per nav bit interval.
 *
 * Extracts GAL E1 & E7 INAV data for tracker sync.
 *
 * \param n       Nav message decode state struct
 * \param nav_bit Struct containing nav_bit data
 *
 * \return gal_decode_status_t
 */
gal_decode_status_t gal_data_decoding(nav_msg_gal_inav_t *n,
                                      const nav_bit_t *nav_bit) {
  /* Don't decode data while in sensitivity mode. */
  if ((0 == nav_bit->data) || (nav_bit->cnt != n->bit_cnt)) {
    me_gnss_signal_t tmp_mesid = n->mesid;
    gal_inav_msg_init(n, &tmp_mesid);
    n->bit_cnt = nav_bit->cnt + 1;
    return GAL_DECODE_RESET;
  }
  n->bit_cnt++;

  bool upd = gal_inav_msg_update(n, nav_bit->data);
  if (!upd) {
    return GAL_DECODE_WAIT;
  }

  gal_inav_decoded_t dd;
  gal_decode_status_t status = GAL_DECODE_WAIT;
  inav_data_type_t ret = parse_inav_word(n, &dd);
  switch (ret) {
    case INAV_TOW:
    case INAV_UTC:
      status = GAL_DECODE_TOW_UPDATE;
      break;
    case INAV_EPH:
      status = GAL_DECODE_EPH_UPDATE;
      break;
    case INAV_DUMMY:
      status = GAL_DECODE_DUMMY_UPDATE;
      break;
    case INAV_ALM:
    case INAV_INCOMPLETE:
    default:
      break;
  }
  return status;
}

/** Fill in GAL sync data to tracker.
 *
 * Sets sync flags based on decoder status.
 *
 * \param n            Nav message decode state struct
 * \param status       Decoder status
 *
 * \return nav_data_sync_t Struct for tracker synchronization
 */
nav_data_sync_t construct_gal_data_sync(const nav_msg_gal_inav_t *n,
                                        gal_decode_status_t status) {
  nav_data_sync_t from_decoder;
  memset(&from_decoder, 0, sizeof(from_decoder));
  from_decoder.TOW_ms = n->TOW_ms;
  from_decoder.bit_polarity = n->bit_polarity;
  from_decoder.health = n->health;

  switch (status) {
    case GAL_DECODE_TOW_UPDATE:
      from_decoder.sync_flags = SYNC_POL | SYNC_TOW;
      break;
    case GAL_DECODE_EPH_UPDATE:
      from_decoder.sync_flags = SYNC_ALL;
      break;
    case GAL_DECODE_DUMMY_UPDATE:
      from_decoder.sync_flags = SYNC_POL | SYNC_EPH;
      break;
    case GAL_DECODE_WAIT:
    case GAL_DECODE_RESET:
    default:
      from_decoder.sync_flags = SYNC_NONE;
      break;
  }
  return from_decoder;
}

static void parse_inav_bgd(const u8 content[GAL_INAV_CONTENT_BYTE],
                           gal_inav_decoded_t *dd) {
  ephemeris_kepler_t *kep = &(dd->ephemeris.kepler);
  u32 e1e5a = getbitu(content, 47, 10);
  u32 e1e5b = getbitu(content, 57, 10);
  kep->tgd_gal_s[0] = BITS_SIGN_EXTEND_32(10, e1e5a) * C_1_2P32;
  kep->tgd_gal_s[1] = BITS_SIGN_EXTEND_32(10, e1e5b) * C_1_2P32;
}

static void parse_inav_health6(const u8 content[GAL_INAV_CONTENT_BYTE],
                               gal_inav_decoded_t *dd) {
  ephemeris_t *eph = &(dd->ephemeris);
  eph->health_bits = getbitu(content, 67, 6);
}

static gps_time_t parse_inav_w5tow(const u8 content[GAL_INAV_CONTENT_BYTE]) {
  gps_time_t t;
  t.wn = (s16)getbitu(content, 73, 12) + GAL_WEEK_TO_GPS_WEEK;
  t.tow = (double)getbitu(content, 85, 20);
  return t;
}

static gps_time_t parse_inav_w0tow(const u8 content[GAL_INAV_CONTENT_BYTE]) {
  gps_time_t t;
  t.wn = (s16)getbitu(content, 96, 12) + GAL_WEEK_TO_GPS_WEEK;
  t.tow = (double)getbitu(content, 108, 20);
  return t;
}

static u32 parse_inav_utc(const u8 content[GAL_INAV_CONTENT_BYTE],
                          gal_inav_decoded_t *dd) {
  utc_params_t *utc = &(dd->utc);

  u32 a0 = getbitu(content, 6, 32);
  u32 a1 = getbitu(content, 38, 24);
  u32 delta_tls = getbitu(content, 62, 8);
  u32 tot = getbitu(content, 70, 8);
  u32 wnot = getbitu(content, 78, 8);
  u32 wnlsf = getbitu(content, 86, 8);
  u32 dn = getbitu(content, 94, 3);
  u32 delta_tlsf = getbitu(content, 97, 8);
  u32 tow = getbitu(content, 105, 20);

  utc->a0 = BITS_SIGN_EXTEND_32(32, a0) * C_1_2P30;
  utc->a1 = BITS_SIGN_EXTEND_32(24, a1) * C_1_2P50;
  utc->a2 = 0;
  utc->tot.wn = (s16)wnot;
  utc->tot.tow = (double)(tot * 3600);
  utc->t_lse.wn = wnlsf;
  /* from ICD: Day Number at the _end_ of which... */
  utc->t_lse.tow = (double)(dn * DAY_SECS);
  normalize_gps_time(&utc->t_lse);
  utc->dt_ls = BITS_SIGN_EXTEND_32(8, delta_tls);
  utc->dt_lsf = BITS_SIGN_EXTEND_32(8, delta_tlsf);

  return tow;
}

static void parse_inav_eph(const nav_msg_gal_inav_t *nav_msg,
                           gal_inav_decoded_t *dd,
                           const gps_time_t *t_dec) {
  ephemeris_t *eph = &(dd->ephemeris);
  ephemeris_kepler_t *kep = &(dd->ephemeris.kepler);
  kep->iode = getbitu(nav_msg->raw_eph[0], 6, 10);
  kep->iodc = kep->iode;
  eph->fit_interval = GAL_FIT_INTERVAL_SECONDS;
  /* word type 1 */
  u32 toe = getbitu(nav_msg->raw_eph[0], 16, 14);
  u32 m0 = getbitu(nav_msg->raw_eph[0], 30, 32);
  u32 ecc = getbitu(nav_msg->raw_eph[0], 62, 32);
  u32 sqrta = getbitu(nav_msg->raw_eph[0], 94, 32);
  eph->toe.tow = toe * 60.0;
  kep->m0 = BITS_SIGN_EXTEND_32(32, m0) * C_1_2P31 * GPS_PI;
  kep->ecc = ecc * C_1_2P33;
  kep->sqrta = sqrta * C_1_2P19;
  /* word type 2 */
  u32 omega0 = getbitu(nav_msg->raw_eph[1], 16, 32);
  u32 i0 = getbitu(nav_msg->raw_eph[1], 48, 32);
  u32 omega = getbitu(nav_msg->raw_eph[1], 80, 32);
  u32 idot = getbitu(nav_msg->raw_eph[1], 112, 14);
  kep->omega0 = BITS_SIGN_EXTEND_32(32, omega0) * C_1_2P31 * GPS_PI;
  kep->inc = BITS_SIGN_EXTEND_32(32, i0) * C_1_2P31 * GPS_PI;
  kep->w = BITS_SIGN_EXTEND_32(32, omega) * C_1_2P31 * GPS_PI;
  kep->inc_dot = BITS_SIGN_EXTEND_32(14, idot) * C_1_2P43 * GPS_PI;
  /* word type 3 */
  u32 omegadot = getbitu(nav_msg->raw_eph[2], 16, 24);
  u32 deltan = getbitu(nav_msg->raw_eph[2], 40, 16);
  u32 cuc = getbitu(nav_msg->raw_eph[2], 56, 16);
  u32 cus = getbitu(nav_msg->raw_eph[2], 72, 16);
  u32 crc = getbitu(nav_msg->raw_eph[2], 88, 16);
  u32 crs = getbitu(nav_msg->raw_eph[2], 104, 16);
  u32 sisa = getbitu(nav_msg->raw_eph[2], 120, 8);
  kep->omegadot = BITS_SIGN_EXTEND_32(24, omegadot) * C_1_2P43 * GPS_PI;
  kep->dn = BITS_SIGN_EXTEND_32(16, deltan) * C_1_2P43 * GPS_PI;
  kep->cuc = BITS_SIGN_EXTEND_32(16, cuc) * C_1_2P29;
  kep->cus = BITS_SIGN_EXTEND_32(16, cus) * C_1_2P29;
  kep->crc = BITS_SIGN_EXTEND_32(16, crc) * C_1_2P5;
  kep->crs = BITS_SIGN_EXTEND_32(16, crs) * C_1_2P5;
  eph->ura = sisa_map(sisa);
  /* word type 4 */
  u32 sat = getbitu(nav_msg->raw_eph[3], 16, 6);
  u32 cic = getbitu(nav_msg->raw_eph[3], 22, 16);
  u32 cis = getbitu(nav_msg->raw_eph[3], 38, 16);
  u32 toc = getbitu(nav_msg->raw_eph[3], 54, 14);
  u32 af0 = getbitu(nav_msg->raw_eph[3], 68, 31);
  u32 af1 = getbitu(nav_msg->raw_eph[3], 99, 21);
  u32 af2 = getbitu(nav_msg->raw_eph[3], 120, 6);
  eph->sid.sat = sat;
  kep->cic = BITS_SIGN_EXTEND_32(16, cic) * C_1_2P29;
  kep->cis = BITS_SIGN_EXTEND_32(16, cis) * C_1_2P29;
  kep->toc.tow = toc * 60.0;
  kep->af0 = BITS_SIGN_EXTEND_32(31, af0) * C_1_2P34;
  kep->af1 = BITS_SIGN_EXTEND_32(21, af1) * C_1_2P46;
  kep->af2 = BITS_SIGN_EXTEND_32(6, af2) * C_1_2P59;

  /* Match TOE week number with the time of transmission, fixes the case
   * near week roll-over where time of ephemeris is across the week boundary */
  gps_time_match_weeks(&eph->toe, t_dec);
  gps_time_match_weeks(&kep->toc, t_dec);
}

static void parse_inav_alm3(nav_msg_gal_inav_t *nav_msg,
                            gal_inav_decoded_t *dd) {
  (void)nav_msg;
  (void)dd;
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
  /* copy the first 114 bit */
  u32 tmpw = getbitu(even, 6, 32);
  setbitu(temp, 0, 32, tmpw);
  tmpw = getbitu(even, 38, 32);
  setbitu(temp, 32, 32, tmpw);
  tmpw = getbitu(even, 70, 32);
  setbitu(temp, 64, 32, tmpw);
  tmpw = getbitu(even, 102, 18);
  setbitu(temp, 96, 18, tmpw);
  /* copy the remaining bits */
  tmpw = getbitu(odd, 6, 32);
  /* overwrite the tail bits of the even page */
  setbitu(temp, 114, 32, tmpw); /* 114 - 145 */
  tmpw = getbitu(odd, 38, 32);
  setbitu(temp, 114 + 32, 32, tmpw); /* 146 - 177 */
  tmpw = getbitu(odd, 70, 32);
  setbitu(temp, 114 + 64, 32, tmpw); /* 178 - 209 */

  u32 crc = crc24q_bits(0, temp, 196, invert);

  return crc;
}

static u32 gal_inav_extract_crc(const u8 odd[static GAL_INAV_PAGE_BYTE],
                                int inv) {
  bool invert = inv ? true : false;
  u32 crc = getbitu(odd, 88, 24);

  if (invert) {
    crc ^= 0xFFFFFF;
  }
  return crc;
}

static void extract_inav_content(u8 content[static GAL_INAV_CONTENT_BYTE],
                                 const u8 even[static GAL_INAV_PAGE_BYTE],
                                 const u8 odd[static GAL_INAV_PAGE_BYTE]) {
  memcpy(content, even + 1, 14);
  memcpy(content + 14, odd + 1, 2);
}

static bool eph_complete(nav_msg_gal_inav_t *nav_msg) {
  if ((GAL_INAV_IOD_INVALID != nav_msg->iod_nav[0]) &&
      (nav_msg->iod_nav[1] == nav_msg->iod_nav[0]) &&
      (nav_msg->iod_nav[2] == nav_msg->iod_nav[0]) &&
      (nav_msg->iod_nav[3] == nav_msg->iod_nav[0])) {
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

static float sisa_map(u8 sisa) {
  float ura = INVALID_URA_VALUE;
  sisa &= 0xff;
  if (sisa < 50) {
    ura = sisa * 0.01f;
  } else if (sisa < 75) {
    ura = 0.5f + (sisa - 50) * 0.02f;
  } else if (sisa < 100) {
    ura = 1.0f + (sisa - 75) * 0.04f;
  } else if (sisa < 126) {
    ura = 2.0f + (sisa - 100) * 0.16f;
  } else if (INVALID_GAL_SISA_INDEX != sisa) {
    /* Note: SISA Index 126...254 are considered as Spare. */
    ura = 6.0f;
  }
  if (URA_VALID(ura)) {
    ura = rintf(ura / 0.01f) * 0.01f;
  }
  return ura;
}

#if defined GAL_INAV_FEC_HARD
/** Shifts bits properly in the bit array */
static void inav_buffer_1bit_pushr(u32 buff[static GAL_INAV_DECODE_BUFF_SIZE],
                                   const bool bitval) {
  u8 k;

  ASSERT(buff);

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
