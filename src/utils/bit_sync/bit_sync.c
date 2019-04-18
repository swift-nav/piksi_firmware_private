/*
 * Copyright (c) 2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *          Pasi Miettinen <pasi.miettinen@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "bit_sync.h"

#include <assert.h>
#include <inttypes.h>
#include <math.h>
#include <string.h>
#include <swiftnav/bits.h>

#include "soft_macq/bds_prns.h"
#include "soft_macq/gal_prns.h"

/* Approx number of nav bit edges needed to accept bit sync for a
   strong signal (sync will take longer on a weak signal) */
#define BITSYNC_THRES_HI 11
#define BITSYNC_THRES_LO 3

#define SYMBOL_LENGTH_NH20_MS 20
#define GAL_CS100_MS 100
#define GAL_CS25_LEN 25
#define GAL_CS25_MS (GAL_CS25_LEN * GAL_E1C_PRN_PERIOD_MS)

/* NH20[0]  = [0 0 0 0  0 1 0 0  1 1 0 1  0 1 0 0  1 1 1 0 ] */
/* NH20[+2] = [1 0 0 0  0 0 0 1  0 0 1 1  0 1 0 1  0 0 1 1 ] */
static const u8 nh20_bits[INT_NUM_BYTES(SYMBOL_LENGTH_NH20_MS)] = {
    0x81, 0x35, 0x30};

/* Galileo E1C symbols upsampled to 1 ms, represented on bits and shifted right
 * by 2 */
static u8 e1c_sc_ms_bits[INT_NUM_BYTES(GAL_CS25_MS)] = {0x00,
                                                        0x3f,
                                                        0xfc,
                                                        0x00,
                                                        0x00,
                                                        0x00,
                                                        0x3c,
                                                        0x3c,
                                                        0x3f,
                                                        0xc3,
                                                        0xfc,
                                                        0x03,
                                                        0xc0};

/* Galileo E5aQ symbols represented on bits and shifted right by 2 */
static u8 e5q_sc_ms_bits[NUM_SATS_GAL][GAL_CS100_BYTES];

/* Galileo E5bQ symbols represented on bits and shifted right by 2 */
static u8 e7q_sc_ms_bits[NUM_SATS_GAL][GAL_CS100_BYTES];

/* Beidou3 B2aQ symbols represented on bits and shifted right by 2 */
static u8 c5q_sc_ms_bits[NUM_SATS_BDS][BDS3_SC100_BYTES];

static void histogram_update(bit_sync_t *b,
                             s32 corr_prompt_real,
                             s32 corr_prompt_imag);

/** \defgroup bit_sync Bit Sync
 * Functions and calculations related to data bit synchronization
 *
 * \{ */

/** Initialize a bit sync structure
 *
 * \param b      Pointer to a bit sync structure
 * \param mesid  ME signal identifier
 */
void bit_sync_init(bit_sync_t *b, const me_gnss_signal_t mesid) {
  assert(mesid_valid(mesid));
  memset(b, 0, sizeof(bit_sync_t));
  b->bit_phase_ref = BITSYNC_UNSYNCED;
  b->mesid = mesid;
  const u8 sat = mesid.sat - 1;
  u8 bit_length = 1;

  switch ((s8)mesid.code) {
    case CODE_GPS_L1CA:
    case CODE_AUX_GPS:
    case CODE_GPS_L2CM:
    case CODE_GPS_L5I:
    case CODE_QZS_L1CA:
    case CODE_AUX_QZS:
    case CODE_QZS_L2CM:
    case CODE_QZS_L5I:
      bit_length = GPS_L1CA_SYMBOL_LENGTH_MS;
      break;

    case CODE_GLO_L1OF:
    case CODE_GLO_L2OF:
      bit_length = GLO_L1CA_SYMBOL_LENGTH_MS;
      break;

    case CODE_SBAS_L1CA:
      bit_length = SBAS_L1CA_SYMBOL_LENGTH_MS;
      break;

    case CODE_BDS2_B1:
    case CODE_BDS2_B2:
      if (bds_d2nav(mesid)) {
        bit_length = BDS2_B11_D2NAV_SYMBOL_LENGTH_MS;
      } else {
        bit_length = BDS2_B11_D1NAV_SYMBOL_LENGTH_MS;
      }
      break;

    case CODE_GAL_E1B:
      bit_length = 4;
      break;

    case CODE_GAL_E5I:
      bit_length = 20;
      for (u8 i = 0; i < GAL_CS100_MS; i++) {
        u32 srci = (i + GAL_CS100_MS - 2) % GAL_CS100_MS;
        u32 bit = getbitu(gal_e5q_sec_codes[sat], srci, 1);
        setbitu(e5q_sc_ms_bits[sat], i, 1, bit);
      }
      break;

    case CODE_GAL_E7I:
      bit_length = 4;
      for (u8 i = 0; i < GAL_CS100_MS; i++) {
        u32 srci = (i + GAL_CS100_MS - 2) % GAL_CS100_MS;
        u32 bit = getbitu(gal_e7q_sec_codes[sat], srci, 1);
        setbitu(e7q_sc_ms_bits[sat], i, 1, bit);
      }
      break;

    case CODE_BDS3_B5I:
      bit_length = 20;
      for (u8 i = 0; i < BDS3_B5Q_SC_MS; i++) {
        u32 srci = (i + BDS3_B5Q_SC_MS - 2) % BDS3_B5Q_SC_MS;
        u32 bit = getbitu(bds3_b2aq_sec_codes[sat], srci, 1);
        setbitu(c5q_sc_ms_bits[sat], i, 1, bit);
      }
      break;

    default:
      log_error("Unsupported code type %d", mesid.code);
      assert(0);
      break;
  }
  b->bit_length = bit_length;
}

/** Force bit sync.
 * Sets the bit phase reference.
 *
 * \param b    Pointer to a bit sync structure.
 * \param bit_phase_ref Bit phase reference.
 */
void bit_sync_set(bit_sync_t *b, s8 bit_phase_ref) {
  b->bit_phase_ref = bit_phase_ref;
}

/** Update bit sync and get bit integration output
 *
 * \param b                 Pointer to a bit sync structure
 * \param corr_prompt_real  Real part of the prompt correlation
 * \param corr_prompt_imag  Imaginary part of the prompt correlation
 * \param ms                Integration time (ms) of the correlation
 * \param bit_integrate     Pointer to output bit integration (if valid)
 *
 * \return  True if *bit_integrate contains a valid bit integration,
 *          False otherwise
 */
bool bit_sync_update(bit_sync_t *b,
                     s32 corr_prompt_real,
                     s32 corr_prompt_imag,
                     u32 ms,
                     s32 *bit_integrate) {
  assert(ms <= b->bit_length && "Integration length exceeds symbol length");

  b->bit_phase += ms;
  b->bit_phase %= b->bit_length;
  b->bit_integrate += corr_prompt_real;

  /* Search for bit phase if not yet locked. */
  if (b->bit_phase_ref == BITSYNC_UNSYNCED) {
    if (ms > 1) {
      log_warn_mesid(b->mesid, "attempts bitsync with %" PRIu32 " ms", ms);
    }
    histogram_update(b, corr_prompt_real, corr_prompt_imag);
  }

  /* Return the integration at the end of the bit period */
  if (b->bit_phase == b->bit_phase_ref) {
    *bit_integrate = SIGN(b->bit_integrate);

    b->bit_integrate = 0;
    return true;
  }

  return false;
}

/** Update phase-difference based histogram
 *
 * \param b                 Pointer to a bit sync structure
 * \param corr_prompt_real  Real part of the prompt correlation
 * \param corr_prompt_imag  Imaginary part of the prompt correlation
 *
 */
static void histogram_update(bit_sync_t *b,
                             s32 corr_prompt_real,
                             s32 corr_prompt_imag) {
  /* compute the sign of the real part of the dot product
   * (to check if the angle has changed by more than PI/2) */
  s64 dot_prod_real =
      ((b->prev_real) * corr_prompt_real) + ((b->prev_imag) * corr_prompt_imag);
  bool nav_bit_change = (dot_prod_real < 0);
  const code_t c = b->mesid.code;
  const u8 sat = b->mesid.sat - 1;
  s16 sum = 0;

  if ((((CODE_BDS2_B1 == c) || (CODE_BDS2_B2 == c)) && !bds_d2nav(b->mesid)) ||
      (CODE_GPS_L5I == c)) {
    /* Codes with NH20 need perfect match */
    u8 last_symb = getbitu(b->symb_bit, (SYMBOL_LENGTH_NH20_MS - 1), 1);
    bitshl(b->symb_bit, INT_NUM_BYTES(SYMBOL_LENGTH_NH20_MS), 1);
    u32 new_symb = nav_bit_change ? 1 - last_symb : last_symb;
    setbitu(b->symb_bit, (SYMBOL_LENGTH_NH20_MS - 1), 1, new_symb);
    for (u8 i = 0; i < INT_NUM_BYTES(SYMBOL_LENGTH_NH20_MS); i++) {
      sum += count_bits_u8(b->symb_bit[i] ^ nh20_bits[i], 1);
    }
    sum = SYMBOL_LENGTH_NH20_MS - 2 * sum;
    if (ABS(sum) >= SYMBOL_LENGTH_NH20_MS - 1) {
      b->bit_phase_ref = (b->bit_phase + 2) % b->bit_length;
    }

  } else if (CODE_GAL_E7I == c) {
    /* Galileo E7Q has a SC100 secondary code */
    u8 last_symb = getbitu(b->symb_bit, (GAL_CS100_MS - 1), 1);
    bitshl(b->symb_bit, GAL_CS100_BYTES, 1);
    u32 new_symb = nav_bit_change ? 1 - last_symb : last_symb;
    setbitu(b->symb_bit, (GAL_CS100_MS - 1), 1, new_symb);
    for (u8 i = 0; i < GAL_CS100_BYTES; i++) {
      sum += count_bits_u8(b->symb_bit[i] ^ e7q_sc_ms_bits[sat][i], 1);
    }
    sum = GAL_CS100_MS - 2 * sum;
    if (ABS(sum) >= (3 * GAL_CS100_MS / 4)) {
      b->bit_phase_ref = (b->bit_phase + 2) % b->bit_length;
    }

  } else if (CODE_GAL_E5I == c) {
    /* Galileo E5Q has a SC100 secondary code */
    u8 last_symb = getbitu(b->symb_bit, (GAL_CS100_MS - 1), 1);
    bitshl(b->symb_bit, GAL_CS100_BYTES, 1);
    u32 new_symb = nav_bit_change ? 1 - last_symb : last_symb;
    setbitu(b->symb_bit, (GAL_CS100_MS - 1), 1, new_symb);
    for (u8 i = 0; i < GAL_CS100_BYTES; i++) {
      sum += count_bits_u8(b->symb_bit[i] ^ e5q_sc_ms_bits[sat][i], 1);
    }
    sum = GAL_CS100_MS - 2 * sum;
    if (ABS(sum) >= (3 * GAL_CS100_MS / 4)) {
      b->bit_phase_ref = (b->bit_phase + 2) % b->bit_length;
    }

  } else if (CODE_GAL_E1B == c) {
    /* Galileo E1C has a SC25 secondary code */
    u8 last_symb = getbitu(b->symb_bit, (GAL_CS25_MS - 1), 1);
    bitshl(b->symb_bit, INT_NUM_BYTES(GAL_CS25_MS), 1);
    u32 new_symb = nav_bit_change ? 1 - last_symb : last_symb;
    setbitu(b->symb_bit, (GAL_CS25_MS - 1), 1, new_symb);
    for (u8 i = 0; i < INT_NUM_BYTES(GAL_CS25_MS); i++) {
      sum += count_bits_u8(b->symb_bit[i] ^ e1c_sc_ms_bits[i], 1);
    }
    sum = GAL_CS25_MS - 2 * sum;
    if (ABS(sum) >= GAL_CS25_MS - 1) {
      b->bit_phase_ref = (b->bit_phase + 2) % b->bit_length;
    }

  } else if (CODE_BDS3_B5I == c) {
    /* Beidou3 B2aQ has a SC100 secondary code */
    u8 last_symb = getbitu(b->symb_bit, (BDS3_SC100_BYTES - 1), 1);
    bitshl(b->symb_bit, INT_NUM_BYTES(BDS3_SC100_BYTES), 1);
    u32 new_symb = nav_bit_change ? 1 - last_symb : last_symb;
    setbitu(b->symb_bit, (BDS3_SC100_BYTES - 1), 1, new_symb);
    for (u8 i = 0; i < INT_NUM_BYTES(BDS3_SC100_BYTES); i++) {
      sum += count_bits_u8(b->symb_bit[i] ^ c5q_sc_ms_bits[sat][i], 1);
    }
    sum = BDS3_SC100_BYTES - 2 * sum;
    if (ABS(sum) >= (3 * BDS3_SC100_BYTES / 4)) {
      b->bit_phase_ref = (b->bit_phase + 2) % b->bit_length;
    }

  } else {
    /* check one in the histogram if the above is negative */
    b->histogram[b->bit_phase] += nav_bit_change;

    /* Find the two highest values. */
    s8 max = 0, next_best = 0;
    u8 max_i = 0;
    for (u8 i = 0; i < (b->bit_length); i++) {
      s8 v = b->histogram[i];
      if (v > max) {
        next_best = max;
        max = v;
        max_i = i;
      } else if (v > next_best) {
        next_best = v;
      }
    }

    /* Check best and second-best */
    if (max > BITSYNC_THRES_HI) {
      if (next_best < BITSYNC_THRES_LO) {
        /* We are synchronized! */
        b->bit_phase_ref = ((max_i + (b->bit_length) - 1) % b->bit_length);
      } else {
        /* FIXME: resetting te histogram is a bit brutal.. */
        memset(b->histogram, 0, sizeof(b->histogram));
      }
    }
  }

  b->prev_real = corr_prompt_real;
  b->prev_imag = corr_prompt_imag;
}

/** \} */
