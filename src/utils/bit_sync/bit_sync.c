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

#include "soft_macq/gal_prns.h"

/* Approx number of nav bit edges needed to accept bit sync for a
   strong signal (sync will take longer on a weak signal) */
#define BITSYNC_THRES_HI 11
#define BITSYNC_THRES_LO 3

#define SYMBOL_LENGTH_NH20_MS 20
#define GAL_CS100_MS 100
#define GAL_CS25_LEN 25
#define GAL_CS25_MS (GAL_CS25_LEN * GAL_E1C_PRN_PERIOD_MS)

#define HS(n) ((b->histogram[n] > 0) ? '+' : '-')

/* The sync histogram should be as follows for NH20 code
 * NH20 = [0 0 0 0 0  1 0 0 1 1  0 1 0 1 0  0 1 1 1 0]
 * XANS = 0,0,0,0,0, 1,1,0,1,0, 1,1,1,1,1, 0,1,0,0,1
 */
static const s8 nh20_xans[SYMBOL_LENGTH_NH20_MS] = {+1, +1, +1, +1, +1, -1, -1,
                                                    +1, -1, +1, -1, -1, -1, -1,
                                                    -1, +1, -1, +1, +1, -1};

/* Galileo E1C transitions array, common to all satellites */
static s8 e1c_xans[GAL_CS25_MS];

/* Galileo E5aQ transitions array, built per satellite */
static s8 e5q_xans[NUM_SATS_GAL][GAL_CS100_MS];

/* Galileo E5bQ transitions array, built per satellite */
static s8 e7q_xans[NUM_SATS_GAL][GAL_CS100_MS];

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
  u8 sat = mesid.sat - 1;
  u8 bit_length = 1;
  u8 prev_chip, curr_chip;

  switch ((s8)mesid.code) {
    case CODE_GPS_L1CA:
    case CODE_GPS_L2CM:
    case CODE_GPS_L5I:
    case CODE_QZS_L1CA:
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
      /* TODO: add GAL_CS100_MS to constants.h in LSNP, or me_constants.h */
      prev_chip = getbitu(gal_e1c_sec25, GAL_CS25_LEN - 1, 1);
      for (u8 sec_chip_idx = 0; sec_chip_idx < GAL_CS25_LEN; sec_chip_idx++) {
        curr_chip = getbitu(gal_e1c_sec25, sec_chip_idx, 1);
        e1c_xans[sec_chip_idx * GAL_E1C_PRN_PERIOD_MS + 0] =
            (curr_chip != prev_chip) ? -1 : +1;
        e1c_xans[sec_chip_idx * GAL_E1C_PRN_PERIOD_MS + 1] = +1;
        e1c_xans[sec_chip_idx * GAL_E1C_PRN_PERIOD_MS + 2] = +1;
        e1c_xans[sec_chip_idx * GAL_E1C_PRN_PERIOD_MS + 3] = +1;
        prev_chip = curr_chip;
      }
      break;

    case CODE_GAL_E5I:
      bit_length = 20;
      /* TODO: add GAL_CS100_MS to constants.h in LSNP, or me_constants.h */
      prev_chip = getbitu(gal_e5q_sec_codes[sat], GAL_CS100_MS - 1, 1);
      for (u8 sec_chip_idx = 0; sec_chip_idx < GAL_CS100_MS; sec_chip_idx++) {
        curr_chip = getbitu(gal_e5q_sec_codes[sat], sec_chip_idx, 1);
        e5q_xans[sat][sec_chip_idx] = (curr_chip != prev_chip) ? -1 : +1;
        prev_chip = curr_chip;
      }
      break;

    case CODE_GAL_E7I:
      bit_length = 4;
      /* TODO: add GAL_CS100_MS to constants.h in LSNP, or me_constants.h */
      prev_chip = getbitu(gal_e7q_sec_codes[sat], GAL_CS100_MS - 1, 1);
      for (u8 sec_chip_idx = 0; sec_chip_idx < GAL_CS100_MS; sec_chip_idx++) {
        curr_chip = getbitu(gal_e7q_sec_codes[sat], sec_chip_idx, 1);
        e7q_xans[sat][sec_chip_idx] = (curr_chip != prev_chip) ? -1 : +1;
        prev_chip = curr_chip;
      }
      break;

    default:
      assert(!"Unsupported code type");
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

  if ((IS_BDS2(b->mesid) && !bds_d2nav(b->mesid)) ||
      (CODE_GPS_L5I == b->mesid.code) || (CODE_GPS_L5Q == b->mesid.code)) {
    /* Codes with NH20 are a little special */

    /* FIXME: resetting the histogram is a bit brutal.. */
    if (ABS(b->histogram[0]) > 12) {
      memset(b->histogram, 0, sizeof(b->histogram));
    }
    /* rotate the histogram left */
    s8 hist_head = b->histogram[0];
    memmove(&(b->histogram[0]),
            &(b->histogram[1]),
            sizeof(s8) * (SYMBOL_LENGTH_NH20_MS - 1));
    /* adjust the histogram element if there was a transition */
    hist_head += SIGN(dot_prod_real);
    b->histogram[SYMBOL_LENGTH_NH20_MS - 1] = hist_head;
    s32 sum = 0;
    /* cross-correlate transitions at the current symbol */
    /* transitions on the first element shouldn't count: it's data */
    for (u8 i = 2; i < SYMBOL_LENGTH_NH20_MS; i++) {
      sum += b->histogram[i] * (nh20_xans[i - 2]);
    }
    if (sum >= 9 * SYMBOL_LENGTH_NH20_MS) {
      /* We are synchronized! */
      b->bit_phase_ref = (b->bit_phase + 2) % b->bit_length;
    }

  } else if (CODE_GAL_E7I == b->mesid.code) {
    /* Galileo E7Q has a SC100 secondary code */
    if (ABS(b->histogram[0]) > 3) {
      memset(b->histogram, 0, sizeof(b->histogram));
    }
    s8 hist_head = b->histogram[0];
    /* rotate the histogram left */
    memmove(&(b->histogram[0]),
            &(b->histogram[1]),
            sizeof(s8) * (GAL_CS100_MS - 1));
    /* if there was a transition subtract 1 */
    hist_head += SIGN(dot_prod_real);
    b->histogram[(GAL_CS100_MS - 1)] = hist_head;
    s32 sum = 0;
    u8 sat = b->mesid.sat - 1;
    /* cross-correlate transitions at the current symbol */
    /* FPGA is working on new bit already: need to anticipate by 1 */
    for (u8 i = 2; i < GAL_CS100_MS; i++) {
      sum += b->histogram[i] * (e7q_xans[sat][i - 2]);
    }
    if (sum >= (2 * GAL_CS100_MS)) {
      /* We are synchronized! */
      /* might be a +2 or a -2.. we'll know when we do the same for NH20s */
      b->bit_phase_ref = (b->bit_phase + 2) % b->bit_length;
    }

  } else if (CODE_GAL_E5I == b->mesid.code) {
    /* TODO: this is untested */
    /* Galileo E5Q has a SC100 secondary code */
    if (ABS(b->histogram[0]) > 3) {
      memset(b->histogram, 0, sizeof(b->histogram));
    }
    s8 hist_head = b->histogram[0];
    /* rotate the histogram left */
    memmove(&(b->histogram[0]),
            &(b->histogram[1]),
            sizeof(s8) * (GAL_CS100_MS - 1));
    /* if there was a transition subtract 1 */
    hist_head += SIGN(dot_prod_real);
    b->histogram[(GAL_CS100_MS - 1)] = hist_head;
    s32 sum = 0;
    u8 sat = b->mesid.sat - 1;
    /* cross-correlate transitions at the current symbol */
    /* transitions on the first element do count: it's a pure pilot channel */
    for (u8 i = 2; i < GAL_CS100_MS; i++) {
      sum += b->histogram[i] * (e5q_xans[sat][i - 2]);
    }
    if (sum >= (2 * GAL_CS100_MS)) {
      b->bit_phase_ref = (b->bit_phase + 2) % b->bit_length;
    }

  } else if (CODE_GAL_E1B == b->mesid.code) {
    /* Galileo E1C has a SC25 secondary code */
    s8 hist_head = b->histogram[0];
    /* rotate the histogram left */
    memmove(
        &(b->histogram[0]), &(b->histogram[1]), sizeof(s8) * (GAL_CS25_MS - 1));
    /* if there was a transition subtract 1 */
    hist_head = SIGN(dot_prod_real);
    b->histogram[(GAL_CS25_MS - 1)] = hist_head;
    s32 sum = 0;
    /* cross-correlate transitions at the current symbol */
    /* transitions on the first element do count: it's a pure pilot channel
     */
    for (u8 i = 0; i < GAL_CS25_MS; i++) {
      sum += b->histogram[i] * (e1c_xans[(i - 2 + GAL_CS25_MS) % GAL_CS25_MS]);
    }
    if (sum == GAL_CS25_MS) {
      b->bit_phase_ref = (b->bit_phase + 2) % b->bit_length;
    }
  } else {
    /* check one in the histogram if the above is negative */
    if (dot_prod_real < 0) {
      b->histogram[b->bit_phase] += 1;
    }

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
