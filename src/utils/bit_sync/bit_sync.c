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

#include <assert.h>
#include <math.h>
#include <string.h>

#include <libswiftnav/bits.h>
#include "soft_macq/gal_prns.h"

#include "bit_sync.h"

/* Approx number of nav bit edges needed to accept bit sync for a
   strong signal (sync will take longer on a weak signal) */
#define BITSYNC_THRES_HI 5
#define BITSYNC_THRES_LO 2

#define SYMBOL_LENGTH_NH20_MS 20
#define GAL_CS100_MS 100

/* The sync hisotgram should be as follows for NH20 code
 * NH20 = [0 0 0 0 0  1 0 0 1 1  0 1 0 1 0  0 1 1 1 0]
 * XANS = 0,0,0,0,0, 1,1,0,1,0, 1,1,1,1,1, 0,1,0,0,1
 */
static const s8 nh20_xans[20] = {+1, +1, +1, +1, +1, -1, -1, +1, -1, +1,
                                 -1, -1, -1, -1, -1, +1, -1, +1, +1, -1};

/* The sync histogram should be as follows for CS4 code
 * CS4  = [1 1 1 0]
 * XANS = 1,0,0,1
 */
//~ static const s8 e7i_xans[4] = {-1, +1, +1, -1};

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

  switch (mesid.code) {
    case CODE_GPS_L1CA:
    case CODE_GPS_L2CM:
    case CODE_GPS_L2CL:
    case CODE_GPS_L5I:
    case CODE_GPS_L5Q:
    case CODE_QZS_L1CA:
    case CODE_QZS_L2CM:
    case CODE_QZS_L2CL:
    case CODE_QZS_L5I:
    case CODE_QZS_L5Q:
      bit_length = GPS_L1CA_SYMBOL_LENGTH_MS;
      break;

    case CODE_GLO_L1OF:
    case CODE_GLO_L2OF:
      bit_length = GLO_L1CA_SYMBOL_LENGTH_MS;
      break;

    case CODE_SBAS_L1CA:
      bit_length = SBAS_L1CA_SYMBOL_LENGTH_MS;
      break;

    case CODE_BDS2_B11:
    case CODE_BDS2_B2:
      if (bds_d2nav(mesid)) {
        bit_length = BDS2_B11_D2NAV_SYMBOL_LENGTH_MS;
      } else {
        bit_length = BDS2_B11_D1NAV_SYMBOL_LENGTH_MS;
      }
      break;

    case CODE_GAL_E5X:
      bit_length = 20;
      /* TODO: add this GAL_CS100_MS to constants.h in LSNP, or me_constants.h
       */
      prev_chip = getbitu(gal_e5q_sec_codes[sat], GAL_CS100_MS - 1, 1);
      for (u8 sec_chip_idx = 0; sec_chip_idx < GAL_CS100_MS; sec_chip_idx++) {
        curr_chip = getbitu(gal_e5q_sec_codes[sat], sec_chip_idx, 1);
        e5q_xans[sat][sec_chip_idx] = (curr_chip != prev_chip) ? -1 : +1;
        prev_chip = curr_chip;
      }
      break;
    case CODE_GAL_E7X:
      bit_length = 4;
      /* TODO: add this GAL_CS100_MS to constants.h in LSNP, or me_constants.h
       */
      prev_chip = getbitu(gal_e7q_sec_codes[sat], GAL_CS100_MS - 1, 1);
      for (u8 sec_chip_idx = 0; sec_chip_idx < GAL_CS100_MS; sec_chip_idx++) {
        curr_chip = getbitu(gal_e7q_sec_codes[sat], sec_chip_idx, 1);
        e7q_xans[sat][sec_chip_idx] = (curr_chip != prev_chip) ? -1 : +1;
        prev_chip = curr_chip;
      }
      break;

    case CODE_GPS_L1P:
    case CODE_GPS_L2P:
    case CODE_GPS_L2CX:
    case CODE_GPS_L5X:
    case CODE_GAL_E1B:
    case CODE_GAL_E1C:
    case CODE_GAL_E1X:
    case CODE_GAL_E6B:
    case CODE_GAL_E6C:
    case CODE_GAL_E6X:
    case CODE_GAL_E7I:
    case CODE_GAL_E7Q:
    case CODE_GAL_E8:
    case CODE_GAL_E5I:
    case CODE_GAL_E5Q:
    case CODE_QZS_L2CX:
    case CODE_QZS_L5X:
    case CODE_INVALID:
    case CODE_COUNT:
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

    /* FIXME: resetting te histogram is a bit brutal.. */
    if (ABS(b->histogram[0]) > 9) {
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
    for (u8 i = 1; i < SYMBOL_LENGTH_NH20_MS; i++) {
      sum += b->histogram[i] * (nh20_xans[i]);
    }
    if (sum >= 100) {
      /* We are synchronized! */
      b->bit_phase_ref = b->bit_phase;
    }

  } else if (CODE_GAL_E7X == b->mesid.code) {
    /* Galileo E7Q has a SC100 secondary code */

    /* FIXME: resetting the histogram is a bit brutal.. */
    if (ABS(b->histogram[0]) > 3) {
      memset(b->histogram, 0, sizeof(b->histogram));
    }
    /* rotate the histogram left */
    s8 hist_head = b->histogram[0];
    memmove(&(b->histogram[0]),
            &(b->histogram[1]),
            sizeof(s8) * (GAL_CS100_MS - 1));
    /* if there was a transition subtract 1 */
    hist_head += SIGN(dot_prod_real);
    b->histogram[(GAL_CS100_MS - 1)] = hist_head;
    s32 sum = 0;
    u8 sat = b->mesid.sat - 1;
    /* cross-correlate transitions at the current symbol */
    /* the FPGA is working on the new bit already, so one needs to anticipate by
     * 1 */
    for (u8 i = 2; i < GAL_CS100_MS; i++) {
      sum += b->histogram[i] * (e7q_xans[sat][i - 2]);
    }
    if (sum >= (2 * GAL_CS100_MS)) {
      /* We are synchronized! */
      /* might be a +2 or a -2.. we'll know when we do the same for NH20s */
      b->bit_phase_ref = (b->bit_phase + 2) % b->bit_length;
    }

  } else if (CODE_GAL_E5X == b->mesid.code) {
    /* Galileo E5Q has a SC100 secondary code */

    /* FIXME: resetting the histogram is a bit brutal.. */
    if (ABS(b->histogram[0]) > 3) {
      memset(b->histogram, 0, sizeof(b->histogram));
    }
    /* rotate the histogram left */
    s8 hist_head = b->histogram[0];
    memmove(&(b->histogram[0]),
            &(b->histogram[1]),
            sizeof(s8) * (GAL_CS100_MS - 1));
    /* if there was a transition subtract 1 */
    hist_head += SIGN(dot_prod_real);
    b->histogram[(GAL_CS100_MS - 1)] = hist_head;
    s32 sum = 0;
    u8 sat = b->mesid.sat - 1;
    /* cross-correlate transitions at the current symbol */
    /* transitions on the first element shouldn't count: it's data */
    for (u8 i = 1; i < GAL_CS100_MS; i++) {
      sum += b->histogram[i] * (e5q_xans[sat][i]);
    }
    if (sum >= (2 * GAL_CS100_MS)) {
      /* might be a +2 or a -2.. we'll know when we do the same for NH20s */
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
