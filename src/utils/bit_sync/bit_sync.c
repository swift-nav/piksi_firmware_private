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

#include <libswiftnav/gnss_capabilities.h>

#include "bit_sync.h"

/* Approx number of nav bit edges needed to accept bit sync for a
   strong signal (sync will take longer on a weak signal) */
#define BITSYNC_THRES_HI 11
#define BITSYNC_THRES_LO 3

/* Symbol lengths for different constellations. Bounded by BIT_LENGTH_MAX */
#define SYMBOL_LENGTH_GPS_MS 20
#define SYMBOL_LENGTH_GLO_MS 10
#define SYMBOL_LENGTH_SBAS_L1_MS 2
#define SYMBOL_LENGTH_BDS_D1NAV_MS 20
#define SYMBOL_LENGTH_BDS_D2NAV_MS 2
#define SYMBOL_LENGTH_NH20_MS 20

/* The sync hisotgram should be as follows for Beidou2 NH20 code
 * NH20 = [0 0 0 0 0 1 0 0 1 1 0 1 0 1 0 0 1 1 1 0]
 * XANS = ?,0,0,0,0,1,1,0,1,0,1,1,1,1,1,0,1,0,0,1
 */
static const s8 nh20_xans[20] = {0, 0, 0, 0, 0, 1, 1, 0, 1, 0,
                                 1, 1, 1, 1, 1, 0, 1, 0, 0, 1};

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

  u8 bit_length = 1;
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
      bit_length = SYMBOL_LENGTH_GPS_MS;
      break;

    case CODE_GLO_L1OF:
    case CODE_GLO_L2OF:
      bit_length = SYMBOL_LENGTH_GLO_MS;
      break;

    case CODE_SBAS_L1CA:
      bit_length = SYMBOL_LENGTH_SBAS_L1_MS;
      break;

    case CODE_BDS2_B11:
    case CODE_BDS2_B2:
      if (bds_d2nav(mesid)) {
        bit_length = SYMBOL_LENGTH_BDS_D2NAV_MS;
      } else {
        bit_length = SYMBOL_LENGTH_BDS_D1NAV_MS;
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
    case CODE_GAL_E7X:
    case CODE_GAL_E8:
    case CODE_GAL_E5I:
    case CODE_GAL_E5Q:
    case CODE_GAL_E5X:
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

    /* rotate the histogram left */
    s8 hist_head = b->bitsync_histogram[0];
    if (2 * BITSYNC_THRES_HI < hist_head) {
      /* bound the hist values by scaling large ones */
      hist_head >>= 1;
    }
    memmove(&(b->bitsync_histogram[0]),
            &(b->bitsync_histogram[1]),
            sizeof(s8) * (SYMBOL_LENGTH_NH20_MS - 1));
    /* adjust the histogram element if there was a transition */
    hist_head += SIGN(dot_prod_real);
    b->bitsync_histogram[SYMBOL_LENGTH_NH20_MS - 1] = hist_head;
    s32 sum = 0;
    for (u8 i = 1; i < SYMBOL_LENGTH_NH20_MS; i++) {
      /* transitions on the first element don't count: they are data */
      sum += b->bitsync_histogram[i] * (1 - 2 * nh20_xans[i]);
    }
    if (sum >= BITSYNC_THRES_HI * SYMBOL_LENGTH_NH20_MS) {
      /* We are synchronized! */
      log_info_mesid(b->mesid,
                     "BSYNC"
                     " %+3d %+3d %+3d %+3d %+3d %+3d %+3d %+3d %+3d %+3d"
                     " %+3d %+3d %+3d %+3d %+3d %+3d %+3d %+3d %+3d %+3d",
                     b->bitsync_histogram[0],
                     b->bitsync_histogram[1],
                     b->bitsync_histogram[2],
                     b->bitsync_histogram[3],
                     b->bitsync_histogram[4],
                     b->bitsync_histogram[5],
                     b->bitsync_histogram[6],
                     b->bitsync_histogram[7],
                     b->bitsync_histogram[8],
                     b->bitsync_histogram[9],
                     b->bitsync_histogram[10],
                     b->bitsync_histogram[11],
                     b->bitsync_histogram[12],
                     b->bitsync_histogram[13],
                     b->bitsync_histogram[14],
                     b->bitsync_histogram[15],
                     b->bitsync_histogram[16],
                     b->bitsync_histogram[17],
                     b->bitsync_histogram[18],
                     b->bitsync_histogram[19]);
      b->bit_phase_ref = b->bit_phase;
    }

  } else {
    /* check one in the histogram if the above is negative */
    if (dot_prod_real < 0) {
      b->bitsync_histogram[b->bit_phase] += 1;
    }

    /* Find the two highest values. */
    s8 max = 0, next_best = 0;
    u8 max_i = 0;
    for (u8 i = 0; i < (b->bit_length); i++) {
      s8 v = b->bitsync_histogram[i];
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
        memset(b->bitsync_histogram, 0, sizeof(b->bitsync_histogram));
      }
    }
  }

  b->prev_real = corr_prompt_real;
  b->prev_imag = corr_prompt_imag;
}

/** \} */
