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

#include <math.h>
#include <string.h>

#include "lock_detector.h"

/* FLL saturation threshold in [Hz]. When signal is lost, filtered frequency
 * error can grow fast.
 * When the signal comes back, the saturation threshold helps the filter to
 * converge quickly below error threshold.
 */
#define TP_FLL_SATURATION_THRESHOLD_HZ (20.f)

/* FLL error threshold in [Hz]. Used to assess FLL frequency lock.
 * The threshold should be less than the expected aliased frequency, < 25 Hz.
 * Another factor is to avoid false positives from high dynamics.
 */
#define TP_FLL_ERR_THRESHOLD_HZ (10.f)

/** Initialise the PLL lock detector state.
 * \param l
 * \param k1 LPF coefficient.
 * \param k2 I Scale factor.
 * \param lp Pessimistic count threshold.
 * \param lo Optimistic count threshold
 */
void pll_lock_detect_init(
    lock_detect_t *l, float k1, float k2, u16 lp, u16 lo) {
  memset(l, 0, sizeof(*l)); /* sets l->lpf[iq].y = 0 */
  lock_detect_reinit(l, k1, k2, lp, lo);
}

/** Initialise the FLL lock detector state.
 * \param l
 * \param k1 LPF coefficient.
 * \param k2 I Scale factor.
 * \param lp Pessimistic count threshold.
 * \param lo Optimistic count threshold
 */
void fll_lock_detect_init(
    lock_detect_t *l, float k1, float k2, u16 lp, u16 lo) {
  memset(l, 0, sizeof(*l));
  lock_detect_reinit(l, k1, k2, lp, lo);
  l->lpfi.y = TP_FLL_SATURATION_THRESHOLD_HZ;  /* init to positive freq err */
  l->lpfq.y = -TP_FLL_SATURATION_THRESHOLD_HZ; /* init to negative freq err */
}

/** Update the lock detector parameters, preserving internal state.
 * \param l
 * \param k1 LPF coefficient.
 * \param k2 I Scale factor.
 * \param lp Pessimistic count threshold.
 * \param lo Optimistic count threshold
 */
void lock_detect_reinit(lock_detect_t *l, float k1, float k2, u16 lp, u16 lo) {
  /* Adjust LPF coefficient */
  l->k1 = k1;
  l->k2 = k2;
  l->lp = lp;
  l->lo = lo;
}

static float lock_detect_lpf_update(struct loop_detect_lpf *lpf,
                                    float x,
                                    float k1) {
  lpf->y += k1 * (x - lpf->y);
  return lpf->y;
}

/** Update the lock detector with new prompt correlations.
 * \param l
 * \param I In-phase prompt correlation.
 * \param Q Quadrature prompt correlation.
 * \param DT Integration time
 *
 * References:
 *  -# Understanding GPS: Principles and Applications, 2nd Edition
 *     Section 5.11.2, pp 233-235
 *     Elliott D. Kaplan. Artech House, 1996.
 */
void pll_lock_detect_update(lock_detect_t *l, float I, float Q, float DT) {
  float a, b;
  /* Calculated low-pass filtered prompt correlations */
  a = lock_detect_lpf_update(&l->lpfi, fabsf(I) / DT, l->k1) / l->k2;
  b = lock_detect_lpf_update(&l->lpfq, fabsf(Q) / DT, l->k1);

  if (a > b) {
    /* In-phase > quadrature, looks like we're locked */
    l->outo = true;
    l->pcount2 = 0;
    /* Wait before raising the pessimistic indicator */
    if (l->pcount1 > l->lp) {
      l->outp = true;
    } else {
      l->pcount1++;
    }
  } else {
    /* In-phase < quadrature, looks like we're not locked */
    l->outp = false;
    l->pcount1 = 0;
    /* Wait before lowering the optimistic indicator */
    if (l->pcount2 > l->lo) {
      l->outo = false;
    } else {
      l->pcount2++;
    }
  }
}

/** Update the frequency lock detector with frequency error.
 * \param l   Lock detector state structure.
 * \param err Frequency error of FLL.
 *
 */
void fll_lock_detect_update(lock_detect_t *l, float err) {
  /* Calculate filtered frequency error */
  lock_detect_lpf_update(&l->lpfi, err, l->k1);
  lock_detect_lpf_update(&l->lpfq, err, l->k1);

  /* Saturate filter */
  if (l->lpfi.y > TP_FLL_SATURATION_THRESHOLD_HZ) {
    l->lpfi.y = TP_FLL_SATURATION_THRESHOLD_HZ;
  } else if (l->lpfi.y < -TP_FLL_SATURATION_THRESHOLD_HZ) {
    l->lpfi.y = -TP_FLL_SATURATION_THRESHOLD_HZ;
  }

  if (l->lpfq.y > TP_FLL_SATURATION_THRESHOLD_HZ) {
    l->lpfq.y = TP_FLL_SATURATION_THRESHOLD_HZ;
  } else if (l->lpfq.y < -TP_FLL_SATURATION_THRESHOLD_HZ) {
    l->lpfq.y = -TP_FLL_SATURATION_THRESHOLD_HZ;
  }

  if ((fabsf(l->lpfi.y) < TP_FLL_ERR_THRESHOLD_HZ) &&
      (fabsf(l->lpfq.y) < TP_FLL_ERR_THRESHOLD_HZ)) {
    /* error < threshold, looks like we're locked */
    l->outo = true;
    l->pcount2 = 0;
    /* Wait before raising the pessimistic indicator */
    if (l->pcount1 > l->lp) {
      l->outp = true;
    } else {
      l->pcount1++;
    }
  } else {
    /* error >= threshold, looks like we're not locked */
    l->outp = false;
    l->pcount1 = 0;
    /* Wait before lowering the optimistic indicator */
    if (l->pcount2 > l->lo) {
      l->outo = false;
    } else {
      l->pcount2++;
    }
  }
}
