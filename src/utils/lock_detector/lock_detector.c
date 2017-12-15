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

/** Initialise the lock detector state.
 * \param l
 * \param k1 LPF coefficient.
 * \param k2 I Scale factor.
 * \param lp Pessimistic count threshold.
 * \param lo Optimistic count threshold
 */
void lock_detect_init(lock_detect_t *l,
                      float k1,
                      float k2,
                      u16 lp, 
                      u16 lo) {
  memset(l, 0, sizeof(*l)); /* sets l->lpf[iq].y = 0 */
  lock_detect_reinit(l, k1, k2, lp, lo);
}

/** Update the lock detector parameters, preserving internal state.
 * \param l
 * \param k1 LPF coefficient.
 * \param k2 I Scale factor.
 * \param lp Pessimistic count threshold.
 * \param lo Optimistic count threshold
 */
void lock_detect_reinit(lock_detect_t *l,
                        float k1,
                        float k2,
                        u16 lp, 
                        u16 lo) {
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
void lock_detect_update(lock_detect_t *l,
                        float I,
                        float Q,
                        float DT) {
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
