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

#include <libswiftnav/common.h>
#include <string.h>

#include "lockd.h"

static float lockd_lpf_update(struct lockd_lpf *lpf, float x, float fc) {
  lpf->y += fc * (x - lpf->y);
  return lpf->y;
}

/** Initialise the lock detector state.
 * \param l Lock detector state
 * \param fc Filter coefficient
 * \param scale Signal scale factor
 * \param cnt_threshodl Lock count threshold
 */
void lockd_init(lockd_t *l, float fc, float scale, u16 cnt_threshold) {
  memset(l, 0, sizeof(*l));
  lockd_reinit(l, fc, scale, cnt_threshold);
}

/** Update the lock detector parameters, preserving internal state.
 * \param l Lock detector state
 * \param fc Filter coefficient
 * \param scale Signal scale factor
 * \param cnt_threshodl Lock count threshold
 */
void lockd_reinit(lockd_t *l, float fc, float scale, u16 cnt_threshold) {
  l->fc = fc;
  l->scale = scale;
  l->cnt_threshold = cnt_threshold;
}

/** Update the lock detector with new signal and noise readings.
 * \param l Lock detector state
 * \param signal Signal reading.
 * \param noise Noise reading.
 *
 * References:
 *  -# Understanding GPS: Principles and Applications, 2nd Edition
 *     Section 5.11.2, pp 233-235
 *     Elliott D. Kaplan. Artech House, 1996.
 */
void lockd_update(lockd_t *l, float signal, float noise) {
  float signalf = lockd_lpf_update(&l->lpfs, signal, l->fc) * l->scale;
  float noisef = lockd_lpf_update(&l->lpfn, noise, l->fc);

  if (signalf > noisef) {
    /* Wait before raising the lock indicator */
    if (l->cnt > l->cnt_threshold) {
      l->locked = true;
    } else {
      l->cnt++;
    }
  } else {
    l->locked = false;
    l->cnt = 0;
  }
}
