/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Tommi Paakki <tpaakki@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#include "track_utils.h"

/** Update the frequency lock detector with frequency error.
 * \param l   Lock detector state structure.
 * \param err Frequency error of FLL.
 *
 */
void freq_lock_detect_update(lock_detect_t *l, float err) {
  /* Calculate filtered frequency error */
  l->lpfi.y += l->k1 * (err - l->lpfi.y);

  /* Saturate filter */
  if (l->lpfi.y > TP_FLL_SATURATION_THRESHOLD_HZ) {
    l->lpfi.y = TP_FLL_SATURATION_THRESHOLD_HZ;
  } else if (l->lpfi.y < -TP_FLL_SATURATION_THRESHOLD_HZ) {
    l->lpfi.y = -TP_FLL_SATURATION_THRESHOLD_HZ;
  }

  if (fabsf(l->lpfi.y) < TP_FLL_ERR_THRESHOLD_HZ) {
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
