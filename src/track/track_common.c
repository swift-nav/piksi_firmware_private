/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Adel Mamin <adel.mamin@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "math.h"

#include <libswiftnav/track.h>
#include "track.h"

/** Runs alias detection logic
 * \param alias_detect Alias detector's state
 * \param I the value of in-phase arm of the correlator
 * \param Q the value of quadrature arm of the correlator
 * \return The frequency error of PLL [Hz]
 */
s16 tp_tl_detect_alias(alias_detect_t *alias_detect, s32 I, s32 Q)
{
  float err = alias_detect_second(alias_detect, I, Q);
  float abs_err = fabsf(err);
  int err_sign = (err > 0) ? 1 : -1;
  s16 correction;

  /* The expected frequency errors are +-(25 + N * 50) Hz
     For more details, see:
     https://swiftnav.hackpad.com/Alias-PLL-lock-detector-in-L2C-4fWUJWUNnOE */
  if (abs_err > 25. / 2.)
    correction = 50 * ((int)abs_err / 50) + 25;
  else
    correction = 0;

  return correction * err_sign;
}
