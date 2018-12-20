/*
 * Copyright (C) 2012, 2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <swiftnav/constants.h>
#include <swiftnav/coord_system.h>
#include <swiftnav/ephemeris.h>
#include <swiftnav/signal.h>

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "trk_loop_common.h"

/** \defgroup track Tracking
 * Functions used in tracking.
 * \{ */

/** \defgroup track_loop Tracking Loops
 * Functions used by the tracking loops.
 * \{ */

/** Phase discriminator for a Costas loop.
 *
 * References:
 *  -# Understanding GPS: Principles and Applications.
 *     Elliott D. Kaplan. Artech House, 1996.
 *
 * \param I The prompt in-phase correlation
 * \param Q The prompt quadrature correlation
 * \return The discriminator value [cycles]
 */
float costas_discriminator(float I, float Q) {
  if (I == 0) {
    // Technically, it should be +/- 0.25, but then we'd have to keep track
    //  of the previous sign do it right, so it's simple enough to just return
    //  the average of 0.25 and -0.25 in the face of that ambiguity, so zero.
    return 0;
  }
  return atanf(Q / I) * (float)(1 / (2 * M_PI));
}

/** Normalised non-coherent early-minus-late envelope DLL discriminator.
 *
 * References:
 *  -# Understanding GPS: Principles and Applications.
 *     Elliott D. Kaplan. Artech House, 1996.
 *
 * \param cs An array [E, P, L] of correlation_t structs for the Early, Prompt
 *           and Late correlations.
 * \return The discriminator value
 */
float dll_discriminator(const correlation_t cs[3]) {
  float early_mag = sqrtf(cs[0].I * cs[0].I + cs[0].Q * cs[0].Q);
  float late_mag = sqrtf(cs[2].I * cs[2].I + cs[2].Q * cs[2].Q);
  float early_late_mag_sum = early_mag + late_mag;

  if (0. == early_late_mag_sum) {
    return 0;
  }

  float ret = 0.5f * (early_mag - late_mag) / early_late_mag_sum;
  if (isnormal(ret)) {
    return ret;
  }

  return 0;
}

/** \} */
/** \} */
