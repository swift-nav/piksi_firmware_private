/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Michele Bavaro <michele@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "soft_macq_utils.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/** baseband LUT conversion table, takes uint8_t with a signal sample
 *  and a phase argument and generates the rotated complex sample
 * */
sc16_t bbConvTable[BBLUT_SIZE];

/*! \fn int InitBBConvLut
 *  \brief This function simply fills up the lookup table with complex values
 * resulting from the rotation of the real sample
 * (in sign-magnitude format) by a phase.
 * The NCO generates the phase and appends it to the sample,
 * then uses the lookup table to generate the result.
 * The scaling here is arbitrary. In reality only 1 bit more
 * than the input sample width is needed, but as we are going
 * to 16 bits to make the following decimation faster,
 * why not scaling up a bit in the first place?
 * */
int InitBBConvLut(void)
{
  int k;
  int16_t iSampleLUT[(1 << SAMPLE_BITS)] = {-1, -3, +1, +3};
  uint32_t uVal, uPhase;
  int16_t iSampleVal;
  float fPhase, fScale, fCos, fSin;

  fScale = 32.0;
  for (k = 0; k < BBLUT_SIZE; k++) {
    uVal = k;
    iSampleVal = iSampleLUT[(uVal >> BBNCO_CARRPH_BITS) & SAMPLE_MASK];
    uPhase = uVal & BBNCO_CARRPH_MASK;
    fPhase = (float)uPhase * TWOPI / BBNCO_CARRPH_SIZE;
    fCos = +fScale * iSampleVal * cosf(fPhase);
    if (fCos < 0) {
      bbConvTable[k].r = 2 * floorf(fCos) + 1;
    } else {
      bbConvTable[k].r = 2 * ceilf(fCos) - 1;
    }
    fSin = -fScale * iSampleVal * sinf(fPhase);
    if (fSin < 0) {
      bbConvTable[k].i = 2 * floorf(fSin) + 1;
    } else {
      bbConvTable[k].i = 2 * ceilf(fSin) - 1;
    }
  }

  return 0;
}

/*! \fn uint32_t CirclesToUint32
 *  \brief Converts from the unit circle domain
 * to the unsigned integer domain, it is useful for a NCO where an unsigned
 * registers
 * overflows to zero (like in a circle you come back to the beginning).
 */
uint32_t CirclesToUint32(double dCircles)
{
  double dTmp;
  uint32_t uiRet;

  dTmp = fmod(POW_TWO_P32 * dCircles, POW_TWO_P32);
  if (dTmp < 0.0) {
    uiRet = (uint32_t)rint(POW_TWO_P32 + dTmp);
  } else {
    uiRet = (uint32_t)rint(dTmp);
  }
  return uiRet;
}
