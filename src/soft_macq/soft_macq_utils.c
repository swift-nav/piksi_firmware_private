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
int InitBBConvLut(void) {
  int k;
  s16 iSampleLUT[(1 << SAMPLE_BITS)] = {-1, -3, +1, +3};
  uint32_t uVal, uPhase;
  s16 iSampleVal;
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
uint32_t CirclesToUint32(double dCircles) {
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

/*! \fn void Sc16ArrayMulX
 *  \brief Multiply-conjugate two arrays of complex s16 values.
 * Please note that size should be aligned and multiple of 4,
 * otherwise the NEON version will not comply.
 * Please note that the loop could be unrolled further
 * if the array length can be inferred
 */
void Sc16ArrayMulX(sc16_t *_pr, sc16_t *_pa, sc16_t *_pb, u32 _iSize) {
  if (_iSize <= 0) return;
  if (NULL == _pr) return;
  if (NULL == _pa) return;
  if (NULL == _pb) return;

#if defined __ARM_NEON__

  __asm__ __volatile__(
      "1000:                             \n\t"
      "SUBS       %[S], %[S], #4         \n\t"  // size -= 4
      "VLD2.16    {d0, d1}, [%[A]]!      \n\t"  // D0 holds 4 real values of A,
                                                // D1 holds 4 imaginary values
                                                // of A
      "VLD2.16    {d2, d3}, [%[B]]!      \n\t"  // D2 holds 4 real values of B,
                                                // D3 holds 4 imaginary values
                                                // of B
      "VMULL.s16        q2, d2, d0       \n\t"  // Q2  = aRE*bRE
      "VMLAL.s16        q2, d3, d1       \n\t"  // Q2 += aIM*bIM
      "VMULL.s16        q3, d2, d1       \n\t"  // Q3  = aIM*bRE
      "VMLSL.s16        q3, d3, d0       \n\t"  // Q3 -= aRE*bIM
      "VQRSHRN.s32      d0, q2, #16      \n\t"  //
      "VQRSHRN.s32      d1, q3, #16      \n\t"  //
      "VST2.16    {d0, d1}, [%[R]]!      \n\t"  // Store 4 CPX
      "BGT 1000b                         \n\t"  // keep on if size>0
      : [R] "+r"(_pr),
        [A] "+r"(_pa),
        [B] "+r"(_pb),
        [S] "+r"(_iSize)::"q0",
        "q1",
        "q2",
        "q3",
        "memory");

#elif defined NOOPT__ARM_ARCH_7A__

  __asm__ __volatile__(
      "1000:                        \n\t"
      "SUBS       %[S], %[S], #1    \n\t"  // size -= 1
      "LDR      r1, [%[A]]!         \n\t"
      "LDR      r2, [%[B]]!         \n\t"
      "SMUAD    r0, r1, r2          \n\t"
      "SMUSDX   r1, r1, r2          \n\t"
      "PKHBT    r0, r1, r0, LSL #16 \n\t"
      "STR      r0, [%[R]]!         \n\t"
      "BGT 1000b                    \n\t"  // keep on if size>0
      : [R] "+r"(_pr), [A] "+r"(_pa), [B] "+r"(_pb), [S] "+r"(_iSize)
      : "r0", "r1", "r2", "memory");

#else

  sc32_t pr32;
  for (u32 i = 0; i < _iSize; i++) {
    pr32.r =
        +(s32)(_pa[i].r) * (s32)(_pb[i].r) + (s32)(_pa[i].i) * (s32)(_pb[i].i);
    pr32.i =
        -(s32)(_pa[i].r) * (s32)(_pb[i].i) + (s32)(_pa[i].i) * (s32)(_pb[i].r);
    _pr[i].r = (s16)(pr32.r >> 16);
    _pr[i].i = (s16)(pr32.i >> 16);
  }

#endif
}

/*! \fn void Sc16ArrayAddAbsTo
 *  \brief computes mag square of complex 16 bit value array and adds to a float
 * array
 *  */
void Sc16ArrayAddAbsTo(float *_fOut, sc16_t *_fIn, u32 _iSize) {
  if (NULL == _fOut) return;
  if (NULL == _fIn) return;
  if (_iSize <= 0) return;

  for (u32 i = 0; i < _iSize; i++) {
    _fOut[i] +=
        (((float)_fIn[i].r * _fIn[i].r) + ((float)_fIn[i].i * _fIn[i].i));
  }
}

/*! \fn int IsAcquired3D
 *  \brief takes in a matrix in array form and produces the maximum
 * value along with relevant code and frequency indexes
 */
int IsAcquired3D(const float *vec,
                 const u32 _iCodeSh,
                 const u32 _iFreqSh,
                 const u32 _iNonCoh,
                 float *_fMval,
                 float *_pfCodeMaxI,
                 float *_pfFreqMaxI) {
  u32 k, i;
  u32 code_len_4th, max_freq_index;
  float m[4] = {0.0};
  float max = 0.0f;
  u32 im[4] = {0};
  u32 imax = 0, kmax = 0;

  code_len_4th = _iCodeSh / 4;

  /* first absolute (2D) maximum */
  for (i = 0; i < _iCodeSh * _iFreqSh; i++) {
    if (vec[i] > max) {
      max = vec[i];
      imax = i;
    }
  }
  max_freq_index = imax % _iFreqSh;

  (*_fMval) = 0.0f;
  (*_pfCodeMaxI) = 0.0f;
  (*_pfFreqMaxI) = 0.0f;

  /* first slice of code-dimension correlation */
  for (k = 0; k < 1 * code_len_4th; k++) {
    i = k * _iFreqSh + max_freq_index;
    if (vec[i] > m[0]) {
      m[0] = vec[i];
      im[0] = i;
    }
  }
  /* second slice */
  for (; k < 2 * code_len_4th; k++) {
    i = k * _iFreqSh + max_freq_index;
    if (vec[i] > m[1]) {
      m[1] = vec[i];
      im[1] = i;
    }
  }
  /* third slice  */
  for (; k < 3 * code_len_4th; k++) {
    i = k * _iFreqSh + max_freq_index;
    if (vec[i] > m[2]) {
      m[2] = vec[i];
      im[2] = i;
    }
  }
  /* fourth slice */
  for (; k < _iCodeSh; k++) {
    i = k * _iFreqSh + max_freq_index;
    if (vec[i] > m[3]) {
      m[3] = vec[i];
      im[3] = i;
    }
  }

  /* find highest of four peaks */
  max = 0.0f;
  for (k = 0; k < 4; k++) {
    if (m[k] > max) {
      max = m[k];
      imax = im[k];
      kmax = k;
    }
  }
  (*_fMval) = max;

  k = (kmax + 2) % 4;

  /* compute mean far from peak */
  float mean_clean = 0.0f;
  for (u32 idx = (k * code_len_4th); idx < ((k + 1) * code_len_4th); idx++) {
    i = idx * _iFreqSh + max_freq_index;
    mean_clean += vec[i];
  }
  mean_clean /= (float)code_len_4th;

  /* is threshold higher? */
  if (max > (23.0f * mean_clean / (1+log2f(_iNonCoh)))) {
    /* code */
    (*_pfCodeMaxI) = (float)imax / _iFreqSh;

    /* quadratic fit on freq (it's a sinc actually) */
    u32 max_code_idx = _iFreqSh * (imax / _iFreqSh);
    float ea = (float)vec[max_code_idx + ((max_freq_index - 1) % _iFreqSh)];
    float la = (float)vec[max_code_idx + ((max_freq_index + 1) % _iFreqSh)];
    float freq_delta = 0.0f;
    if ((0.0f != ea) && (0.0f != la)) {
      freq_delta = 0.5f * (la - ea) / (la + ea);
    }
    if ((0 == max_freq_index) && (0.0f > freq_delta)) {
      (*_pfFreqMaxI) = (float)_iFreqSh + freq_delta;
    } else {
      (*_pfFreqMaxI) = (float)max_freq_index + freq_delta;
    }
    return 1;
  }
  return 0;
}
