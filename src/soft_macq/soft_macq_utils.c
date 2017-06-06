/*
 * lut.c
 *
 *  Created on: Feb 1, 2017
 *      Author: mic
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "soft_macq_utils.h"


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
  int16_t iSampleLUT[(1<<SAMPLE_BITS)] = {-1, -3, +1, +3};
  uint32_t uVal, uPhase;
  int16_t iSampleVal;
  float fPhase, fScale, fCos, fSin;

  fScale = 32.0;
  for (k=0; k<BBLUT_SIZE; k++) {
    uVal = k;
    iSampleVal = iSampleLUT[(uVal>>BBNCO_CARRPH_BITS) & SAMPLE_MASK];
    uPhase = uVal & BBNCO_CARRPH_MASK;
    fPhase = (float) uPhase * TWOPI / BBNCO_CARRPH_SIZE;
    fCos = +fScale * iSampleVal * cosf(fPhase);
    if ( fCos < 0 ) {
      bbConvTable[k].r = 2*floorf(fCos)+1;
    } else {
      bbConvTable[k].r = 2*ceilf(fCos) -1;
    }
    fSin = -fScale * iSampleVal * sinf(fPhase);
    if (fSin < 0) {
      bbConvTable[k].i = 2*floorf(fSin)+1;
    } else {
      bbConvTable[k].i = 2*ceilf(fSin) -1;
    }
  }

  return 0;
}


/*! \fn uint8_t HexToDec
 *  \brief Converts an hexadecimal character to its decimal representation
 * this is such a dumb function >:|
 */
uint8_t HexToDec(const char cHex) {
  if (cHex >= '0' && cHex <= '9') return (     cHex - '0');
  if (cHex >= 'A' && cHex <= 'F') return (10 + cHex - 'A');
  if (cHex >= 'a' && cHex <= 'f') return (10 + cHex - 'a');
  return 0;
}


/*! \fn uint32_t CirclesToUint32
 *  \brief Converts from the unit circle domain
 * to the unsigned integer domain, it is useful for a NCO where an unsigned registers
 * overflows to zero (like in a circle you come back to the beginning).
 */
uint32_t CirclesToUint32(double dCircles) {
  double dTmp;
  uint32_t uiRet;

  dTmp = fmod(POW_TWO_P32 * dCircles, POW_TWO_P32);
  if (dTmp < 0.0) {
    uiRet = (uint32_t) rint(POW_TWO_P32 + dTmp);
  } else {
    uiRet = (uint32_t) rint(dTmp);
  }
  return uiRet;
}


/*! \fn void Sc16ArrayMulX
 *  \brief Multiply-conjugate two arrays of complex int16_t values.
 * Please note that size should be aligned and multiple of 4,
 * otherwise the NEON version will not comply.
 * Please note that the loop could be unrolled further
 * if the array length can be inferred
 */
void Sc16ArrayMulX(sc16_t *_pr, sc16_t *_pa, sc16_t *_pb, int _iSize) {
  if (_iSize <= 0) return;
  if (NULL == _pr) return;
  if (NULL == _pa) return;
  if (NULL == _pb) return;

#if defined __ARM_NEON__

  __asm__ __volatile__
  ( "1000:                             \n\t"
    "SUBS       %[S], %[S], #4         \n\t" // size -= 4
    "VLD2.16    {d0, d1}, [%[A]]!      \n\t" // D0 holds 4 real values of A, D1 holds 4 imaginary values of A
    "VLD2.16    {d2, d3}, [%[B]]!      \n\t" // D2 holds 4 real values of B, D3 holds 4 imaginary values of B
    "VMULL.s16        q2, d2, d0       \n\t" // Q2  = aRE*bRE
    "VMLAL.s16        q2, d3, d1       \n\t" // Q2 += aIM*bIM
    "VMULL.s16        q3, d2, d1       \n\t" // Q3  = aIM*bRE
    "VMLSL.s16        q3, d3, d0       \n\t" // Q3 -= aRE*bIM
    "VQRSHRN.s32      d0, q2, #16      \n\t" //
    "VQRSHRN.s32      d1, q3, #16      \n\t" //
    "VST2.16    {d0, d1}, [%[R]]!      \n\t" // Store 4 CPX
    "BGT 1000b                         \n\t" // keep on if size>0
    :[R]"+r" (_pr), [A]"+r" (_pa), [B]"+r" (_pb), [S]"+r" (_iSize)
     ::"q0", "q1", "q2", "q3", "memory" );

#elif defined NOOPT__ARM_ARCH_7A__

  __asm__ __volatile__
  ("1000:                        \n\t"
      "SUBS       %[S], %[S], #1    \n\t" // size -= 1
      "LDR      r1, [%[A]]!         \n\t"
      "LDR      r2, [%[B]]!         \n\t"
      "SMUAD    r0, r1, r2          \n\t"
      "SMUSDX   r1, r1, r2          \n\t"
      "PKHBT    r0, r1, r0, LSL #16 \n\t"
      "STR      r0, [%[R]]!         \n\t"
      "BGT 1000b                    \n\t" // keep on if size>0
      :[R]"+r" (_pr), [A]"+r" (_pa), [B]"+r" (_pb), [S]"+r" (_iSize)
       :"r0", "r1", "r2", "memory");

#else

  int i;
  sc32_t pr32;

  for (i=0; i<_iSize; i++) {
    pr32.r = +(int32_t)(_pa[i].r)*(int32_t)(_pb[i].r) +(int32_t)(_pa[i].i)*(int32_t)(_pb[i].i);
    pr32.i = -(int32_t)(_pa[i].r)*(int32_t)(_pb[i].i) +(int32_t)(_pa[i].i)*(int32_t)(_pb[i].r);
    _pr[i].r = (int16_t) (pr32.r >> 16);
    _pr[i].i = (int16_t) (pr32.i >> 16);
  }

#endif
}


void Sc16ArrayAddAbsTo(float *_fOut, sc16_t *_fIn, int _iSize) {
  int i;
  if (NULL == _fOut) return;
  if (NULL == _fIn) return;
  if (_iSize <= 0) return;

  for (i=0; i<_iSize; i++) {
    _fOut[i] += ( ((float)_fIn[i].r * _fIn[i].r) + ((float)_fIn[i].i * _fIn[i].i) );
  }
}

/** Multiply-conjugate two arrays of complex float values.
 * Please note that size should be aligned
 */
void Fc32ArrayMulX(fc32_t *_pr, fc32_t *_pa, fc32_t *_pb, int _iSize) {
  if (_iSize <= 0) return;
  if (NULL == _pr) return;
  if (NULL == _pa) return;
  if (NULL == _pb) return;

#if defined NOOPT__ARM_NEON__

  __asm__ __volatile__
  ("2000:                             \n\t"
      "SUBS       %[S], %[S], #4         \n\t" // size -= 4
      "VLD2.32   {d16-d19}, [%[A]]!      \n\t" //  Load 4 RE "a" values in  q8 and 4 IM "a" values in  q9
      "VLD2.32   {d24-d27}, [%[B]]!      \n\t" //  Load 4 RE "b" values in q12 and 4 IM "b" values in q13
      "VMUL.f32       q0,  q8, q12       \n\t" // q0  = aR * bR    [0-3]
      "VMUL.f32       q1,  q9, q12       \n\t" // q1  = aI * bR
      "VMLA.f32       q0,  q9, q13       \n\t" // q0 += aI * bI    [0-3]
      "VMLS.f32       q1,  q8, q13       \n\t" // q1 -= aR * bI
      "VST2.32     {d0-d3}, [%[R]]!      \n\t" // Store q0,q1
      "BGT 2000b                         \n\t" // keep on if size>0
      :[R]"+r" (_pr), [A]"+r" (_pa), [B]"+r" (_pb), [S]"+r" (_iSize)
       ::"q0", "q1", "q2", "q3", "memory");

#else

  int i;

  for (i=0; i<_iSize; i++) {
    _pr[i].r = +(_pa[i].r)*(_pb[i].r) +(_pa[i].i)*(_pb[i].i);
    _pr[i].i = -(_pa[i].r)*(_pb[i].i) +(_pa[i].i)*(_pb[i].r);
  }

#endif /* NOOPT__ARM_NEON__ */

}


/*! \fn void Fc32ArrayAddAbsTo
 *  \brief
*/
void Fc32ArrayAddAbsTo(float *_fOut, fc32_t *_fIn, int _iSize) {
  int i;
  if (NULL == _fOut) return;
  if (NULL == _fIn) return;
  if (_iSize <= 0) return;

  for (i=0; i<_iSize; i++) {
    _fOut[i] += sqrtf( (_fIn[i].r * _fIn[i].r) + (_fIn[i].i * _fIn[i].i) );
  }
}


/*! \fn float FloatMax
 *  \brief compute max for float type
*/
float FloatMax(float *_pVec, int _iSize) {
  int i;
  if (NULL == _pVec) return -1.0;

  float fMax = _pVec[0];
  for (i=1; i<_iSize; i++) {
    if (fMax<_pVec[i]) {
      fMax = _pVec[i];
    }
  }
  return fMax;
}


/*! \fn int Usi2Satid
 *  \brief from Rinex name convention to Universal Satellite Identifier
*/
void Satid2Usi(int *_puUsi, char _sSatId[4]) {
  int uUsi = 0;
  _sSatId[3] = 0;
  switch(_sSatId[0]) {
  case 'G':
    uUsi = GPS_PRN_MIN+atoi(_sSatId+1);
    break;
  case 'R':
    uUsi = GLO_PRN_MIN+atoi(_sSatId+1);
    break;
  case 'E':
    uUsi = GAL_PRN_MIN+atoi(_sSatId+1);
    break;
  case 'I':
    uUsi = IRN_PRN_MIN+atoi(_sSatId+1);
    break;
  case 'J':
    uUsi = QZS_PRN_MIN+atoi(_sSatId+1);
    break;
  case 'S':
    uUsi = SBS_PRN_MIN+atoi(_sSatId+1);
    break;
  default:
    break;
  }
  (*_puUsi) = uUsi;
}

/*! \fn int Usi2Satid
 * from Universal Satellite Identifier to Rinex name convention
*/
void Usi2Satid(char _sSatId[4], int _uUsi) {
  if        ((_uUsi >= GPS_PRN_MIN) && (_uUsi < (GPS_PRN_MIN+NUM_GPS_SVS))) {
    sprintf(_sSatId, "G%02d", _uUsi - GPS_PRN_MIN + 1);
  } else if ((_uUsi >= GAL_PRN_MIN) && (_uUsi < (GAL_PRN_MIN+NUM_GAL_SVS))) {
    sprintf(_sSatId, "E%02d", _uUsi - GAL_PRN_MIN + 1);
  } else if ((_uUsi >= GLO_PRN_MIN) && (_uUsi < (GLO_PRN_MIN+NUM_GLO_SVS))) {
    sprintf(_sSatId, "R%02d", _uUsi - GLO_PRN_MIN + 1);
  } else if ((_uUsi >= BEI_PRN_MIN) && (_uUsi < (BEI_PRN_MIN+NUM_BEI_SVS))) {
    sprintf(_sSatId, "C%02d", _uUsi - BEI_PRN_MIN + 1);
  } else if ((_uUsi >= IRN_PRN_MIN) && (_uUsi < (IRN_PRN_MIN+NUM_IRN_SVS))) {
    sprintf(_sSatId, "I%02d", _uUsi - IRN_PRN_MIN + 1);
  } else if ((_uUsi >= QZS_PRN_MIN) && (_uUsi < (QZS_PRN_MIN+NUM_QZS_SVS))) {
    sprintf(_sSatId, "J%02d", _uUsi - QZS_PRN_MIN + 1);
  } else if ((_uUsi >= SBS_PRN_MIN) && (_uUsi < (SBS_PRN_MIN+NUM_SBS_SVS))) {
    sprintf(_sSatId, "S%02d", _uUsi - SBS_PRN_MIN + 1);
  }
}


/*! \fn int IsAcquired3D
 *  \brief takes in a matrix in array form and produces the maximum
 * value along with relevant code and frequency indexes
 */
int IsAcquired3D(float *vec, int _iCodeSh, int _iFreqSh, float *_fMval, int *_piCodeMaxI, int *_piFreqMaxI) {
  int k, i;
  int iCodeLen4th, iFreqMaxI;
  float m[4]  = {0.0};
  float max = 0;
  int   im[4] = {0};
  int   imax = 0, kmax = 0;

  iCodeLen4th = _iCodeSh / 4;

  for (i=0; i<_iCodeSh*_iFreqSh; i++) {
    if (vec[i] > max) { max = vec[i]; imax = i; }
  }
  iFreqMaxI = imax % _iFreqSh;

  (*_fMval) = 0.0; (*_piCodeMaxI) = 0; (*_piFreqMaxI) = 0;

  for (k=0; k < 1*iCodeLen4th; k++) {
    i = k*_iFreqSh + iFreqMaxI;
    if (vec[i] > m[ 0]) { m[ 0] = vec[i]; im[ 0] = i; }
  }
  for (   ; k < 2*iCodeLen4th; k++) {
    i = k*_iFreqSh + iFreqMaxI;
    if (vec[i] > m[ 1]) { m[ 1] = vec[i]; im[ 1] = i; }
  }
  for (   ; k < 3*iCodeLen4th; k++) {
    i = k*_iFreqSh + iFreqMaxI;
    if (vec[i] > m[ 2]) { m[ 2] = vec[i]; im[ 2] = i; }
  }
  for (   ; k <      _iCodeSh; k++) {
    i = k*_iFreqSh + iFreqMaxI;
    if (vec[i] > m[ 3]) { m[ 3] = vec[i]; im[ 3] = i; }
  }

  for (k=0; k<4; k++) {
    if (m[k] > max) { max = m[k]; imax = im[k]; kmax = k; }
  }
  k = (kmax + 2) % 4;
  (*_fMval)  = max;
  if (max > (3 * m[k])) {
    (*_piCodeMaxI) = imax / _iFreqSh;
    (*_piFreqMaxI) = iFreqMaxI;
    return 1;
  }
  return 0;
}


