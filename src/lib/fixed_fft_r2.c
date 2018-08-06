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

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <libswiftnav/logging.h>

#include "fixed_fft_r2.h"

/*! \struct intFFTr2_t
 *  \brief generic size FFT when statically declared
 */
typedef struct _intFFTr2 {
  u32 M;       /* FFT order */
  u32 N;       /* FFT length */
  sc16_t *W;   /* Twiddle factors table */
  u32 *tmpBRX; /* Shuffle temp array */
  u16 *BR;     /* Bit-reversal */

  uint8_t *pMem; /* internal memory pointer */
} intFFTr2_t;

static void InitW(intFFTr2_t *pIntFFT);
static void InitBR(intFFTr2_t *pIntFFT);
static void DoShuffle(intFFTr2_t *pIntFFT, sc16_t *_x);

static void RankF2p(
    sc16_t *_A, sc16_t *_B, sc16_t *_W, u32 _nblocks, u32 _bsize);

static void RankF2pN(
    sc16_t *_A, sc16_t *_B, sc16_t *_W, u32 _nblocks, u32 _bsize);

static void RankB2p(
    sc16_t *_A, sc16_t *_B, sc16_t *_W, u32 _nblocks, u32 _bsize);

static void RankB2pN(
    sc16_t *_A, sc16_t *_B, sc16_t *_W, u32 _nblocks, u32 _bsize);

static void RankF1(sc16_t *_A, sc16_t *_B, u32 _nblocks);

static void RankF1N(sc16_t *_A, sc16_t *_B, u32 _nblocks);

static void RankB1(sc16_t *_A, sc16_t *_B, u32 _nblocks);

static void RankB1N(sc16_t *_A, sc16_t *_B, u32 _nblocks);

static void Rank0(sc16_t *_A, sc16_t *_B, u32 _nblocks);

static void Rank0N(sc16_t *_A, sc16_t *_B, u32 _nblocks);

/*! \fn int InitINTFFT
 *  \brief Init the memory for a FFT
 */
void InitIntFFTr2(void *_pFFT, u32 _N) {
  ASSERT((_N > 0) && (_N <= INTFFT_MAXSIZE) && !(_N & (_N - 1)));

  intFFTr2_t *pIntFFT = (intFFTr2_t *)_pFFT;

  log_debug("_pFFT %08" PRIuPTR " _N %" PRIu32, (uintptr_t)_pFFT, _N);

  uintptr_t pMemStart = (uintptr_t) & (pIntFFT->pMem);
  pIntFFT->W = (sc16_t *)(pMemStart + 0);
  pIntFFT->tmpBRX = (u32 *)(pMemStart + (_N * sizeof(sc16_t)));
  pIntFFT->BR = (u16 *)(pMemStart + (_N * (sizeof(sc16_t) + sizeof(u32))));

  pIntFFT->M = 0;
  pIntFFT->N = _N;
  while (_N > 1) {
    pIntFFT->M++;
    _N >>= 1;
  }

  InitW(pIntFFT);
  InitBR(pIntFFT);

  log_debug("M %" PRIu32 "  N %" PRIu32 "  W %08" PRIuPTR "  tmpBRX %08" PRIuPTR
            "  BR %08" PRIuPTR,
            pIntFFT->M,
            pIntFFT->N,
            (uintptr_t)pIntFFT->W,
            (uintptr_t)pIntFFT->tmpBRX,
            (uintptr_t)pIntFFT->BR);
}

/*! \fn void DoFwdIntFFTr2
 *  \brief Decimation in frequency forward FFT
 */
void DoFwdIntFFTr2(void *_pFFT, sc16_t *_x, u32 _uScale, int _iShuf) {
  u32 cnt, nBlocks, _bsize;
  sc16_t *a, *b, *w;

  intFFTr2_t *pIntFFT = (intFFTr2_t *)_pFFT;

  _bsize = (pIntFFT->N) >> 1;
  nBlocks = 1;

  for (cnt = 0; cnt < (pIntFFT->M); cnt++) {
    a = _x;
    b = _x + _bsize;
    w = pIntFFT->W + _bsize;

    /* */
    if (_bsize > 2) {
      if ((_uScale >> (2 * cnt)) & 0x3) {
        RankF2p(a, b, w, nBlocks, _bsize);
      } else {
        RankF2pN(a, b, w, nBlocks, _bsize);
      }
    } else if (_bsize > 1) {
      if ((_uScale >> (2 * cnt)) & 0x3) {
        RankF1(a, b, nBlocks);
      } else {
        RankF1N(a, b, nBlocks);
      }
    } else {
      if ((_uScale >> (2 * cnt)) & 0x3) {
        Rank0(a, b, nBlocks);
      } else {
        Rank0N(a, b, nBlocks);
      }
    }

    _bsize >>= 1;
    nBlocks <<= 1;
  }

  if (_iShuf) {
    DoShuffle(pIntFFT, _x);
  }
}

/*! \fn void DoBwdIntFFTr2
 *  \brief Decimation in time backward FFT
 */
void DoBwdIntFFTr2(void *_pFFT, sc16_t *_x, u32 _uScale, int _iShuf) {
  u32 cnt, nBlocks, _bsize;
  sc16_t *a, *b, *w;

  intFFTr2_t *pIntFFT = (intFFTr2_t *)_pFFT;

  if (_iShuf) {
    DoShuffle(pIntFFT, _x);
  }

  _bsize = 1;
  nBlocks = (pIntFFT->N) >> 1;

  for (cnt = 0; cnt < (pIntFFT->M); cnt++) {
    a = _x;
    b = _x + _bsize;
    w = pIntFFT->W + _bsize;

    /* */
    if (_bsize > 2) {
      if ((_uScale >> (2 * cnt)) & 0x3) {
        RankB2p(a, b, w, nBlocks, _bsize);
      } else {
        RankB2pN(a, b, w, nBlocks, _bsize);
      }
    } else if (_bsize > 1) {
      if ((_uScale >> (2 * cnt)) & 0x3) {
        RankB1(a, b, nBlocks);
      } else {
        RankB1N(a, b, nBlocks);
      }
    } else {
      if ((_uScale >> (2 * cnt)) & 0x3) {
        Rank0(a, b, nBlocks);
      } else {
        Rank0N(a, b, nBlocks);
      }
    }

    _bsize <<= 1;
    nBlocks >>= 1;
  }
}

/********************************************************
 *                  STATIC FUNCTIONS                    *
 ********************************************************/

/*! \fn void InitW
 *  \brief Twiddles initialisation
 */
static void InitW(intFFTr2_t *pIntFFT) {
  u32 k, nBlocks, _bsize;
  sc16_t *tW;
  float s, c, phase, fScale;

  fScale = 16 * 1024;
  _bsize = 1;
  nBlocks = (pIntFFT->N) >> 1;

  while (_bsize < (pIntFFT->N)) {
    tW = &(pIntFFT->W[_bsize]);

    for (k = 0; k < _bsize; k++) {
      phase = (-2.0 * M_PI * k * nBlocks) / (pIntFFT->N);
      c = floorf(fScale * cosf(phase));
      s = floorf(fScale * sinf(phase));

      tW[k].r = (s16)c;
      tW[k].i = (s16)s;
    }

    _bsize <<= 1;
    nBlocks >>= 1;
  }
}

/*! \fn void InitBR
 *  \brief Bit reversal initialisation
 */
static void InitBR(intFFTr2_t *pIntFFT) {
  u32 val, bit;
  u16 temp;

  for (val = 0; val < (pIntFFT->N); val++) {
    temp = 0;
    for (bit = 0; bit < (pIntFFT->M); bit++) {
      temp <<= 1;
      temp |= ((val >> bit) & 0x1);
    }
    pIntFFT->BR[val] = temp;
  }
}

/*! \fn void DoShuffle
 *  \brief Bit reversal initialisation
 */
static void DoShuffle(intFFTr2_t *pIntFFT, sc16_t *_x) {
  u32 cnt, *p;

  p = (u32 *)_x;
  memcpy(pIntFFT->tmpBRX, p, (pIntFFT->N) * sizeof(sc16_t));

  for (cnt = 0; cnt < (pIntFFT->N); cnt++) {
    p[cnt] = pIntFFT->tmpBRX[pIntFFT->BR[cnt]];
  }
}

#if defined __ARM_ARCH_7A__

static void Rank0(sc16_t *_A, sc16_t *_B, u32 _nblks) {
  (void)_B;
  __asm__ __volatile__(
      "MOV      r3, #0            \n\t"
      "1:                         \n\t"
      "SUBS     %[nB], %[nB], #1  \n\t"
      "LDRD     r0, r1, [%[A]]    \n\t"
      "SHADD16  r0, r0, r3        \n\t"
      "SHADD16  r2, r1, r3        \n\t"
      "QSUB16   r1, r0, r2        \n\t"
      "QADD16   r0, r0, r2        \n\t"
      "STRD     r0, r1, [%[A]]    \n\t"
      "ADD      %[A], %[A], #8    \n\t"
      "BNE 1b                     \n\t"
      :
      [A] "+r"(_A), [nB] "+r"(_nblks)::"r0", "r1", "r2", "r3", "cc", "memory");
}

static void Rank0N(sc16_t *_A, sc16_t *_B, u32 _nblks) {
  (void)_B;
  __asm__ __volatile__(
      "2:                         \n\t"
      "SUBS     %[nB], %[nB], #1  \n\t"
      "LDRD     r0, r1, [%[A]]    \n\t"
      "MOV      r2, r1            \n\t"
      "QSUB16   r1, r0, r2        \n\t"
      "QADD16   r0, r0, r2        \n\t"
      "STRD     r0, r1, [%[A]]    \n\t"
      "ADD      %[A], %[A], #8    \n\t"
      "BNE 2b                     \n\t"
      :
      [A] "+r"(_A), [nB] "+r"(_nblks)::"r0", "r1", "r2", "r3", "cc", "memory");
}

#else

static void Rank0(sc16_t *_A, sc16_t *_B, u32 _nblocks) {
  u32 blkIdx;
  s16 bi, bq;

  for (blkIdx = 0; blkIdx < _nblocks; blkIdx++) {
    (*_A).r >>= 1;
    (*_A).i >>= 1;
    (*_B).r >>= 1;
    (*_B).i >>= 1;

    bi = (*_B).r;
    bq = (*_B).i;
    (*_B).r = (*_A).r - bi;
    (*_B).i = (*_A).i - bq;
    (*_A).r = (*_A).r + bi;
    (*_A).i = (*_A).i + bq;

    _A += 2;
    _B += 2;
  }
}

static void Rank0N(sc16_t *_A, sc16_t *_B, u32 _nblocks) {
  u32 blkIdx;
  s16 bi, bq;

  for (blkIdx = 0; blkIdx < _nblocks; blkIdx++) {
    bi = (*_B).r;
    bq = (*_B).i;
    (*_B).r = (*_A).r - bi;
    (*_B).i = (*_A).i - bq;
    (*_A).r = (*_A).r + bi;
    (*_A).i = (*_A).i + bq;

    _A += 2;
    _B += 2;
  }
}

#endif /*  __ARCH_7A__ */

#if defined __ARM_ARCH_7A__

static void RankF1N(sc16_t *_A, sc16_t *_B, u32 _nblks) {
  __asm__ __volatile__(
      "MOV      r3, #0            \n\t"
      "3:                         \n\t"
      "LDR      r0, [%[A]]        \n\t"
      "LDR      r2, [%[B]]        \n\t"
      "QSUB16   r1, r0, r2        \n\t"
      "QADD16   r0, r0, r2        \n\t"
      "STR      r0, [%[A]]        \n\t"
      "STR      r1, [%[B]]        \n\t"
      "ADD      %[A], %[A], #4    \n\t"
      "ADD      %[B], %[B], #4    \n\t"
      "LDR      r0, [%[A]]        \n\t"
      "LDR      r2, [%[B]]        \n\t"
      "QSUB16   r1, r0, r2        \n\t"
      "QSUBADDX r1, r3, r1        \n\t"
      "QADD16   r0, r0, r2        \n\t"
      "STR      r0, [%[A]]        \n\t"
      "STR      r1, [%[B]]        \n\t"
      "ADD      %[A], %[A], #12   \n\t"
      "ADD      %[B], %[B], #12   \n\t"
      "SUBS     %[nB], %[nB], #1  \n\t"
      "BNE 3b                     \n\t"
      : [A] "+r"(_A),
        [B] "+r"(_B),
        [nB] "+r"(_nblks)::"r0",
        "r1",
        "r2",
        "r3",
        "cc",
        "memory");
}

static void RankF1(sc16_t *_A, sc16_t *_B, u32 _nblks) {
  __asm__ __volatile__(
      "MOV      r3, #0            \n\t"
      "4:                         \n\t"
      "LDR      r0, [%[A]]        \n\t"
      "LDR      r2, [%[B]]        \n\t"
      "SHADD16  r0, r0, r3        \n\t"
      "SHADD16  r2, r2, r3        \n\t"
      "QSUB16   r1, r0, r2        \n\t"
      "QADD16   r0, r0, r2        \n\t"
      "STR      r0, [%[A]]        \n\t"
      "STR      r1, [%[B]]        \n\t"
      "ADD      %[A], %[A], #4    \n\t"
      "ADD      %[B], %[B], #4    \n\t"
      "LDR      r0, [%[A]]        \n\t"
      "LDR      r2, [%[B]]        \n\t"
      "SHADD16  r0, r0, r3        \n\t"
      "SHADD16  r2, r2, r3        \n\t"
      "QSUB16   r1, r0, r2        \n\t"
      "QSUBADDX r1, r3, r1        \n\t"
      "QADD16   r0, r0, r2        \n\t"
      "STR      r0, [%[A]]        \n\t"
      "STR      r1, [%[B]]        \n\t"
      "ADD      %[A], %[A], #12   \n\t"
      "ADD      %[B], %[B], #12   \n\t"
      "SUBS     %[nB], %[nB], #1  \n\t"
      "BNE 4b                     \n\t"
      : [A] "+r"(_A),
        [B] "+r"(_B),
        [nB] "+r"(_nblks)::"r0",
        "r1",
        "r2",
        "r3",
        "cc",
        "memory");
}

#else

static void RankF1N(sc16_t *_A, sc16_t *_B, u32 _nblocks) {
  u32 blkIdx;
  s32 bi, bq;

  for (blkIdx = 0; blkIdx < _nblocks; blkIdx++) {
    bi = (*_B).r;
    bq = (*_B).i;

    (*_B).r = +(*_A).r - bi;
    (*_B).i = +(*_A).i - bq;
    (*_A).r = +(*_A).r + bi;
    (*_A).i = +(*_A).i + bq;

    _A++;
    _B++;

    bi = (*_B).r;
    bq = (*_B).i;

    (*_B).r = +(*_A).i - bq;
    (*_B).i = -(*_A).r + bi;
    (*_A).r = +(*_A).r + bi;
    (*_A).i = +(*_A).i + bq;

    _A += 3;
    _B += 3;
  }
}

static void RankF1(sc16_t *_A, sc16_t *_B, u32 _nblocks) {
  u32 blkIdx;
  s32 bi, bq;

  for (blkIdx = 0; blkIdx < _nblocks; blkIdx++) {
    (*_A).r >>= 1;
    (*_A).i >>= 1;
    (*_B).r >>= 1;
    (*_B).i >>= 1;

    bi = (*_B).r;
    bq = (*_B).i;

    (*_B).r = +(*_A).r - bi;
    (*_B).i = +(*_A).i - bq;
    (*_A).r = +(*_A).r + bi;
    (*_A).i = +(*_A).i + bq;

    _A++;
    _B++;

    (*_A).r >>= 1;
    (*_A).i >>= 1;
    (*_B).r >>= 1;
    (*_B).i >>= 1;

    bi = (*_B).r;
    bq = (*_B).i;

    (*_B).r = +(*_A).i - bq;
    (*_B).i = -(*_A).r + bi;
    (*_A).r = +(*_A).r + bi;
    (*_A).i = +(*_A).i + bq;

    _A += 3;
    _B += 3;
  }
}

#endif /*  __ARCH_7A__ */

#if defined __ARM_ARCH_7A__

static void RankB1N(sc16_t *_A, sc16_t *_B, u32 _nblks) {
  __asm__ __volatile__(
      "MOV      r3, #0            \n\t"
      "5:                         \n\t"
      "LDR      r0, [%[A]]        \n\t"
      "LDR      r2, [%[B]]        \n\t"
      "QSUB16   r1, r0, r2        \n\t"
      "QADD16   r0, r0, r2        \n\t"
      "STR      r0, [%[A]]        \n\t"
      "STR      r1, [%[B]]        \n\t"
      "ADD      %[A], %[A], #4    \n\t"
      "ADD      %[B], %[B], #4    \n\t"
      "LDR      r0, [%[A]]        \n\t"
      "LDR      r2, [%[B]]        \n\t"
      "QADDSUBX r2, r3, r2        \n\t"
      "QSUB16   r1, r0, r2        \n\t"
      "QADD16   r0, r0, r2        \n\t"
      "STR      r0, [%[A]]        \n\t"
      "STR      r1, [%[B]]        \n\t"
      "ADD      %[A], %[A], #12   \n\t"
      "ADD      %[B], %[B], #12   \n\t"
      "SUBS     %[nB], %[nB], #1  \n\t"
      "BNE 5b                     \n\t"
      : [A] "+r"(_A),
        [B] "+r"(_B),
        [nB] "+r"(_nblks)::"r0",
        "r1",
        "r2",
        "r3",
        "cc",
        "memory");
}

static void RankB1(sc16_t *_A, sc16_t *_B, u32 _nblks) {
  __asm__ __volatile__(
      "MOV      r3, #0              \n\t"
      "6:                           \n\t"
      "LDR      r0, [%[A]]          \n\t"
      "LDR      r2, [%[B]]          \n\t"
      "SHADD16  r0, r0, r3          \n\t"
      "SHADD16  r2, r2, r3          \n\t"
      "QSUB16   r1, r0, r2          \n\t"
      "QADD16   r0, r0, r2          \n\t"
      "STR      r0, [%[A]]          \n\t"
      "STR      r1, [%[B]]          \n\t"
      "ADD      %[A], %[A], #4      \n\t"
      "ADD      %[B], %[B], #4      \n\t"
      "LDR      r0, [%[A]]          \n\t"
      "LDR      r2, [%[B]]          \n\t"
      "SHADD16  r0, r0, r3          \n\t"
      "SHADD16  r2, r2, r3          \n\t"
      "QADDSUBX r2, r3, r2          \n\t"
      "QSUB16   r1, r0, r2          \n\t"
      "QADD16   r0, r0, r2          \n\t"
      "STR      r0, [%[A]]          \n\t"
      "STR      r1, [%[B]]          \n\t"
      "ADD      %[A], %[A], #12     \n\t"
      "ADD      %[B], %[B], #12     \n\t"
      "SUBS     %[nB], %[nB], #1    \n\t"
      "BNE 6b                       \n\t"
      : [A] "+r"(_A),
        [B] "+r"(_B),
        [nB] "+r"(_nblks)::"r0",
        "r1",
        "r2",
        "r3",
        "cc",
        "memory");
}

#else

static void RankB1N(sc16_t *_A, sc16_t *_B, u32 _nblocks) {
  u32 blkIdx;
  s16 bi, bq;

  for (blkIdx = 0; blkIdx < _nblocks; blkIdx++) {
    bi = +(*_B).r;
    bq = +(*_B).i;
    (*_B).r = (*_A).r - bi;
    (*_B).i = (*_A).i - bq;
    (*_A).r = (*_A).r + bi;
    (*_A).i = (*_A).i + bq;
    _A++;
    _B++;

    bi = -(*_B).i;
    bq = +(*_B).r;
    (*_B).r = (*_A).r - bi;
    (*_B).i = (*_A).i - bq;
    (*_A).r = (*_A).r + bi;
    (*_A).i = (*_A).i + bq;
    _A += 3;
    _B += 3;
  }
}

static void RankB1(sc16_t *_A, sc16_t *_B, u32 _nblocks) {
  u32 blkIdx;
  s16 bi, bq;

  for (blkIdx = 0; blkIdx < _nblocks; blkIdx++) {
    (*_A).r >>= 1;
    (*_A).i >>= 1;
    (*_B).r >>= 1;
    (*_B).i >>= 1;
    bi = +(*_B).r;
    bq = +(*_B).i;
    (*_B).r = (*_A).r - bi;
    (*_B).i = (*_A).i - bq;
    (*_A).r = (*_A).r + bi;
    (*_A).i = (*_A).i + bq;
    _A++;
    _B++;

    (*_A).r >>= 1;
    (*_A).i >>= 1;
    (*_B).r >>= 1;
    (*_B).i >>= 1;
    bi = -(*_B).i;
    bq = +(*_B).r;
    (*_B).r = (*_A).r - bi;
    (*_B).i = (*_A).i - bq;
    (*_A).r = (*_A).r + bi;
    (*_A).i = (*_A).i + bq;
    _A += 3;
    _B += 3;
  }
}

#endif /*  __ARCH_7A__ */

#if defined __ARM_NEON__

static void RankF2pN(
    sc16_t *_A, sc16_t *_B, sc16_t *_W, u32 _nblks, u32 _bsize) {
  __asm__ __volatile__(
      "7:                                   \n\t"
      "MOV        r2, %[W]                  \n\t"
      "MOV        r4, %[bS]                 \n\t"
      "8:                                   \n\t"
      "VLD1.32   {d0,  d1}, [%[A]]          \n\t"  // Load 4 A
      "VLD1.32   {d4,  d5}, [%[B]]          \n\t"  // Load 4 B'
      "VSUB.s16   q1,   q0,   q2            \n\t"  // B = A - B'
      "VADD.s16   q0,   q0,   q2            \n\t"  // A = A + B'
      "VLD1.32   {d4,  d5}, [r2]!           \n\t"  // Load 4 W
      "VTRN.16    d2,   d3                  \n\t"  // Collect RE and IM on B
      "VTRN.16    d4,   d5                  \n\t"  // Collect RE and IM on W
      "VMULL.s16  q3,   d2,   d4            \n\t"  // bi = Br*Wr
      "VMULL.s16  q5,   d3,   d4            \n\t"  // bq = Bi*Wr
      "VMLSL.s16  q3,   d3,   d5            \n\t"  // bi = bi - Bi*Wi
      "VMLAL.s16  q5,   d2,   d5            \n\t"  // bq = bq + Br*Wq
      "VRSHRN.i32 d2,   q3,    #14          \n\t"  // (bi+8192) >> 14
      "VRSHRN.i32 d3,   q5,    #14          \n\t"  // (bq+8192) >> 14
      "VTRN.16    d2,   d3                  \n\t"  // Unpack RE and IM on b
      "SUBS       r4,   r4,     #4          \n\t"  // sizeIdx -= 4
      "VST1.32   {d0,  d1}, [%[A]]!         \n\t"  // Write A
      "VST1.32   {d2,  d3}, [%[B]]!         \n\t"  // Write B
      "BGT 8b                               \n\t"  //
      "SUBS    %[nB], %[nB],  #1            \n\t"  // _nblks--
      "ADD      %[A],  %[A], %[bS], LSL #2  \n\t"  // A += _bsize
      "ADD      %[B],  %[B], %[bS], LSL #2  \n\t"  // B += _bsize
      "BGT 7b                               \n\t"  //
      : [A] "+&r"(_A), [B] "+&r"(_B), [nB] "+&r"(_nblks)
      : [W] "r"(_W), [bS] "r"(_bsize)
      : "r2", "r4", "q0", "q1", "q2", "q3", "q5", "cc", "memory");
}

#elif defined __ARM_ARCH_7A__

static void RankF2pN(
    sc16_t *_A, sc16_t *_B, sc16_t *_W, u32 _nblks, u32 _bsize) {
  __asm__ __volatile__(
      "7:                                \n\t"
      "MOV       r12,  %[W]              \n\t"
      "MOV        r4,  %[bS]             \n\t"
      "8:                                \n\t"
      "LDR        r0,  [%[A]]            \n\t"  // Load A
      "LDR        r1,  [%[B]]            \n\t"  // Load B
      "QSUB16     r2,  r0, r1            \n\t"  // B = A - B
      "QADD16     r0,  r0, r1            \n\t"  // A = A + B
      "LDR        r5,  [r12]             \n\t"  // Load W
      "SMUADX     r6,  r2, r5            \n\t"  //  imaginary
      "SMUSD      r5,  r2, r5            \n\t"  //  real
      "ADD        r6,  r6, #8192         \n\t"  //
      "ADD        r5,  r5, #8192         \n\t"  //
      "ASR        r5,  r5, #14           \n\t"  //
      "PKHBT      r1,  r5, r6, LSL #2    \n\t"
      "STR        r0,  [%[A]]            \n\t"       // Store A
      "STR        r1,  [%[B]]            \n\t"       // Store B
      "ADD      %[A], %[A],  #4          \n\t"       // A ++
      "ADD      %[B], %[B],  #4          \n\t"       // B ++
      "ADD       r12,  r12,  #4          \n\t"       // W ++
      "SUBS       r4,   r4,  #1          \n\t"       // sizeIdx --
      "BNE 8b                            \n\t"       //
      "ADD      %[A],  %[A], %[bS], LSL #2    \n\t"  // A += _bsize
      "ADD      %[B],  %[B], %[bS], LSL #2    \n\t"  // B += _bsize
      "SUBS     %[nB],  %[nB], #1            \n\t"   // _nblks--
      "BNE 7b                            \n\t"       //
      : [A] "+&r"(_A), [B] "+&r"(_B), [nB] "+&r"(_nblks)
      : [W] "r"(_W), [bS] "r"(_bsize)
      : "r0", "r1", "r2", "r4", "r5", "r6", "r12", "cc", "memory");
}

#else

static void ButterflyFwdDfNoScale(sc16_t *_A, sc16_t *_B, sc16_t *_W) {
  s32 bi, bq;

  bi = (*_B).r;
  bq = (*_B).i;

  (*_B).r = (*_A).r - bi;
  (*_B).i = (*_A).i - bq;
  (*_A).r = (*_A).r + bi;
  (*_A).i = (*_A).i + bq;

  bi = (*_B).r * (*_W).r - (*_B).i * (*_W).i;
  bq = (*_B).r * (*_W).i + (*_B).i * (*_W).r;

  bi = (bi + 8192) >> 14;
  bq = (bq + 8192) >> 14;

  (*_B).r = (s16)bi;
  (*_B).i = (s16)bq;
}

static void RankF2pN(
    sc16_t *_A, sc16_t *_B, sc16_t *_W, u32 _nblks, u32 _bsize) {
  u32 blkIdx, sizeIdx;
  sc16_t *wBase = _W;

  for (blkIdx = 0; blkIdx < _nblks; blkIdx++) {
    _W = wBase;
    for (sizeIdx = 0; sizeIdx < _bsize; sizeIdx++) {
      ButterflyFwdDfNoScale(_A, _B, _W);
      _A++;
      _B++;
      _W++;
    }

    _A += _bsize;
    _B += _bsize;
  }
}

#endif /* __ARM_NEON__ or __ARM__ARCH_7A__ */

#if defined __ARM_NEON__

static void RankF2p(
    sc16_t *_A, sc16_t *_B, sc16_t *_W, u32 _nblks, u32 _bsize) {
  __asm__ __volatile__(
      "9:                                   \n\t"
      "MOV        r2, %[W]                  \n\t"
      "MOV        r4, %[bS]                 \n\t"
      "10:                                   \n\t"
      "VLD1.32   {d0,  d1}, [%[A]]          \n\t"   // Load 4 A
      "VLD1.32   {d4,  d5}, [%[B]]          \n\t"   // Load 4 B'
      "VSHR.s16   q0,   q0, #1              \n\t"   // Scale A
      "VSHR.s16   q2,   q2, #1              \n\t"   // Scale B
      "VSUB.s16   q1,   q0, q2              \n\t"   // B = A - B'
      "VADD.s16   q0,   q0, q2              \n\t"   // A = A + B'
      "VLD1.32   {d4,  d5}, [r2]!            \n\t"  // Load 4 W
      "VTRN.16    d2,   d3                  \n\t"   // Collect RE and IM on B
      "VTRN.16    d4,   d5                  \n\t"   // Collect RE and IM on W
      "VMULL.s16  q3,   d2,   d4            \n\t"   // bi = Br*Wr
      "VMULL.s16  q5,   d3,   d4            \n\t"   // bq = Bi*Wr
      "VMLSL.s16  q3,   d3,   d5            \n\t"   // bi = bi - Bi*Wi
      "VMLAL.s16  q5,   d2,   d5            \n\t"   // bq = bq + Br*Wq
      "VRSHRN.i32 d2,   q3,    #14          \n\t"   // (bi+8192) >> 14
      "VRSHRN.i32 d3,   q5,    #14          \n\t"   // (bq+8192) >> 14
      "VTRN.16    d2,   d3                  \n\t"   // Unpack RE and IM on b
      "VST1.32   {d0,  d1}, [%[A]]!          \n\t"  // Write A
      "VST1.32   {d2,  d3}, [%[B]]!          \n\t"  // Write B
      "SUBS       r4,   r4,     #4          \n\t"   // sizeIdx -= 4
      "BNE 10b                               \n\t"  //
      "ADD      %[A],  %[A], %[bS], LSL #2  \n\t"   // A += _bsize
      "ADD      %[B],  %[B], %[bS], LSL #2  \n\t"   // B += _bsize
      "SUBS    %[nB], %[nB],  #1            \n\t"   // _nblks--
      "BNE 9b                               \n\t"   //
      : [A] "+&r"(_A), [B] "+&r"(_B), [nB] "+&r"(_nblks)
      : [W] "r"(_W), [bS] "r"(_bsize)
      : "r2", "r4", "q0", "q1", "q2", "q3", "q5", "cc", "memory");
}

#elif defined __ARM_ARCH_7A__

static void RankF2p(
    sc16_t *_A, sc16_t *_B, sc16_t *_W, u32 _nblks, u32 _bsize) {
  __asm__ __volatile__(
      "MOV        r3,  #0                      \n\t"
      "9:                                      \n\t"
      "MOV       r12,  %[W]                    \n\t"
      "MOV        r4,  %[bS]                   \n\t"
      "10:                                     \n\t"
      "LDR        r0, [%[A]]                   \n\t"  // Load A
      "LDR        r1, [%[B]]                   \n\t"  // Load B
      "SHADD16    r0,  r0, r3                  \n\t"  // A >>= 1
      "SHADD16    r1,  r1, r3                  \n\t"  // B >>= 1
      "QSUB16     r2,  r0, r1                  \n\t"  // B = A - B
      "QADD16     r0,  r0, r1                  \n\t"  // A = A + B
      "LDR        r5, [r12]                    \n\t"  // Load W
      "SMUADX     r6,  r2, r5                  \n\t"  //  imaginary
      "SMUSD      r5,  r2, r5                  \n\t"  //  real
      "ADD        r6,  r6, #8192               \n\t"  //
      "ADD        r5,  r5, #8192               \n\t"  //
      "ASR        r5,  r5, #14                 \n\t"  //
      "PKHBT      r1,  r5, r6, LSL #2          \n\t"
      "STR        r0, [%[A]]                   \n\t"  // Store A
      "STR        r1, [%[B]]                   \n\t"  // Store B
      "ADD      %[A], %[A], #4                 \n\t"  // A ++
      "ADD      %[B], %[B], #4                 \n\t"  // B ++
      "ADD       r12,  r12, #4                 \n\t"  // W ++
      "SUBS       r4,   r4, #1                 \n\t"  // sizeIdx --
      "BNE 10b                                 \n\t"  //
      "ADD        %[A],  %[A], %[bS], LSL #2   \n\t"  // A += _bsize
      "ADD        %[B],  %[B], %[bS], LSL #2   \n\t"  // B += _bsize
      "SUBS       %[nB],  %[nB],  #1           \n\t"  // _nblks--
      "BNE 9b                                  \n\t"  //
      : [A] "+&r"(_A), [B] "+&r"(_B), [nB] "+&r"(_nblks)
      : [W] "r"(_W), [bS] "r"(_bsize)
      : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r12", "cc", "memory");
}

#else

static void ButterflyFwdDf(sc16_t *_A, sc16_t *_B, sc16_t *_W) {
  s32 bi, bq;

  (*_A).r >>= 1;
  (*_A).i >>= 1;
  (*_B).r >>= 1;
  (*_B).i >>= 1;

  bi = (*_B).r;
  bq = (*_B).i;

  (*_B).r = (*_A).r - bi;
  (*_B).i = (*_A).i - bq;
  (*_A).r = (*_A).r + bi;
  (*_A).i = (*_A).i + bq;

  bi = (*_B).r * (*_W).r - (*_B).i * (*_W).i;
  bq = (*_B).r * (*_W).i + (*_B).i * (*_W).r;

  bi = (bi + 8192) >> 14;
  bq = (bq + 8192) >> 14;

  (*_B).r = (s16)bi;
  (*_B).i = (s16)bq;
}

static void RankF2p(
    sc16_t *_A, sc16_t *_B, sc16_t *_W, u32 _nblks, u32 _bsize) {
  u32 blkIdx, sizeIdx;
  sc16_t *wtmp;

  for (blkIdx = 0; blkIdx < _nblks; blkIdx++) {
    wtmp = _W;
    for (sizeIdx = 0; sizeIdx < _bsize; sizeIdx++) {
      ButterflyFwdDf(_A, _B, wtmp);
      _A++;
      _B++;
      wtmp++;
    }

    _A += _bsize;
    _B += _bsize;
  }
}

#endif /* __ARM_NEON__ or __ARM_ARCH_7A__ */

#if defined __ARM_NEON__

static void RankB2pN(
    sc16_t *_A, sc16_t *_B, sc16_t *_W, u32 _nblks, u32 _bsize) {
  u32 sizeIdx;
  sc16_t *wbase = _W;
  do {
    sizeIdx = _bsize;
    _W = wbase;

    __asm__ __volatile__(
        "7:                               \n\t"
        "SUBS       %[S], %[S], #4        \n\t"  // size -= 4
        "VLD2.16   {d0, d1}, [%[A]]       \n\t"  // d0 has real(A), d1 has
                                                 // imag(A)
        "VLD2.16   {d2, d3}, [%[B]]       \n\t"  // d2 has real(B), d3 has
                                                 // imag(B)
        "VLD2.16   {d4, d5}, [%[W]]!      \n\t"  // d4 has real(W), d5 has
                                                 // imag(W)
        "VMULL.s16  q3, d2, d4            \n\t"  // bi = Br*Wr
        "VMLAL.s16  q3, d3, d5            \n\t"  // bi = bi + Bi*Wi
        "VMULL.s16  q4, d3, d4            \n\t"  // bq = Bi*Wr
        "VMLSL.s16  q4, d2, d5            \n\t"  // bq = bq - Br*Wi
        "VRSHRN.i32 d4, q3, #14           \n\t"  // (bi+8192) >> 14  -- in q2
        "VRSHRN.i32 d5, q4, #14           \n\t"  // (bq+8192) >> 14
        "VSUB.s16   q1, q0, q2            \n\t"  // B = A - B'
        "VADD.s16   q0, q0, q2            \n\t"  // A = A + B'
        "VST2.16   {d0, d1}, [%[A]]!      \n\t"  // Write A
        "VST2.16   {d2, d3}, [%[B]]!      \n\t"  // Write B
        "BGT 7b                           \n\t"  // keep on if size>0
        : [A] "+r"(_A),
          [B] "+r"(_B),
          [W] "+r"(_W),
          [S] "+r"(sizeIdx)::"q0",
          "q1",
          "q2",
          "q3",
          "q4",
          "memory");

    _nblks--;
    _A += _bsize;
    _B += _bsize;
  } while (_nblks);
}

#elif defined __ARM_ARCH_7A__

static void RankB2pN(
    sc16_t *_A, sc16_t *_B, sc16_t *_W, u32 _nblks, u32 _bsize) {
  __asm__ __volatile__(
      "7:                                       \n\t"
      "MOV       r12,  %[W]                     \n\t"
      "MOV        r4,  %[bS]                    \n\t"
      "8:                                       \n\t"
      "LDR        r0,  [%[A]]                   \n\t"  // Load A
      "LDR        r2,  [%[B]]                   \n\t"  // Load B
      "LDR        r1, [r12]                     \n\t"  // Load W
      "SMUSDX     r5,   r1,   r2                \n\t"  //  imaginary
      "SMUAD      r2,   r1,   r2                \n\t"  //  real
      "ADD        r5,   r5, #8192               \n\t"  //
      "ADD        r2,   r2, #8192               \n\t"  //
      "ASR        r2,   r2, #14                 \n\t"  //
      "PKHBT      r1,   r2,   r5, LSL #2        \n\t"
      "QSUB16     r2,   r0,   r1                \n\t"  // B = A - B
      "QADD16     r0,   r0,   r1                \n\t"  // A = A + B
      "STR        r0,  [%[A]]                   \n\t"  // Store A
      "STR        r2,  [%[B]]                   \n\t"  // Store B
      "ADD      %[A], %[A], #4                  \n\t"  // A ++
      "ADD      %[B], %[B], #4                  \n\t"  // B ++
      "ADD       r12,  r12, #4                  \n\t"  // W ++
      "SUBS       r4,   r4, #1                  \n\t"  // sizeIdx --
      "BNE 8b                                   \n\t"  //
      "ADD        %[A],  %[A], %[bS], LSL #2    \n\t"  // A += _bsize
      "ADD        %[B],  %[B], %[bS], LSL #2    \n\t"  // B += _bsize
      "SUBS       %[nB],  %[nB], #1             \n\t"  // _nblks--
      "BNE 7b                                   \n\t"  //
      : [A] "+&r"(_A), [B] "+&r"(_B), [nB] "+&r"(_nblks)
      : [W] "r"(_W), [bS] "r"(_bsize)
      : "r0", "r1", "r2", "r4", "r5", "r12", "cc", "memory");
}

#else

static void ButterflyBwdDtNoScale(sc16_t *_A, sc16_t *_B, sc16_t *_W) {
  s32 bi, bq;

  bi = +(*_B).r * (*_W).r + (*_B).i * (*_W).i;
  bq = -(*_B).r * (*_W).i + (*_B).i * (*_W).r;

  bi = (bi + 8192) >> 14;
  bq = (bq + 8192) >> 14;

  (*_B).r = (*_A).r - (s16)bi;
  (*_B).i = (*_A).i - (s16)bq;
  (*_A).r = (*_A).r + (s16)bi;
  (*_A).i = (*_A).i + (s16)bq;
}

static void RankB2pN(
    sc16_t *_A, sc16_t *_B, sc16_t *_W, u32 _nblks, u32 _bsize) {
  u32 blkIdx, sizeIdx;
  sc16_t *wbase = _W;

  for (blkIdx = 0; blkIdx < _nblks; blkIdx++) {
    for (sizeIdx = 0; sizeIdx < _bsize; sizeIdx++) {
      ButterflyBwdDtNoScale(_A, _B, _W);
      _A++;
      _B++;
      _W++;
    }

    _A += _bsize;
    _B += _bsize;
    _W = wbase;
  }
}

#endif /* __ARM_NEON__ or __ARM_ARCH_7A__ */

#if defined __ARM_NEON__

static void RankB2p(
    sc16_t *_A, sc16_t *_B, sc16_t *_W, u32 _nblks, u32 _bsize) {
  u32 sizeIdx;
  sc16_t *wbase = _W;

  do {
    sizeIdx = _bsize;
    _W = wbase;

    do {
      __asm__ __volatile__(
          "VLD1.32   {d0, d1}, [%[A]]       \n\t"  // Load 4 A
          "VLD1.32   {d2, d3}, [%[B]]       \n\t"  // Load 4 B'
          "VSHR.s16   q0, q0, #1            \n\t"  // Scale A
          "VSHR.s16   q1, q1, #1            \n\t"  // Scale B
          "VLD1.32   {d4, d5}, [%[W]]       \n\t"  // Load 4 W
          "VTRN.16    d2, d3                \n\t"  // Collect RE and IM on B
          "VTRN.16    d4, d5                \n\t"  // Collect RE and IM on W
          "VMULL.s16  q3, d2, d4            \n\t"  // bi = Br*Wr
          "VMULL.s16  q5, d3, d4            \n\t"  // bq = Bi*Wr
          "VMLAL.s16  q3, d3, d5            \n\t"  // bi = bi + Bi*Wi
          "VMLSL.s16  q5, d2, d5            \n\t"  // bq = bq - Br*Wi
          "VRSHRN.i32 d4, q3, #14           \n\t"  // (bi+8192) >> 14
          "VRSHRN.i32 d5, q5, #14           \n\t"  // (bq+8192) >> 14
          "VTRN.16    d4, d5                \n\t"  // Unpack RE and IM on b
          "VSUB.s16   q1, q0, q2            \n\t"  // B = A - B'
          "VADD.s16   q0, q0, q2            \n\t"  // A = A + B'
          "VST1.32   {d0, d1}, [%[A]]       \n\t"  // Write A
          "VST1.32   {d2, d3}, [%[B]]       \n\t"  // Write B
          :
          : [A] "r"(_A), [B] "r"(_B), [W] "r"(_W)
          : "q0", "q1", "q2", "q3", "q5", "memory");

      sizeIdx -= 4;
      _A += 4;
      _B += 4;
      _W += 4;
    } while (sizeIdx);

    _nblks--;
    _A += _bsize;
    _B += _bsize;
  } while (_nblks);
}

#elif defined __ARM_ARCH_7A__

static void RankB2p(
    sc16_t *_A, sc16_t *_B, sc16_t *_W, u32 _nblks, u32 _bsize) {
  __asm__ __volatile__(
      "MOV        r3,  #0                       \n\t"
      "9:                                       \n\t"
      "MOV       r12, %[W]                      \n\t"
      "MOV        r4, %[bS]                     \n\t"
      "10:                                      \n\t"
      "LDR        r0, [%[A]]                    \n\t"  // Load A
      "LDR        r2, [%[B]]                    \n\t"  // Load B
      "SHADD16    r0,  r0,  r3                  \n\t"  // A >>= 1
      "SHADD16    r2,  r2,  r3                  \n\t"  // B >>= 1
      "LDR        r1, [r12]                     \n\t"  // Load W
      "SMUSDX     r5,  r1,  r2                  \n\t"  //  imaginary
      "SMUAD      r2,  r1,  r2                  \n\t"  //  real
      "ADD        r5,  r5, #8192                \n\t"  //
      "ADD        r2,  r2, #8192                \n\t"  //
      "ASR        r2,  r2, #14                  \n\t"  //
      "PKHBT      r1,  r2,  r5, LSL #2          \n\t"
      "QSUB16     r2,  r0,  r1                  \n\t"  // B = A - B
      "QADD16     r0,  r0,  r1                  \n\t"  // A = A + B
      "STR        r0, [%[A]]                    \n\t"  // Store A
      "STR        r2, [%[B]]                    \n\t"  // Store B
      "ADD      %[A],  %[A],  #4                \n\t"  // A ++
      "ADD      %[B],  %[B],  #4                \n\t"  // B ++
      "ADD       r12, r12,  #4                  \n\t"  // tmpW ++
      "SUBS       r4,  r4,  #1                  \n\t"  // sizeIdx --
      "BNE 10b                                  \n\t"  //
      "ADD        %[A],  %[A],  %[bS], LSL #2   \n\t"  // A += _bsize
      "ADD        %[B],  %[B],  %[bS], LSL #2   \n\t"  // B += _bsize
      "SUBS      %[nB], %[nB],  #1              \n\t"  // _nblks--
      "BNE 9b                                   \n\t"  //
      : [A] "+&r"(_A), [B] "+&r"(_B), [nB] "+&r"(_nblks)
      : [W] "r"(_W), [bS] "r"(_bsize)
      : "r0", "r1", "r2", "r3", "r4", "r5", "r12", "cc", "memory");
}

#else

static void ButterflyBwdDt(sc16_t *_A, sc16_t *_B, sc16_t *_W) {
  s32 bi, bq;

  (*_A).r >>= 1;
  (*_A).i >>= 1;
  (*_B).r >>= 1;
  (*_B).i >>= 1;

  bi = +(*_B).r * (*_W).r + (*_B).i * (*_W).i;
  bq = -(*_B).r * (*_W).i + (*_B).i * (*_W).r;

  bi = (bi + 8192) >> 14;
  bq = (bq + 8192) >> 14;

  (*_B).r = (*_A).r - (s16)bi;
  (*_B).i = (*_A).i - (s16)bq;
  (*_A).r = (*_A).r + (s16)bi;
  (*_A).i = (*_A).i + (s16)bq;
}

static void RankB2p(
    sc16_t *_A, sc16_t *_B, sc16_t *_W, u32 _nblks, u32 _bsize) {
  u32 blkIdx, sizeIdx;
  sc16_t *wbase = _W;

  for (blkIdx = 0; blkIdx < _nblks; blkIdx++) {
    for (sizeIdx = 0; sizeIdx < _bsize; sizeIdx++) {
      ButterflyBwdDt(_A, _B, _W);
      _A++;
      _B++;
      _W++;
    }

    _A += _bsize;
    _B += _bsize;
    _W = wbase;
  }
}

#endif /* __ARM_NEON__ or __ARM_ARCH_7A__ */
