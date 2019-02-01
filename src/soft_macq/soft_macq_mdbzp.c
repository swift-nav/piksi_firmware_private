#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <swiftnav/constants.h>
#include <swiftnav/logging.h>
#include <unistd.h>

#include "lib/fixed_fft_r2.h"
#include "soft_macq_defines.h"
#include "soft_macq_utils.h"

#define SKIP_FFT_SORT (0)
#define DO_FFT_SORT (1)

/* constants assuming 3975 MSPS and 15 slices (7.5 kHz Doppler span) */
#define MDBZP_CORR_FFTLEN 512 /* closest power of two for 2 * (3975 / 15) */
#define MDBZP_FREQ_FFTLEN 64  /* closest power of two for 15 * 4 */

#define MDBZP_CODE_SLICES_MAX (FAU_MDBZP_MS_SLICES * GAL_E1C_PRN_PERIOD_MS)

static FFT_DECL(MDBZP_CORR_FFTLEN, sCorrFftConf);
static FFT_DECL(MDBZP_FREQ_FFTLEN, sFreqFftConf);

sc16_t cFftWork[MDBZP_CORR_FFTLEN];
/* 4* 15 * 512 = <0.1M */
sc16_t pCodeMat[MDBZP_CODE_SLICES_MAX * MDBZP_CORR_FFTLEN];
/* 4*  5 * 15 * 512 = ~0.2M */
sc16_t pSignalMat[FAU_MS_MAX * FAU_MDBZP_MS_SLICES * MDBZP_CORR_FFTLEN];
/* 4*  5 * 15 * 3975 = ~1.2M */
sc16_t pCorrMat[FAU_MS_MAX * MDBZP_CODE_SLICES_MAX * FAU_CODE_SIZE];
/* 4*    3975 *   64 = ~1.0M */
float pFreqCodeMat[FAU_CODE_SIZE * MDBZP_FREQ_FFTLEN * GAL_E1C_PRN_PERIOD_MS];

/** Modified Double Block Zero Padding
 *
 * \param _cSignal input samples (complex int16_t)
 * \param _piCode resampled code
 * \param _piCode resampled code
 * \param _pPar acquisition parameters
 * \param _pRes acquisition results
 *  */
bool mdbzp_static(sc16_t *_cSignal,
                  s8 *_piCode,
                  sFauParams_t *_pPar,
                  acqResults_t *_pRes) {
  const u32 code_slices = _pPar->iNumCodeSlices;
  const u32 code_ms = _pPar->iCodeTimeMs;
  const u32 code_coh = _pPar->iCohCodes;
  const u32 code_nonc = _pPar->iNcohAcc;
  const u32 samples_ms = _pPar->iSampMs; /* samples per ms: 3975 */

  const u32 samples_block = MDBZP_CORR_FFTLEN;
  const u32 freqfft_sz = MDBZP_FREQ_FFTLEN;
  const float freq_resolution =
      ((float)FAU_MDBZP_MS_SLICES) * 1000 / freqfft_sz;

  u32 slices_coh = code_coh * code_slices; /* e.g. 4 * 15 = 60 */
  u32 samples_code = samples_ms * code_ms; /* code samples: 3975 * 1 = 3975 */
  u32 samples_slice =
      samples_code / code_slices; /* slice samples: 3975 / 15 = 265 */
  u32 num_codes = code_coh * code_nonc;

  sc16_t *pCodePt;
  sc16_t *pSignalPt;

  /** alloc forward FFT and support structures */
  /* One-time configuration */
  if (sCorrFftConf.N != samples_block) {
    InitIntFFTr2(&sCorrFftConf, samples_block);
  }

  /***********************************
   *         STORE CODE FFTs         *
   ************************************/
  u32 scale_mask = 0x0000;
  for (u32 j = 0; j < code_slices; j++) {
    memset(cFftWork, 0, samples_block * sizeof(sc16_t));

    for (u32 i = 0; i < samples_slice; i++) {
      /* note how this effectively zero pads the code */
      cFftWork[i].r = 256 * _piCode[j * samples_slice + i];
    }
    DoFwdIntFFTr2(&sCorrFftConf, cFftWork, scale_mask, SKIP_FFT_SORT);
    memcpy(pCodeMat + (j * samples_block),
           cFftWork,
           samples_block * sizeof(sc16_t));
  }

  /*******************************
   *     STORE SIGNAL FFTs       *
   *******************************/
  scale_mask = 0x0000;
  for (u32 j = 0; j < num_codes * code_slices; j++) {
    memset(cFftWork, 0, sizeof(cFftWork));

    for (u32 i = 0; i < samples_block; i++) {
      /* note stride is the slice but fills a block (2 x slices) */
      cFftWork[i].r = _cSignal[j * samples_slice + i].r;
      cFftWork[i].i = _cSignal[j * samples_slice + i].i;
    }
    DoFwdIntFFTr2(&sCorrFftConf, cFftWork, scale_mask, SKIP_FFT_SORT);
    memcpy(pSignalMat + (j * samples_block),
           cFftWork,
           samples_block * sizeof(sc16_t));
  }

  /******************************
   * COMPUTE THE BIG DBZP MATRIX
   ******************************/
  scale_mask = 0x0155;
  for (u32 j = 0; j < code_slices; j++) {
    for (u32 h = 0; h < num_codes; h++) {
      for (u32 k = 0; k < code_slices; k++) {
        u32 i = (k - j + code_slices) % code_slices;
        u32 m = k + h * code_slices;
        pCodePt = pCodeMat + (i * samples_block);
        pSignalPt = pSignalMat + (m * samples_block);
        memset(cFftWork, 0, sizeof(cFftWork));

        Sc16ArrayMulX(cFftWork, pSignalPt, pCodePt, samples_block);
        DoBwdIntFFTr2(&sCorrFftConf, cFftWork, scale_mask, SKIP_FFT_SORT);

        /* note how the instruction below throws half block away */
        memcpy(pCorrMat + (m * samples_code) + j * (samples_slice),
               cFftWork,
               samples_slice * sizeof(sc16_t));
      }
    }
  }

  /*********************************
   *  COMPUTE FINAL FREQUENCY FFT
   *********************************/
  if (sFreqFftConf.N != freqfft_sz) {
    InitIntFFTr2(&sFreqFftConf, freqfft_sz);
  }

  /** alloc final code-frequency matrix */
  memset(pFreqCodeMat, 0, freqfft_sz * samples_code * sizeof(float));
  scale_mask = 0x0155;

  for (u32 m = 0; m < samples_code; m++) {
    for (u32 n = 0; n < code_nonc; n++) {
      memset(cFftWork, 0, sizeof(cFftWork));
      for (u32 j = 0; j < code_coh * code_slices; j++) {
        cFftWork[j].r = pCorrMat[(((n * slices_coh) + j) * samples_code) + m].r;
        cFftWork[j].i = pCorrMat[(((n * slices_coh) + j) * samples_code) + m].i;
      } /* for j in (code_coh*code_slices) */
      DoFwdIntFFTr2(&sFreqFftConf, cFftWork, scale_mask, DO_FFT_SORT);
      Sc16ArrayAddAbsTo(pFreqCodeMat + m * freqfft_sz, cFftWork, freqfft_sz);
    } /* for n in (code_nonc) */
  }   /* for m in (samples_code) */

  float max_freqidx, max_codeidx;
  _pRes->bAcquired = IsAcquired3D(pFreqCodeMat,
                                  samples_code,
                                  freqfft_sz,
                                  code_nonc,
                                  &_pRes->fMaxCorr,
                                  &max_codeidx,
                                  &max_freqidx);
  _pRes->fCodeDelay = (max_codeidx) / samples_code;
  _pRes->fDoppFreq = (max_freqidx < (freqfft_sz / 2))
                         ? max_freqidx
                         : (max_freqidx - freqfft_sz);
  _pRes->fDoppFreq *= freq_resolution;

  return _pRes->bAcquired;
}
