#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "lib/fixed_fft_r2.h"
#include "soft_macq_defines.h"
#include "soft_macq_utils.h"


#define SKIP_FFT_SORT (0)
#define DO_FFT_SORT   (1)
/* MDBZP constants as we assume 3975 MSPS and 15 slices for 7.5 kHz Doppler span */
#define MDBZP_CORR_FFTLEN 512 /* roughly 2 * (3975 / 15) */
#define MDBZP_FREQ_FFTLEN 64  /* roughly 15 * 4 */

static FFT_DECL(MDBZP_CORR_FFTLEN, sCorrFftConf);
static FFT_DECL(MDBZP_FREQ_FFTLEN, sFreqFftConf);

sc16_t cFftWork[MDBZP_CORR_FFTLEN];

sc16_t     pCodeMat[           FAU_CODE_SLICES*MDBZP_CORR_FFTLEN]; /* 4 *      15 *  512 = ~0.03M */
sc16_t   pSignalMat[FAU_MAX_MS*FAU_CODE_SLICES*MDBZP_CORR_FFTLEN]; /* 4 *  5 * 15 *  512 = ~0.2M */
sc16_t     pCorrMat[FAU_MAX_MS*FAU_CODE_SLICES*FAU_CODE_SIZE];     /* 4 *  5 * 15 * 3975 = ~1.2M */
float  pFreqCodeMat[           FAU_CODE_SIZE  *MDBZP_FREQ_FFTLEN]; /* 4 *    3975 *   64 = ~1.0M */

bool mdbzp_static( sc16_t *_cSignal, s8 *_piCode, sFauParams_t *_pPar, acqResults_t *_pRes ) {

  const u32 code_slices = _pPar->iNumCodeSlices;
  const u32 code_ms     = _pPar->iCodeTimeMs;
  const u32 code_coh     = _pPar->iCohCodes;
  const u32 code_nonc     = _pPar->iNcohAcc;
  const u32 samples_ms  = _pPar->iSampMs;         /** samples per ms: 3975 */

  u32 samples_slice;
  u32 samples_block = MDBZP_CORR_FFTLEN;
  u32 freqfft_sz = MDBZP_FREQ_FFTLEN;
  u32 num_codes, iCodeSamp, slices_coh;
  float fResolution;
  u32 max_freqidx, max_codeidx;
  u32 uScaleFactors;

  sc16_t *pCodePt;
  sc16_t *pSignalPt;

#ifdef DEBUG_MDZP
  FILE *fid;
  char fName[64];
#endif

  slices_coh = code_coh * code_slices;       /** e.g. 4 * 15 = 60 */
  iCodeSamp  = samples_ms * code_ms;      /** code samples: 3975 * 1 = 3975 */
  samples_slice = iCodeSamp/code_slices;  /** slice samples: 3975 / 15 = 265 */
  num_codes  = code_coh*code_nonc;

  fResolution = (samples_ms / samples_slice) * 1000 / MDBZP_FREQ_FFTLEN;

  /** alloc forward FFT and support structures */
  /* One-time configuration */
  if (sCorrFftConf.N != samples_block) {
    InitIntFFTr2(&sCorrFftConf, samples_block);
  }

  /***********************************
   *         STORE CODE FFTs         *
   ************************************/
  uScaleFactors = 0x0000;
  for (u32 j=0; j<code_slices; j++) {
    memset(cFftWork, 0, samples_block*sizeof(sc16_t));

    for (u32 i=0; i<samples_slice; i++) {
      cFftWork[i].r = 256 * _piCode[j*samples_slice+i]; /* this effectively zero pads the code */
    }
    DoFwdIntFFTr2( &sCorrFftConf, cFftWork, uScaleFactors, SKIP_FFT_SORT );
    memcpy(pCodeMat+(j*samples_block), cFftWork, samples_block*sizeof(sc16_t));
  }
#ifdef DEBUG_MDZP
  sprintf(fName, "G%02d_codefft.sc16", _pRes->iUsi+1);
  fid = fopen(fName, "wb");
  if (NULL != fid) {
    fwrite(pCodeMat, sizeof(sc16_t), 1*code_slices*samples_block, fid);
    fclose(fid);
  }
#endif


  /*******************************
   *     STORE SIGNAL FFTs       *
   *******************************/
  uScaleFactors = 0x03;
  for (u32 j=0; j<num_codes*code_slices; j++) {
    memset(cFftWork, 0, samples_block*sizeof(sc16_t));

    for (u32 i=0; i<samples_block; i++) {
      cFftWork[i].r = _cSignal[j*samples_slice + i].r;
      cFftWork[i].i = _cSignal[j*samples_slice + i].i;  /* note stride is the slice but fills a block (2xslices) */
    }
    DoFwdIntFFTr2( &sCorrFftConf, cFftWork, uScaleFactors, SKIP_FFT_SORT );
    memcpy(pSignalMat+(j*samples_block), cFftWork, samples_block*sizeof(sc16_t));
  }
#ifdef DEBUG_MDZP
  sprintf(fName, "G%02d_sigfft.sc16", _pRes->iUsi+1);
  fid = fopen(fName, "wb");
  if (NULL != fid) {
    fwrite(pSignalMat, sizeof(sc16_t), num_codes*code_slices*samples_block, fid);
    fclose(fid);
  }
#endif


  /******************************
   * COMPUTE THE BIG DBZP MATRIX
   ******************************/
  uScaleFactors = 0x0003;
  for (u32 j=0; j<code_slices; j++) {
    for (u32 h=0; h<num_codes; h++) {
      for (u32 k=0; k<code_slices; k++) {
        u32 i = (k -j +code_slices) %code_slices;
        u32 m = k +h*code_slices;
        pCodePt   = pCodeMat+(i*samples_block);
        pSignalPt = pSignalMat+(m*samples_block);
        memset(cFftWork, 0, samples_block*sizeof(sc16_t));

        Sc16ArrayMulX(cFftWork, pSignalPt, pCodePt, samples_block);
        DoBwdIntFFTr2( &sCorrFftConf, cFftWork, uScaleFactors, SKIP_FFT_SORT );
        /** note how the instruction below throws half block away */
        memcpy(pCorrMat+(m*iCodeSamp)+j*(samples_slice), cFftWork, samples_slice*sizeof(sc16_t));
      }
    }
  }
//  for(k=0; k<(num_codes*code_slices)*(iCodeSamp); k++) {
//    pCorrMat[k].r /= samples_block;
//    pCorrMat[k].i /= samples_block;
//  }
#ifdef DEBUG_MDZP
  sprintf(fName, "G%02d_mdbzp.sc16", _pRes->iUsi+1);
  fid = fopen(fName, "wb");
  if (NULL != fid) {
    fwrite(pCorrMat, sizeof(sc16_t), (num_codes*code_slices)*(iCodeSamp), fid);
    fclose(fid);
  }
#endif


  /*********************************
   *  COMPUTE FINAL FREQUENCY FFT
   *********************************/
  if (sFreqFftConf.N != samples_block) {
    InitIntFFTr2( &sFreqFftConf, freqfft_sz );
  }

  /** alloc final code-frequency matrix */
  memset(pFreqCodeMat, 0, freqfft_sz*iCodeSamp*sizeof(float));
  uScaleFactors = 0x001F;

  for (u32 m=0; m<iCodeSamp; m++) {
    for (u32 n=0; n < code_nonc; n++) {
      memset(cFftWork, 0, freqfft_sz*sizeof(sc16_t));
      for (u32 j=0; j<code_coh*code_slices; j++) {
        cFftWork[j].r = pCorrMat[(((n*slices_coh)+j)*iCodeSamp) +m].r ;
        cFftWork[j].i = pCorrMat[(((n*slices_coh)+j)*iCodeSamp) +m].i ;
      } /* for j in (code_coh*code_slices) */
      DoFwdIntFFTr2( &sFreqFftConf, cFftWork, uScaleFactors, DO_FFT_SORT );
      Sc16ArrayAddAbsTo(pFreqCodeMat +m*freqfft_sz, cFftWork, freqfft_sz);
    }  /* for n in (code_nonc) */
  } /* for m in (iCodeSamp) */

#ifdef DEBUG_MDZP
  sprintf(fName, "G%02d_codefreq_sc16.float", _pRes->iUsi+1);
  fid = fopen(fName, "wb");
  if (NULL != fid) {
    fwrite(pFreqCodeMat, sizeof(float), freqfft_sz*iCodeSamp, fid);
    fclose(fid);
  }
#endif

  _pRes->iAcqFlag = IsAcquired3D(pFreqCodeMat, iCodeSamp, freqfft_sz, &_pRes->fMaxCorr, &max_codeidx, &max_freqidx);
  _pRes->fCodeDelay += (float) max_codeidx / iCodeSamp;
  _pRes->fDoppFreq = (max_freqidx < freqfft_sz/2 ) ? (float)(+max_freqidx) : (float)(max_freqidx-freqfft_sz);
  _pRes->fDoppFreq *= fResolution;

  return 0;
}
