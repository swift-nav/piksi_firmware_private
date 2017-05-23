#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "lib/fixed_fft_r2.h"
#include "soft_macq_defines.h"
#include "soft_macq_utils.h"


intFFTr2_t sCorrFftConf;
intFFTr2_t sFreqFftConf;

sc16_t cFftWork[INTFFT_MAXSIZE];

#define SKIP_FFT_SORT (0)
#define DO_FFT_SORT   (1)

sc16_t     pCodeMat[           SOFTMACQ_GPSL1CA_CODE_SLICES*INTFFT_MAXSIZE];         /* 4 *      15 *  512 = ~0.03M */
sc16_t   pSignalMat[SOFTMACQ_MAX_MS*SOFTMACQ_GPSL1CA_CODE_SLICES*INTFFT_MAXSIZE];         /* 4 *  5 * 15 *  512 = ~0.2M */
sc16_t     pCorrMat[SOFTMACQ_MAX_MS*SOFTMACQ_GPSL1CA_CODE_SLICES*SOFTMACQ_GPSL1CA_CODE_SIZE];  /* 4 *  5 * 15 * 3975 = ~1.2M */
float  pFreqCodeMat[           SOFTMACQ_GPSL1CA_CODE_SIZE  *SOFTMACQ_FREQFFT_SIZE];       /* 4 *    3975 *   64 = ~1.0M */
//float pFreqCodeTemp[                    SOFTMACQ_GPSL1CA_CODE_SIZE*INTFFT_MAXSIZE];
float pFreqCodeTemp[2];         /* just to fit in stock firmware, secondary code search capability will not work */


uint32_t uScaleFactors;


int mdbzp_static( sc16_t *_cSignal, int8_t *_piCode, sFauParams_t *_pPar, acqResults_t *_pRes ) {

  const int iCodeSlices = _pPar->iNumCodeSlices;
  const int iCodeMs     = _pPar->iCodeTimeMs;
  const int iCohCod     = _pPar->iCohCodes;
  const int iNcohAc     = _pPar->iNcohAcc;
  int iSpms, iSliceSamp, iBlockSamp, iBkLo, iBkHi, iNumCodes, iCodeSamp, iCohSlices, iFreqFftSz;
  float fTempMax, fResolution, fAbsoluteMax=0;
  int iSign, iSecIdx=0, iMaxFreqIdx, iMaxCodeIdx;
  int i,j,h,k,m,n,q,r;

  sc16_t *pCodePt;
  sc16_t *pSignalPt;

#ifdef DEBUG_MDZP
  FILE *fid;
  char fName[64];
#endif

  iSpms      = _pPar->iSampMs;                          /** samples per ms: 3975 */
  iCohSlices = iCohCod*iCodeSlices;                     /** e.g. 4*15 = 60 */
  iCodeSamp  = iSpms * iCodeMs;                         /** code samples: 3975 * 1 = 3975 */
  iSliceSamp = iCodeSamp/iCodeSlices;                   /** slice samples: 3975 / 15 = 265 */
  // fMaxSliceFreq = iCodeSlices / (0.001*iCodeMs);  11 / 0.004 = 2750
  iBkLo = powf(2, ceilf(log2f(iSliceSamp)));            /** 512  */
  iBkHi = 2*iBkLo;                                      /** 1024 */
  iBlockSamp = (iBkHi-(2*iSliceSamp)) < ((2*iSliceSamp)-iBkLo) ? iBkHi : iBkLo;
  //  iBlockSamp = iBkHi;
  iNumCodes  = iCohCod*iNcohAc;
  iFreqFftSz = powf(2,  (ceilf(log2f(iCohSlices))));    /** 64 */

  fResolution = (iSpms / iSliceSamp)*1000/iFreqFftSz;

  /** alloc forward FFT and support structures */
  InitIntFFTr2( &sCorrFftConf, iBlockSamp );


  /***********************************
   *         STORE CODE FFTs         *
   ************************************/
  uScaleFactors = 0x0000;
  for (j=0; j<iCodeSlices; j++) {
    memset(cFftWork, 0, iBlockSamp*sizeof(sc16_t));

    for (i=0; i<iSliceSamp; i++) {
      cFftWork[i].r = 256 * _piCode[j*iSliceSamp+i]; /* this effectively zero pads the code */
    }
    DoFwdIntFFTr2( &sCorrFftConf, cFftWork, uScaleFactors, SKIP_FFT_SORT );
    memcpy(pCodeMat+(j*iBlockSamp), cFftWork, iBlockSamp*sizeof(sc16_t));
  }
#ifdef DEBUG_MDZP
  sprintf(fName, "G%02d_codefft.sc16", _pRes->iUsi+1);
  fid = fopen(fName, "wb");
  if (NULL != fid) {
    fwrite(pCodeMat, sizeof(sc16_t), 1*iCodeSlices*iBlockSamp, fid);
    fclose(fid);
  }
#endif


  /*******************************
   *     STORE SIGNAL FFTs       *
   *******************************/
  uScaleFactors = 0x03;
  for (j=0; j<iNumCodes*iCodeSlices; j++) {
    //~ k = -rintf(j*iSliceSamp*(_pPar->fDoppler)/GPS_OS_FREQ);
    k = 0;
    memset(cFftWork, 0, iBlockSamp*sizeof(sc16_t));

    for (i=0; i<iBlockSamp; i++) {
      cFftWork[i].r = _cSignal[j*iSliceSamp + i + k].r;
      cFftWork[i].i = _cSignal[j*iSliceSamp + i + k].i;  /* note stride is the slice but fills a block (2xslices) */
    }
    DoFwdIntFFTr2( &sCorrFftConf, cFftWork, uScaleFactors, SKIP_FFT_SORT );
    memcpy(pSignalMat+(j*iBlockSamp), cFftWork, iBlockSamp*sizeof(sc16_t));
  }
#ifdef DEBUG_MDZP
  sprintf(fName, "G%02d_sigfft.sc16", _pRes->iUsi+1);
  fid = fopen(fName, "wb");
  if (NULL != fid) {
    fwrite(pSignalMat, sizeof(sc16_t), iNumCodes*iCodeSlices*iBlockSamp, fid);
    fclose(fid);
  }
#endif


  /******************************
   * COMPUTE THE BIG DBZP MATRIX
   ******************************/
  uScaleFactors = 0x0003;
  for (j=0; j<iCodeSlices; j++) {
    for (h=0; h<iNumCodes; h++) {
      for (k=0; k<iCodeSlices; k++) {
        i = (k -j +iCodeSlices) %iCodeSlices;
        m = k +h*iCodeSlices;
        pCodePt   = pCodeMat+(i*iBlockSamp);
        pSignalPt = pSignalMat+(m*iBlockSamp);
        memset(cFftWork, 0, iBlockSamp*sizeof(sc16_t));

        Sc16ArrayMulX(cFftWork, pSignalPt, pCodePt, iBlockSamp);
        DoBwdIntFFTr2( &sCorrFftConf, cFftWork, uScaleFactors, SKIP_FFT_SORT );
        /** note how the instruction below throws half block away */
        memcpy(pCorrMat+(m*iCodeSamp)+j*(iSliceSamp), cFftWork, iSliceSamp*sizeof(sc16_t));
      }
    }
  }
//  for(k=0; k<(iNumCodes*iCodeSlices)*(iCodeSamp); k++) {
//    pCorrMat[k].r /= iBlockSamp;
//    pCorrMat[k].i /= iBlockSamp;
//  }
#ifdef DEBUG_MDZP
  sprintf(fName, "G%02d_mdbzp.sc16", _pRes->iUsi+1);
  fid = fopen(fName, "wb");
  if (NULL != fid) {
    fwrite(pCorrMat, sizeof(sc16_t), (iNumCodes*iCodeSlices)*(iCodeSamp), fid);
    fclose(fid);
  }
#endif


  /*********************************
   *  COMPUTE FINAL FREQUENCY FFT
   *********************************/
  InitIntFFTr2( &sFreqFftConf, iFreqFftSz );

  /** alloc final code-frequency matrix */
  memset(pFreqCodeMat, 0, iFreqFftSz*iCodeSamp*sizeof(float));
  uScaleFactors = 0x001F;

  if (_pPar->iSecondaryLen) {
    for (q=0; q<_pPar->iSecondaryLen; q++) {  /* browse all possible secondary code starts */
      memset(pFreqCodeTemp, 0, iFreqFftSz*iCodeSamp*sizeof(float));
      for (m=0; m<iCodeSamp; m++) {           /* for every possible code shift */
        r = q;
        for (n=0; n<iNcohAc; n++) {           /* stride across non-coherent accumulations */
          memset(cFftWork, 0, iFreqFftSz*sizeof(sc16_t));


          for (j=0; j<iCohCod; j++) {         /* code by code increment secondary chip */
            iSign = _pPar->pSecondary[r];
            for (i=0; i<iCodeSlices; i++) {   /* all slices multiplied by the same secondary chip */
              k = (n*iCohCod +j)*iCodeSlices + i;
              h =             j *iCodeSlices + i;
              cFftWork[h].r = iSign * pCorrMat[k*iCodeSamp +m].r;
              cFftWork[h].i = iSign * pCorrMat[k*iCodeSamp +m].i;
            }
            r = (r+1) %(_pPar->iSecondaryLen);
          }
          DoFwdIntFFTr2( &sFreqFftConf, cFftWork, uScaleFactors, DO_FFT_SORT );
          Sc16ArrayAddAbsTo(pFreqCodeTemp +m*iFreqFftSz, cFftWork, iFreqFftSz);
        }
      }
      fTempMax = FloatMax(pFreqCodeTemp, iFreqFftSz*iCodeSamp);
      if (fTempMax>fAbsoluteMax) {
        memcpy(pFreqCodeMat, pFreqCodeTemp, iFreqFftSz*iCodeSamp*sizeof(float));
        fAbsoluteMax = fTempMax;
        iSecIdx = q;
      }
    }
  } else {
    for (m=0; m<iCodeSamp; m++) {
      for (n=0; n < iNcohAc; n++) {
        memset(cFftWork, 0, iFreqFftSz*sizeof(sc16_t));
        for (j=0; j<iCohCod*iCodeSlices; j++) {
          cFftWork[j].r = pCorrMat[(((n*iCohSlices)+j)*iCodeSamp) +m].r ;
          cFftWork[j].i = pCorrMat[(((n*iCohSlices)+j)*iCodeSamp) +m].i ;
        } /* for j in (iCohCod*iCodeSlices) */
        DoFwdIntFFTr2( &sFreqFftConf, cFftWork, uScaleFactors, DO_FFT_SORT );
        Sc16ArrayAddAbsTo(pFreqCodeMat +m*iFreqFftSz, cFftWork, iFreqFftSz);
      }  /* for n in (iNcohAc) */
    } /* for m in (iCodeSamp) */
  }
#ifdef DEBUG_MDZP
  sprintf(fName, "G%02d_codefreq_sc16.float", _pRes->iUsi+1);
  fid = fopen(fName, "wb");
  if (NULL != fid) {
    fwrite(pFreqCodeMat, sizeof(float), iFreqFftSz*iCodeSamp, fid);
    fclose(fid);
  }
#endif

  _pRes->iAcqFlag = IsAcquired3D(pFreqCodeMat, iCodeSamp, iFreqFftSz, &_pRes->fMaxCorr, &iMaxCodeIdx, &iMaxFreqIdx);
  _pRes->fCodeDelay += (float) iMaxCodeIdx / iCodeSamp + iSecIdx;
  _pRes->fDoppFreq = (iMaxFreqIdx < iFreqFftSz/2 ) ? (float)(+iMaxFreqIdx) : (float)(iMaxFreqIdx-iFreqFftSz);
  _pRes->fDoppFreq *= fResolution;

  return 0;
}
