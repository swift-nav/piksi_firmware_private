#ifndef SOFT_MACQ_DEFINES_H_
#define SOFT_MACQ_DEFINES_H_

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "lib/fixed_fft_r2.h"

#define POW_TWO_P32 (4294967296.0)

#define UTC_SEC_AT_GPS (315964800) /* More or less twenty years */
#define JAN61980 (44244)
#define JAN11901 (15385)

typedef struct _sFauParams {
  /** DBZP parameters */
  u32 iSampMs;
  u8 iCodeTimeMs;     /* how many millisec is a primary code long */
  u32 iNumCodeSlices; /* this dictates the carrier Doppler search space, more
                         slices is wider */
  u8 iCohCodes;       /* coherent integration time */
  u8 iNcohAcc;        /* non-coherent integration time */
  u8 iPostCorrDec;    /* decimation to apply to code resolution post xcorr */
  s8 uSecCode[128];   /* hopefully no secondary codes longer than 128 bits? */
  u32 uSecCodeLen;    /* secondary code length (assuming 1 secondary chip is 1
                         primary code long!) */
} sFauParams_t;

typedef struct _acqResults_t {
  int iUsi;               //! Universal satellite indentifier
  bool bAcquired;         //! The flag controlling the acquisition
  float fMaxCorr;         //! The correlation peak, in some measure
  float fDoppFreq;        //! In Hertz
  float fCodeDelay;       //! The delay in [code]: 0 <= codeDelay < 1
  uint64_t uFirstLocIdx;  //! The first location of the signal used to acquire
} acqResults_t;

/* you be careful when touching these things below >:| */
#define NT1035_VCO1_FREQ 1590000000
#define NT1035_VCO2_FREQ 1235000000

/* current RAW sampling frequency */
#define FAU_RAW_FS 99375000
/* samples per ms */
#define FAU_RAW_SPMS (FAU_RAW_FS / 1000)
/* GPS L1 IF */
#define FAU_FC_GPSL1 (FAU_GPSL1_FREQ - NT1035_VCO1_FREQ)
/* Glonass L1 IF */
#define FAU_FC_GLOG1 (FAU_GLOG1_FREQ - NT1035_VCO1_FREQ)
/* Beidou B1 IF */
#define FAU_FC_BDSB1 (FAU_BDSB1_FREQ - NT1035_VCO1_FREQ)
/* Galileo E1 IF */
#define FAU_FC_GALE1 (FAU_GALE1_FREQ - NT1035_VCO1_FREQ)
/* Galileo E5b IF */
#define FAU_FC_GALE7 (FAU_GALE7_FREQ - NT1035_VCO2_FREQ)
/* Decimation factor to apply to FAU_RAW_FS to get an integer FS */
#define FAU_DECFACT 25
/* Number of slices to cut 1 ms into */
#define FAU_MDBZP_MS_SLICES 15
/* Decimated samples per ms */
#define FAU_SPMS (FAU_RAW_SPMS / FAU_DECFACT)
/* Samples per code for all 1 ms codes - will have to change with Galileo */
#define FAU_CODE_SIZE (FAU_SPMS)
/* Coherent-noncoherent choice for different constellations */
#define FAU_GPSL1CA_COHE 4
#define FAU_GPSL1CA_NONC 1

#define FAU_GLOG1_COHE 4
#define FAU_GLOG1_NONC 1

#define FAU_SBASL1_COHE 1
#define FAU_SBASL1_NONC 4

#define FAU_BDSB11_COHE 1
#define FAU_BDSB11_NONC 4

#define FAU_GALE1_COHE 1
#define FAU_GALE1_NONC 1

#define FAU_GALE5_COHE 2
#define FAU_GALE5_NONC 2

#define FAU_GALE7_COHE 2
#define FAU_GALE7_NONC 2

/* Max number of ms to process at best */
#define FAU_MS_MAX 5

#endif /* SOFT_MACQ_DEFINES_H_ */
