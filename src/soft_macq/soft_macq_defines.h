#ifndef SOFT_MACQ_DEFINES_H_
#define SOFT_MACQ_DEFINES_H_

#include <math.h>
#include <stdint.h>
#include "lib/fixed_fft_r2.h"

#define FAU_GPSL1_FREQ (1575420000)
#define FAU_GPSL1CA_CODE_CHIPS (1023)
#define FAU_GPSL1CA_PRN_BASE (1)
#define FAU_GPSL1CA_CODE_MS (1)
#define FAU_GPSL2_FREQ (1227600000)

#define FAU_GLOG1_FREQ (1602000000)
#define FAU_GLOG1_FOFF (562500)
#define FAU_GLOG1_CODE_CHIPS (511)
#define FAU_GLOG1_PRN_BASE (1)
#define FAU_GLOG1_CODE_MS (1)
#define FAU_GLOG2_FREQ (1246000000)
#define FAU_GLOG2_FOFF (437500)

#define FAU_SBASL1_CODE_MS (1)
#define FAU_SBASL1_PRN_BASE (120)
#define FAU_SBASL1_CODE_CHIPS (1023)

#define FAU_BDS2B1_FREQ (1561098000)
#define FAU_BDS2B2_FREQ (1207014000)
#define FAU_BDS2B11_CODE_MS (1)
#define FAU_BDS2B11_CODE_CHIPS (2046)

#define TWOPI (2.0 * M_PI)
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
  u8 iNcohAcc;        /* coherent integration time */
  u8 iPostCorrDec;    /* decimation to apply to code resolution post xcorr */
  s8 code_sec[128];   /* hopefully no secondary codes longer than 128 bits? */
  u32 code_sec_len;   /* secondary code length (assuming 1 secondary chip is 1
                         primary code long!) */
} sFauParams_t;

typedef struct _acqResults_t {
  int iUsi;               //! The flag controlling the acquisition
  int iAcqFlag;           //! The flag controlling the acquisition
  float fMaxCorr;         //! The correlation peak, in some measure
  float fDoppFreq;        //! In Hertz
  float fCodeDelay;       //! The delay in [code]: 0 <= codeDelay < 1
  uint64_t uFirstLocIdx;  //! The first location of the signal used to acquire
} acqResults_t;

/* you be careful when touching these things below >:| */
#define NT1035_VCO1_FREQ (1590000000)
#define NT1035_VCO2_FREQ (1235000000)

#define FAU_RAW_FS (99375000) /* current RAW sampling frequency */

#define FAU_RAW_SPMS (FAU_RAW_FS / 1000)

#define FAU_FC_GPSL1 \
  (FAU_GPSL1_FREQ - NT1035_VCO1_FREQ) /* IF of GPS L1 -high side injection */

#define FAU_FC_GLOG1 \
  (FAU_GLOG1_FREQ -  \
   NT1035_VCO1_FREQ) /* IF of Glonass G1 - low side injection */

#define FAU_FC_BDS2B1 (FAU_BDS2B1_FREQ - NT1035_VCO1_FREQ) /* IF of Beidou B1 \
                                                              */

#define FAU_DECFACT (25)
#define FAU_CODE_SLICES (15)
#define FAU_SPMS (FAU_RAW_SPMS / FAU_DECFACT)
#define FAU_CODE_SIZE (FAU_SPMS * FAU_GPSL1CA_CODE_MS)

#define FAU_GPSL1CA_COHE (4)
#define FAU_GPSL1CA_NONC (1)

#define FAU_GLOG1_COHE (2)
#define FAU_GLOG1_NONC (2)
#define FAU_GLOG1_CODE_SLICES (15)

#define FAU_SBASL1_COHE (1)
#define FAU_SBASL1_NONC (4)
#define FAU_SBASL1_CODE_SLICES (5)

#define FAU_BDS2B11_COHE (1)
#define FAU_BDS2B11_NONC (4)
#define FAU_BDS2B11_CODE_SLICES (5)

#define FAU_MAX_MS (1 + (FAU_GPSL1CA_COHE * FAU_GPSL1CA_NONC))

#endif /* SOFT_MACQ_DEFINES_H_ */
