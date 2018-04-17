#ifndef SOFT_MACQ_DEFINES_H_
#define SOFT_MACQ_DEFINES_H_

#include <math.h>
#include <stdint.h>
#include "lib/fixed_fft_r2.h"

#define SOFTMACQ_DEF_F_REF (1023000)

#define SOFTMACQ_GPSL1_FREQ (1575420000)
#define SOFTMACQ_GPSL1CA_CODE_CHIPS (1023)
#define SOFTMACQ_GPSL1CA_PRN_BASE (1)
#define SOFTMACQ_GPSL1CA_CODE_MS (1)

#define SOFTMACQ_SBASL1_CODE_MS (1)
#define SOFTMACQ_SBASL1_PRN_BASE (120)
#define SOFTMACQ_SBASL1_CODE_CHIPS (1023)

#define SOFTMACQ_GPSL2_FREQ (1227600000)

#define SOFTMACQ_GLOG1_FREQ (1602000000)
#define SOFTMACQ_GLOG1_FOFF (562500)
#define SOFTMACQ_GLOG1_CODE_CHIPS (511)
#define SOFTMACQ_GLOG1_PRN_BASE (1)
#define SOFTMACQ_GLOG1_CODE_MS (1)

#define SOFTMACQ_GLOG2_FREQ (1246000000)
#define SOFTMACQ_GLOG2_FOFF (437500)

#define SOFTMACQ_BDS2B1_FREQ (1561098000)
#define SOFTMACQ_BDS2B2_FREQ (1207014000)

#define TWOPI (2.0 * M_PI)
#define HALFPI (M_PI / 2.0)
#define FOURTHPI (M_PI / 4.0)
#define POW_TWO_M05 (3.125000000000000e-02)
#define POW_TWO_M11 (4.882812500000000e-04)
#define POW_TWO_M12 (2.441406250000000e-04)
#define POW_TWO_M13 (1.220703125000000e-04)
#define POW_TWO_M19 (1.907348632812500e-06)
#define POW_TWO_M20 (9.536743164062500e-07)
#define POW_TWO_M21 (4.768371582031250e-07)
#define POW_TWO_M23 (1.192092895507812e-07)
#define POW_TWO_M24 (5.960464477539062e-08)
#define POW_TWO_M27 (7.450580596923828e-09)
#define POW_TWO_M29 (1.862645149230957e-09)
#define POW_TWO_M30 (9.313225746154785e-10)
#define POW_TWO_M31 (4.656612873077393e-10)
#define POW_TWO_M33 (1.164153218269348e-10)
#define POW_TWO_M38 (3.637978807091713e-12)
#define POW_TWO_M43 (1.136868377216160e-13)
#define POW_TWO_M50 (8.881784197001252e-16)
#define POW_TWO_M55 (2.775557561562891e-17)
#define POW_TWO_P11 (2048.0)
#define POW_TWO_P12 (4096.0)
#define POW_TWO_P14 (16384.0)
#define POW_TWO_P16 (65536.0)
#define POW_TWO_P32 (4294967296.0)

#define SEC_PER_MIN (60)
#define SEC_PER_HOUR (SEC_PER_MIN * 60)
#define SEC_PER_DAY (SEC_PER_HOUR * 24)
#define SEC_PER_WEEK (SEC_PER_DAY * 7)
#define UTC_SEC_AT_GPS (315964800) /* More or less twenty years */
#define JAN61980 (44244)
#define JAN11901 (15385)

typedef struct _sFauParams {
  /** DBZP parameters */
  uint32_t iSampMs;
  int32_t iCodeTimeMs; /* how many millisec is a primary code long */
  int32_t iPostCorrDec;
  int32_t iNumCodeSlices; /* this dictates the carrier Doppler search space,
                             more slices is wider */
  int32_t iCohCodes;      /* coherent integration time */
  int32_t iNcohAcc;       /* coherent integration time */
  int8_t
      pSecondary[128]; /* hopefully no secondary codes longer than 128 bits? */
  int32_t
      iSecondaryLen; /* secondary code length (assuming 1 secondary chip is 1
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

#define SOFTMACQ_RAW_FS (99375000) /** current RAW sampling frequency */
#define SOFTMACQ_RAW_SPMS (SOFTMACQ_RAW_FS / 1000)
#define SOFTMACQ_FC_GPSL1 \
  (SOFTMACQ_GPSL1_FREQ -  \
   NT1035_VCO1_FREQ) /** IF of GPS L1 -high side injection */
#define SOFTMACQ_FC_GPSL2 \
  (SOFTMACQ_GPSL2_FREQ -  \
   NT1035_VCO2_FREQ) /** IF of GPS L2 -high side injection */
#define SOFTMACQ_FC_GLOG1 \
  (SOFTMACQ_GLOG1_FREQ -  \
   NT1035_VCO1_FREQ) /** IF of Glonass G1 - low side injection */
#define SOFTMACQ_FC_GLOG2 \
  (SOFTMACQ_GLOG2_FREQ -  \
   NT1035_VCO2_FREQ) /** IF of Glonass G2 - low side injection */
#define SOFTMACQ_FC_BDS2B1 \
  (SOFTMACQ_BDS2B1_FREQ - NT1035_VCO1_FREQ) /** IF of Beidou B1 */
#define SOFTMACQ_FC_BDS2B2 \
  (SOFTMACQ_BDS2B2_FREQ - NT1035_VCO2_FREQ) /** IF of Beidou B2 -high side */
#define SOFTMACQ_DECFACT_GPSL1CA (25)
#define SOFTMACQ_DECFACT_GPSL2 (25)
#define SOFTMACQ_DECFACT_SBASL1 (25)
#define SOFTMACQ_DECFACT_GALE1BC (25)
#define SOFTMACQ_DECFACT_BDS2B1 (25)
#define SOFTMACQ_DECFACT_BDS2B2 (25)
#define SOFTMACQ_DECFACT_GLOG1 (25)
#define SOFTMACQ_DECFACT_GLOG2 (25)

#define SOFTMACQ_GPSL1CA_COHE (4)
#define SOFTMACQ_GPSL1CA_NONC (1)
#define SOFTMACQ_GPSL1CA_CODE_SLICES (15)
#define SOFTMACQ_GPSL1CA_CODE_SIZE \
  (SOFTMACQ_RAW_SPMS / SOFTMACQ_DECFACT_GPSL1CA * SOFTMACQ_GPSL1CA_CODE_MS)

#define SOFTMACQ_GLOG1_COHE (2)
#define SOFTMACQ_GLOG1_NONC (2)
#define SOFTMACQ_GLOG1_CODE_SLICES (15)
#define SOFTMACQ_GLOG1_CODE_SIZE \
  (SOFTMACQ_RAW_SPMS / SOFTMACQ_DECFACT_GLOG1 * SOFTMACQ_GLOG1_CODE_MS)

#define SOFTMACQ_SBASL1_COHE (1)
#define SOFTMACQ_SBASL1_NONC (4)
#define SOFTMACQ_SBASL1_CODE_SLICES (5)
#define SOFTMACQ_SBASL1_CODE_SIZE \
  (SOFTMACQ_RAW_SPMS / SOFTMACQ_DECFACT_SBASL1 * SOFTMACQ_SBASL1_CODE_MS)

#define SOFTMACQ_MAX_SPMS (SOFTMACQ_RAW_SPMS / SOFTMACQ_DECFACT_GPSL1CA)
#define SOFTMACQ_MAX_MS (1 + (SOFTMACQ_GPSL1CA_COHE * SOFTMACQ_GPSL1CA_NONC))

#endif /* SOFT_MACQ_DEFINES_H_ */
