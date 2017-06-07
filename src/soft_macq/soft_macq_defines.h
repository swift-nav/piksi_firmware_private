#ifndef SOFT_MACQ_DEFINES_H_
#define SOFT_MACQ_DEFINES_H_

#include <stdint.h>
#include "lib/fixed_fft_r2.h"

#define ENAGPS
#undef  ENAGLO
#undef  ENABEI
#undef  ENASBS
#undef  ENAGAL
#undef  ENAIRN
#undef  ENAQZS

#define SOFTMACQ_DEF_F_REF                (1023000)

#define SOFTMACQ_GPSL1_FREQ               (1575420000)
#define SOFTMACQ_GPSL1CA_CODE_CHIPS       (1023)
#define SOFTMACQ_GPSL1CA_PRN_BASE         (1)
#define SOFTMACQ_GPSL1CA_CODE_MS          (1)


#define SOFTMACQ_SBASL1_CODE_MS           (1)
#define SOFTMACQ_SBASL1_PRN_BASE          (120)
#define SOFTMACQ_SBASL1_CODE_CHIPS        (1023)

#define SOFTMACQ_GPSL2_FREQ               (1227600000)


#define SOFTMACQ_GLOG1_FREQ               (1602000000)
#define SOFTMACQ_GLOG1_FOFF               (562500)
#define SOFTMACQ_GLOG1_CODE_CHIPS         (511)
#define SOFTMACQ_GLOG1_PRN_BASE           (1)
#define SOFTMACQ_GLOG1_CODE_MS            (1)


#define SOFTMACQ_GLOG2_FREQ               (1246000000)
#define SOFTMACQ_GLOG2_FOFF               (437500)

#define SOFTMACQ_BEIB1_FREQ               (1561098000)
#define SOFTMACQ_BEIB2_FREQ               (1207014000)

#define SOFTMACQ_GAL_NUMSAT               (30)
#define SOFTMACQ_GAL_SATBASE              (1)
#define SOFTMACQ_GALE1B_FREQ_MULT         (1540)
#define SOFTMACQ_GALE1B_FREQ              (GALE1B_FREQ_MULT*DEF_F_REF)
#define SOFTMACQ_GALE1B_CR                (1.023e6)
#define SOFTMACQ_GALE1B_CODE_TIMELEN      (0.004)
#define SOFTMACQ_GALE1B_CODE_TIMELEN_MS   (4)
#define SOFTMACQ_GALE1B_CODECHIPS         (4092)
#define SOFTMACQ_GALE1B_CODECHIPS1MS      (1023)
#define SOFTMACQ_GALE1B_MODSCHEMA         BOC_1_1

#define SOFTMACQ_GALE1C_FREQ_MULT         (1540)
#define SOFTMACQ_GALE1C_FREQ              (GALE1C_FREQ_MULT*DEF_F_REF)
#define SOFTMACQ_GALE1C_CR                (1.023e6)
#define SOFTMACQ_GALE1C_CODE_TIMELEN      (0.004)
#define SOFTMACQ_GALE1C_CODE_MS           (4)
#define SOFTMACQ_GALE1C_CODECHIPS         (4092)
#define SOFTMACQ_GALE1C_MODSCHEMA         BOC_1_1
#define SOFTMACQ_GALE1C_SECCODECHIPS      (25)

#define GAL_INAV_SYNCLEN              (10)
#define GAL_INAV_PAGEPART_SYMBLEN     (240)
#define GAL_INAV_PAGEPART_TOTLEN      (GAL_INAV_SYNCLEN+GAL_INAV_PAGEPART_SYMBLEN)  /* 250 */
#define GAL_INAV_PAGEPART_PAYLOADBITS (114)
#define GAL_INAV_PAGEPART_TAILBITS    (6)
#define GAL_INAV_PAGEPART_TOTBITS     (GAL_INAV_PAGEPART_PAYLOADBITS+GAL_INAV_PAGEPART_TAILBITS)  /* 120 */
#define GAL_INAV_PAGE_EVE_SIZE        (112)
#define GAL_INAV_PAGE_ODD_SIZE        (16)
#define GAL_INAV_PAGE_SIZE            (GAL_INAV_PAGE_EVE_SIZE+GAL_INAV_PAGE_ODD_SIZE)
#define GAL_INAV_DEINT_COL            (8)
#define GAL_INAV_DEINT_ROW            (30)

#define PI (3.14159265358979)
#define TWOPI    (2.0 * PI)
#define HALFPI   (PI / 2.0)
#define FOURTHPI (PI / 4.0)
#define POW_TWO_M05 ( 3.125000000000000e-02 )
#define POW_TWO_M11 ( 4.882812500000000e-04 )
#define POW_TWO_M12 ( 2.441406250000000e-04 )
#define POW_TWO_M13 ( 1.220703125000000e-04 )
#define POW_TWO_M19 ( 1.907348632812500e-06 )
#define POW_TWO_M20 ( 9.536743164062500e-07 )
#define POW_TWO_M21 ( 4.768371582031250e-07 )
#define POW_TWO_M23 ( 1.192092895507812e-07 )
#define POW_TWO_M24 ( 5.960464477539062e-08 )
#define POW_TWO_M27 ( 7.450580596923828e-09 )
#define POW_TWO_M29 ( 1.862645149230957e-09 )
#define POW_TWO_M30 ( 9.313225746154785e-10 )
#define POW_TWO_M31 ( 4.656612873077393e-10 )
#define POW_TWO_M33 ( 1.164153218269348e-10 )
#define POW_TWO_M38 ( 3.637978807091713e-12 )
#define POW_TWO_M43 ( 1.136868377216160e-13 )
#define POW_TWO_M50 ( 8.881784197001252e-16 )
#define POW_TWO_M55 ( 2.775557561562891e-17 )
#define POW_TWO_P11 (                2048.0 )
#define POW_TWO_P12 (                4096.0 )
#define POW_TWO_P14 (               16384.0 )
#define POW_TWO_P16 (               65536.0 )
#define POW_TWO_P32 (          4294967296.0 )

#define SEC_PER_MIN     (60)
#define SEC_PER_HOUR    (SEC_PER_MIN * 60)
#define SEC_PER_DAY     (SEC_PER_HOUR * 24)
#define SEC_PER_WEEK    (SEC_PER_DAY * 7)
#define UTC_SEC_AT_GPS  (315964800)        /* More or less twenty years */
#define JAN61980        (44244)
#define JAN11901        (15385)

#define RAD2DEG       (180.0 / PI)
#define DEG2RAD       (PI / 180.0)
#define LIGHT_SPEED   (2.99792458e+08)

#define GPS_PRN_MIN (0)
#ifdef ENAGPS
#define NUM_GPS_SVS (32)
#else
#define NUM_GPS_SVS (0)
#endif

#define SBS_PRN_MIN (GPS_PRN_MIN+NUM_GPS_SVS)
#ifdef ENASBS
#define NUM_SBS_SVS (19)
#else
#define NUM_SBS_SVS ( 0)
#endif

#define GLO_PRN_MIN (SBS_PRN_MIN+NUM_SBS_SVS)
#ifdef ENAGLO
#define NUM_GLO_SVS (28)
#else
#define NUM_GLO_SVS ( 0)
#endif

#define GAL_PRN_MIN (GLO_PRN_MIN+NUM_GLO_SVS)
#ifdef ENAGAL
#define NUM_GAL_SVS (30)
#else
#define NUM_GAL_SVS ( 0)
#endif

#define IRN_PRN_MIN (GAL_PRN_MIN+NUM_GAL_SVS)
#ifdef ENAIRN
#define NUM_IRN_SVS (19)
#else
#define NUM_IRN_SVS ( 0)
#endif

#define BEI_PRN_MIN (IRN_PRN_MIN+NUM_IRN_SVS)
#ifdef ENABEI
#define NUM_BEI_SVS (30)
#else
#define NUM_BEI_SVS ( 0)
#endif

#define QZS_PRN_MIN (BEI_PRN_MIN+NUM_BEI_SVS)
#ifdef ENAQZS
#define NUM_QZS_SVS (19)
#else
#define NUM_QZS_SVS ( 0)
#endif

#define MAX_NUMSVS (NUM_GPS_SVS+NUM_GAL_SVS+NUM_GLO_SVS+NUM_SBS_SVS+NUM_IRN_SVS+NUM_BEI_SVS)

typedef struct _fc32 { float r;   float i;   } fc32_t;

typedef struct _sFauParams {
  /** DBZP parameters */
  uint32_t iSampMs;
  int32_t iCodeTimeMs;      /* how many millisec is a primary code long */
  int32_t iPostCorrDec;
  int32_t iNumCodeSlices;   /* this dictates the carrier Doppler search space, more slices is wider */
  int32_t iCohCodes;        /* coherent integration time */
  int32_t iNcohAcc;         /* coherent integration time */
  int8_t  pSecondary[128];  /* hopefully no secondary codes longer than 128 bits? */
  int32_t iSecondaryLen;    /* secondary code length (assuming 1 secondary chip is 1 primary code long!) */
} sFauParams_t;


typedef struct _acqResults_t {
  int   iUsi;               //! The flag controlling the acquisition
  int   iAcqFlag;           //! The flag controlling the acquisition
  float fMaxCorr;           //! The correlation peak, in some measure
  float fDoppFreq;          //! In Hertz
  float fCodeDelay;         //! The delay in [code]: 0 <= codeDelay < 1
  uint64_t uFirstLocIdx;    //! The first location of the signal used to acquire
} acqResults_t;


typedef enum {
  SOFTMACQ_INIT_GPSL1CA, SOFTMACQ_INIT_GALE1BC, SOFTMACQ_INIT_GLOG1, SOFTMACQ_INIT_BEIB1,
  SOFTMACQ_INIT_GPSL2C,  SOFTMACQ_INIT_GALE6BC, SOFTMACQ_INIT_GLOG2, SOFTMACQ_INIT_BEIB2
} eFauType_t;

/* you be careful when touching these things below >:| */
#define NT1035_VCO1_FREQ              (1590000000)
#define NT1035_VCO2_FREQ              (1235000000)

#define SOFTMACQ_RAW_FS               (99375000)  /** current RAW sampling frequency */
#define SOFTMACQ_RAW_SPMS             (SOFTMACQ_RAW_FS/1000)
#define SOFTMACQ_FC_GPSL1             (SOFTMACQ_GPSL1_FREQ - NT1035_VCO1_FREQ) /** IF of GPS L1 -high side injection */
#define SOFTMACQ_FC_GPSL2             (SOFTMACQ_GPSL2_FREQ - NT1035_VCO2_FREQ) /** IF of GPS L2 -high side injection */
#define SOFTMACQ_FC_GLOG1             (SOFTMACQ_GLOG1_FREQ - NT1035_VCO1_FREQ) /** IF of Glonass G1 - low side injection */
#define SOFTMACQ_FC_GLOG2             (SOFTMACQ_GLOG2_FREQ - NT1035_VCO2_FREQ) /** IF of Glonass G2 - low side injection */
#define SOFTMACQ_FC_BEIB1             (SOFTMACQ_BEIB1_FREQ - NT1035_VCO1_FREQ) /** IF of Beidou B1 */
#define SOFTMACQ_FC_BEIB2             (SOFTMACQ_BEIB2_FREQ - NT1035_VCO2_FREQ) /** IF of Beidou B2 -high side */
#define SOFTMACQ_DECFACT_GPSL1CA      (25)
#define SOFTMACQ_DECFACT_GPSL2        (25)
#define SOFTMACQ_DECFACT_SBASL1       (25)
#define SOFTMACQ_DECFACT_GALE1BC      (25)
#define SOFTMACQ_DECFACT_BEIB1        (25)
#define SOFTMACQ_DECFACT_BEIB2        (25)
#define SOFTMACQ_DECFACT_GLOG1        (25)
#define SOFTMACQ_DECFACT_GLOG2        (25)

#define SOFTMACQ_GPSL1CA_COHE         (4)
#define SOFTMACQ_GPSL1CA_NONC         (1)
#define SOFTMACQ_GPSL1CA_CODE_SLICES  (15)
#define SOFTMACQ_GPSL1CA_CODE_SIZE    (SOFTMACQ_RAW_SPMS / SOFTMACQ_DECFACT_GPSL1CA * SOFTMACQ_GPSL1CA_CODE_MS)

#define SOFTMACQ_GLOG1_COHE           (2)
#define SOFTMACQ_GLOG1_NONC           (2)
#define SOFTMACQ_GLOG1_CODE_SLICES    (15)
#define SOFTMACQ_GLOG1_CODE_SIZE      (SOFTMACQ_RAW_SPMS / SOFTMACQ_DECFACT_GLOG1 *   SOFTMACQ_GLOG1_CODE_MS  )

#define SOFTMACQ_SBASL1_COHE          (1)
#define SOFTMACQ_SBASL1_NONC          (4)
#define SOFTMACQ_SBASL1_CODE_SLICES   (5)
#define SOFTMACQ_SBASL1_CODE_SIZE     (SOFTMACQ_RAW_SPMS / SOFTMACQ_DECFACT_SBASL1 *  SOFTMACQ_SBASL1_CODE_MS )

#define SOFTMACQ_MAX_SPMS             (SOFTMACQ_RAW_SPMS / SOFTMACQ_DECFACT_GPSL1CA)
#define SOFTMACQ_MAX_MS               (1+(SOFTMACQ_GPSL1CA_COHE*SOFTMACQ_GPSL1CA_NONC))
#define SOFTMACQ_FREQFFT_SIZE         (64)

#endif /* SOFT_MACQ_DEFINES_H_ */
