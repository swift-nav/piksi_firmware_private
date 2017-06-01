#include <ch.h>
#include <assert.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include <libswiftnav/prns.h>
#include <libswiftnav/logging.h>

#include "nap/nap_common.h"
#include "nap/nap_constants.h"
#include "board/v3/nap/grabber.h"

#include "lib/fixed_fft_r2.h"
#include "soft_macq_defines.h"
#include "soft_macq_utils.h"
#include "soft_macq_serial.h"
#include "soft_macq_mdbzp.h"

#define SOFTMACQ_MAX_AGE_MS            (500.0)
#define SOFTMACQ_SAMPLE_GRABBER_LENGTH (512*1024)
#define SOFTMACQ_BASEBAND_SIZE         ( 16*1024)

#if SOFTMACQ_SAMPLE_GRABBER_LENGTH > FIXED_GRABBER_LENGTH
#error "SOFTMACQ_SAMPLE_GRABBER_LENGTH shouldn't be greater than FIXED_GRABBER_LENGTH"
#endif


/**! sample grabber leaves RAW F/E samples here */
static uint8_t *puSampleBuf;

/**! samples are down-converted to baseband and decimated here  */
static sc16_t pBaseBand[SOFTMACQ_BASEBAND_SIZE] __attribute__ ((aligned (32)));

/**! here is where the code gets resampled */
/* static int8_t pResampCode[SOFTMACQ_MAX_SPMS] __attribute__ ((aligned (32))); */

/** the last grabber acquisition time tag */
static uint64_t uLastTimeTag;

/** the last acquired signal */
static me_gnss_signal_t sLastMesId;

/** */
static uint32_t iSamplesMs;

static bool bModuleInit;

/********************************
 * STATIC FUNCTION DECLARATIONS
 *********************************/

static bool BbMixAndDecimate(const me_gnss_signal_t _sMeSid);

/* static bool SoftMacqMdbzp (const me_gnss_signal_t _sMeSid, acqResults_t *_sAcqResult); */

static bool SoftMacqSerial(const me_gnss_signal_t _sMeSid,
  float _fCarrFreqMin, float _fCarrFreqMax, acq_result_t *_sAcqResult);


/*********************************
 *      EXPOSED INTERFACES
 ********************************/
float soft_multi_acq_bin_width(void) {
  return (NAP_FRONTEND_RAW_SAMPLE_RATE_Hz/SOFTMACQ_DECFACT_GPSL1CA) / (SOFTMACQ_BASEBAND_SIZE);
}

/** old interface has cf_bin_width */
/** new interface shall be
 *
 * bool soft_multi_acq_search(const me_gnss_signal_t _sMeSid, float _fCarrFreqMin, float _fCarrFreqMax, enum sensitivity _eSense, acq_result_t *_sAcqResult)
 *
 *  */
bool soft_multi_acq_search(
  const me_gnss_signal_t _sMeSid,
  float _fCarrFreqMin,
  float _fCarrFreqMax,
  float cf_bin_width,
  acq_result_t *_psAcqResult)
{
  uint32_t uTag=0, uBuffLength=0;
  uint64_t uCurrTimeTag;
  /*acqResults_t sLocalResult = {0};*/
  /** sanity checking input parameters */
  assert(NULL != _psAcqResult);

  (void) cf_bin_width;

  if (!bModuleInit) {
    InitBBConvLut();
    bModuleInit = true;
    log_info("InitBBConvLut()");
  }

  /** Check if the last grabbed signal snapshot isn't too old.
   * If yes, simply grab another one */
  uCurrTimeTag = nap_timing_count();
  if ((uLastTimeTag == 0) ||
      (nap_count_to_ms(uCurrTimeTag-uLastTimeTag) > SOFTMACQ_MAX_AGE_MS)) {
    /** GRAB!!! */
    puSampleBuf = grab_samples(&uBuffLength, &uTag);
    if (NULL == puSampleBuf) {
      log_warn("data grabber failed, uBuffLength %u uTag %u", uBuffLength, uTag);
      return false;
    }
    /** update signal time tag */
    uLastTimeTag = uTag;
  }
  /** regardless of the result, store here the time tag */
  _psAcqResult->sample_count = (uint32_t) uLastTimeTag;
  /*sLocalResult.uFirstLocIdx = uLastTimeTag;*/

  /** Perform signal conditioning (down-conversion, filtering and decimation):
   * - if we updated the signal snapshot or
   * - if the last searched `me_gnss_signal_t` was not compatible
   *   with the current one
   * - for Glonass, `sat` holds the FCN and we might want to do this again
   *  */
  if ((uTag) ||
      (!code_equiv(sLastMesId.code, _sMeSid.code)) ||
      ((sLastMesId.code == CODE_GLO_L1CA) && (sLastMesId.sat != _sMeSid.sat))) {
    /** perform again baseband down-conversion and decimation depending on _sMeSid */
    BbMixAndDecimate(_sMeSid);
    //~ log_info("uTag %u BbMixAndDecimate()", uTag);
  }
  /** store now last used mesid */
  sLastMesId = _sMeSid;

  /** call DBZP-like acquisition with current sensitivity parameters
   *
   * NOTE: right now this is just to have he compiler going down
   * this route, but eventually could swap serial search */
  /*
  if ((_fCarrFreqMax - _fCarrFreqMin)>100e3) {
   return SoftMacqMdbzp(_sMeSid, &sLocalResult);
  }
  */

  /** call serial-frequency search acquisition with current sensitivity parameters */
  return SoftMacqSerial(_sMeSid, _fCarrFreqMin, _fCarrFreqMax, _psAcqResult);
}


/**********************************
 * STATIC FUNCTION DEFINITIONS
 ***********************************/

/************* Serial frequency search *****************/

static bool SoftMacqSerial(
  const me_gnss_signal_t _sMeSid,
  float _fCarrFreqMin,
  float _fCarrFreqMax,
  acq_result_t *_psLegacyResult)
{
  float cf_bin_width = soft_acq_bin_width();

  return soft_acq_search(pBaseBand,
    _sMeSid, _fCarrFreqMin, _fCarrFreqMax,
    cf_bin_width, _psLegacyResult);

}


/*************    MDBZP *****************/
/*
static bool SoftMacqMdbzp (
  const me_gnss_signal_t _sMeSid,
  acqResults_t *_psAcqResult)
{
  uint32_t h;
  uint32_t uNco, uNcoStep, uChipInd;
  sFauParams_t sParams;
  const uint8_t *_pLocalCode = ca_code(_sMeSid);

  assert(NULL != _psAcqResult);
  assert(mesid_valid(_sMeSid));

  sParams.iSampMs = iSamplesMs;

  switch (_sMeSid.code) {
  case CODE_GPS_L1CA:
    sParams.iNumCodeSlices = SOFTMACQ_GPSL1CA_CODE_SLICES;
    sParams.iCodeTimeMs    = SOFTMACQ_GPSL1CA_CODE_MS;
    sParams.iPostCorrDec   = 1;
    sParams.iCohCodes      = SOFTMACQ_GPSL1CA_COHE;
    sParams.iNcohAcc       = SOFTMACQ_GPSL1CA_NONC;
    sParams.iSecondaryLen  = 0;

    uNcoStep = SOFTMACQ_GPSL1CA_CODE_CHIPS;
    uChipInd = 0;
    for (h=0, uNco=0; h < iSamplesMs; h++) {
      pResampCode[h] = get_chip((uint8_t *)_pLocalCode, uChipInd);
      uNco += uNcoStep;
      if (uNco >= iSamplesMs) {
        uNco -= iSamplesMs;
        uChipInd = (uChipInd+1) % SOFTMACQ_GPSL1CA_CODE_CHIPS;
      }
    }
    break;

  case CODE_GLO_L1CA:
  case CODE_GLO_L2CA:
    sParams.iNumCodeSlices = SOFTMACQ_GLOG1_CODE_SLICES;
    sParams.iCodeTimeMs    = SOFTMACQ_GLOG1_CODE_MS;
    sParams.iPostCorrDec   = 3;
    sParams.iCohCodes      = SOFTMACQ_GLOG1_COHE;
    sParams.iNcohAcc       = SOFTMACQ_GLOG1_NONC;
    sParams.iSecondaryLen  = 0;

    uNcoStep = SOFTMACQ_GLOG1_CODE_CHIPS;
    uChipInd = 0;
    for (h=0, uNco=0; h < iSamplesMs; h++) {
      pResampCode[h] = get_chip((uint8_t *)_pLocalCode, uChipInd);
      uNco += uNcoStep;
      if (uNco >= iSamplesMs) {
        uNco -= iSamplesMs;
        uChipInd = (uChipInd+1) % SOFTMACQ_GLOG1_CODE_CHIPS;
      }
    }
    break;

  case CODE_SBAS_L1CA:
    sParams.iNumCodeSlices = SOFTMACQ_SBASL1_CODE_SLICES;
    sParams.iCodeTimeMs    = SOFTMACQ_SBASL1_CODE_MS;
    sParams.iPostCorrDec   = 1;
    sParams.iCohCodes      = SOFTMACQ_SBASL1_COHE;
    sParams.iNcohAcc       = SOFTMACQ_SBASL1_NONC;
    sParams.iSecondaryLen  = 0;

    uNcoStep = SOFTMACQ_SBASL1_CODE_CHIPS;
    uChipInd = 0;
    for (h=0, uNco=0; h < iSamplesMs; h++) {
      pResampCode[h] = get_chip((uint8_t *)_pLocalCode, uChipInd);
      uNco += uNcoStep;
      if (uNco >= iSamplesMs) {
        uNco -= iSamplesMs;
        uChipInd = (uChipInd+1) % SOFTMACQ_SBASL1_CODE_CHIPS;
      }
    }
    break;

  case CODE_INVALID:
  case CODE_GPS_L2CM:
  case CODE_GPS_L1P:
  case CODE_GPS_L2P:
  case CODE_GPS_L2CL:
  case CODE_COUNT:
  default:
    break;

  }
  return sParams.iSecondaryLen;
  mdbzp_static(pBaseBand, pResampCode, &sParams, _psAcqResult);
}
*/


/*! \fn bool BbMixAndDecimate
 *  \brief
 **/
static bool BbMixAndDecimate(const me_gnss_signal_t _sMeSid) {
  uint32_t k, h, uDecFactor;
  uint32_t uNco, uNcoVal, uNcoStep=0;
  uint8_t uSample;

  /** first of all reset the destination buffer */
  memset(pBaseBand, 0, SOFTMACQ_BASEBAND_SIZE*sizeof(sc16_t));

  switch (_sMeSid.code) {

  case CODE_GPS_L1CA:
  case CODE_SBAS_L1CA:
    uDecFactor = SOFTMACQ_DECFACT_GPSL1CA;
    iSamplesMs = SOFTMACQ_RAW_SPMS / uDecFactor;
    uNcoStep = CirclesToUint32((double) SOFTMACQ_FC_GPSL1 / (double) SOFTMACQ_RAW_FS);
    //~ log_info("BbMixAndDecimate() uDecFactor %u iSamplesMs %d uNcoStep %u", uDecFactor, iSamplesMs, uNcoStep);

    for (k=0, uNco=0; k<SOFTMACQ_SAMPLE_GRABBER_LENGTH; k++) {
      uSample = ((puSampleBuf[k] >> 0) & 0x3) << BBNCO_CARRPH_BITS;   /** two LSBs are Channel 1 */
      uNcoVal = (uNco >> (32-BBNCO_CARRPH_BITS)) & BBNCO_CARRPH_MASK;

      h = k/uDecFactor;
      if (h == SOFTMACQ_BASEBAND_SIZE) break;

      pBaseBand[h].r += bbConvTable[(uSample | uNcoVal)].r;
      pBaseBand[h].i += bbConvTable[(uSample | uNcoVal)].i;
      uNco += uNcoStep;
    }
    break;

  case CODE_GLO_L1CA:
    uDecFactor = SOFTMACQ_DECFACT_GLOG1;
    iSamplesMs = SOFTMACQ_RAW_SPMS / uDecFactor;
    uNcoStep = CirclesToUint32((double) (SOFTMACQ_FC_GLOG1+(_sMeSid.sat-8)*SOFTMACQ_GLOG1_FOFF) / (double) SOFTMACQ_RAW_FS);

    for (k=0, h=0, uNco=0; k<SOFTMACQ_SAMPLE_GRABBER_LENGTH; k++) {
      h = k/uDecFactor;
      uSample = ((puSampleBuf[k] >> 2) & 0x3) << BBNCO_CARRPH_BITS;   /** B3..2 are Channel 2 */
      uNcoVal = (uNco >> (32-BBNCO_CARRPH_BITS)) & BBNCO_CARRPH_MASK;

      pBaseBand[h].r += bbConvTable[(uSample | uNcoVal)].r;
      pBaseBand[h].i += bbConvTable[(uSample | uNcoVal)].i;
      uNco += uNcoStep;
    }
    break;

  case CODE_GLO_L2CA:
    uDecFactor = SOFTMACQ_DECFACT_GLOG2;
    iSamplesMs = SOFTMACQ_RAW_SPMS / uDecFactor;
    uNcoStep = CirclesToUint32((double) (SOFTMACQ_FC_GLOG2+(_sMeSid.sat-8)*SOFTMACQ_GLOG2_FOFF) / (double) SOFTMACQ_RAW_FS);

    for (k=0, h=0, uNco=0; k<SOFTMACQ_SAMPLE_GRABBER_LENGTH; k++) {
      h = k/uDecFactor;
      uSample = ((puSampleBuf[k] >> 4) & 0x3) << BBNCO_CARRPH_BITS;   /** B5..4 are Channel 3 */
      uNcoVal = (uNco >> (32-BBNCO_CARRPH_BITS)) & BBNCO_CARRPH_MASK;

      pBaseBand[h].r += bbConvTable[(uSample | uNcoVal)].r;
      pBaseBand[h].i += bbConvTable[(uSample | uNcoVal)].i;
      uNco += uNcoStep;
    }
    break;

  case CODE_GPS_L2CM:
  case CODE_GPS_L2CL:
    uDecFactor = SOFTMACQ_DECFACT_GPSL2;
    iSamplesMs = SOFTMACQ_RAW_SPMS / uDecFactor;
    uNcoStep = CirclesToUint32((double) SOFTMACQ_FC_GPSL2 / (double) SOFTMACQ_RAW_FS);

    for (k=0, h=0, uNco=0; k<SOFTMACQ_SAMPLE_GRABBER_LENGTH; k++) {
      h = k/uDecFactor;
      uSample = ((puSampleBuf[k] >> 6) & 0x3) << BBNCO_CARRPH_BITS;   /** B7..6 are Channel 4 */
      uNcoVal = (uNco >> (32-BBNCO_CARRPH_BITS)) & BBNCO_CARRPH_MASK;

      pBaseBand[h].r += bbConvTable[(uSample | uNcoVal)].r;
      pBaseBand[h].i += bbConvTable[(uSample | uNcoVal)].i;
      uNco += uNcoStep;
    }
    break;

  case CODE_INVALID:
  case CODE_GPS_L1P:
  case CODE_GPS_L2P:
  case CODE_COUNT:
  default:
    return false;
    break;
  }

  return true;

}
