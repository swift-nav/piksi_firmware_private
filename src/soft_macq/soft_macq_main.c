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
#include <ch.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include <libswiftnav/logging.h>
#include <libswiftnav/prns.h>

#include "board/v3/nap/grabber.h"
#include "nap/nap_common.h"
#include "nap/nap_constants.h"

#include "lib/fixed_fft_r2.h"
#include "soft_macq_defines.h"
#include "soft_macq_serial.h"
#include "soft_macq_utils.h"

#define SOFTMACQ_MAX_AGE_MS (500.0)
#define SOFTMACQ_SAMPLE_GRABBER_LENGTH (512 * 1024)
#define SOFTMACQ_BASEBAND_SIZE (16 * 1024)

#if SOFTMACQ_SAMPLE_GRABBER_LENGTH > FIXED_GRABBER_LENGTH
#error \
    "SOFTMACQ_SAMPLE_GRABBER_LENGTH shouldn't be greater than FIXED_GRABBER_LENGTH"
#endif

/**! sample grabber leaves RAW F/E samples here */
static uint8_t *puSampleBuf;

/**! samples are down-converted to baseband and decimated here  */
static sc16_t pBaseBand[SOFTMACQ_BASEBAND_SIZE] __attribute__((aligned(32)));

/** the last grabber acquisition time tag */
static u32 uLastTimeTag;

/** the last acquired signal */
static me_gnss_signal_t sLastMesId;

/** */
static uint32_t iSamplesMs;

static bool bModuleInit;

/********************************
 * STATIC FUNCTION DECLARATIONS
 *********************************/

static bool BbMixAndDecimate(const me_gnss_signal_t _sMeSid);

static bool SoftMacqSerial(const me_gnss_signal_t _sMeSid,
                           float _fCarrFreqMin,
                           float _fCarrFreqMax,
                           acq_result_t *_sAcqResult);

/*********************************
 *      EXPOSED INTERFACES
 ********************************/
float soft_multi_acq_bin_width(void) {
  return (NAP_FRONTEND_RAW_SAMPLE_RATE_Hz / SOFTMACQ_DECFACT_GPSL1CA) /
         (SOFTMACQ_BASEBAND_SIZE);
}

/** old interface has cf_bin_width */
/** new interface shall be
 *
 * bool soft_multi_acq_search(const me_gnss_signal_t _sMeSid, float
 * _fCarrFreqMin, float _fCarrFreqMax, enum sensitivity _eSense, acq_result_t
 * *_sAcqResult)
 *
 *  */
bool soft_multi_acq_search(const me_gnss_signal_t _sMeSid,
                           float _fCarrFreqMin,
                           float _fCarrFreqMax,
                           acq_result_t *_psAcqResult) {
  u32 uTag = 0, uBuffLength = 0;
  u32 uCurrTimeTag;
  /** sanity checking input parameters */
  assert(NULL != _psAcqResult);

  if (!bModuleInit) {
    InitBBConvLut();
    bModuleInit = true;
    log_info("InitBBConvLut()");
  }

  /** Check if the last grabbed signal snapshot isn't too old.
   * If yes, simply grab another one */
  uCurrTimeTag = NAP->TIMING_COUNT;
  if ((uLastTimeTag == 0) ||
      ((0.001*(uCurrTimeTag - uLastTimeTag)/NAP_FRONTEND_SAMPLE_RATE_Hz) > SOFTMACQ_MAX_AGE_MS)) {
    /** GRAB!!! */
    puSampleBuf = grab_samples(&uBuffLength, &uTag);
    if (NULL == puSampleBuf) {
      log_warn(
          "data grabber failed, uBuffLength %u uTag %u", uBuffLength, uTag);
      return false;
    }
    /** update signal time tag */
    uLastTimeTag = uTag;
  }
  /** regardless of the result, store here the time tag */
  _psAcqResult->sample_count = uLastTimeTag;

  /** Perform signal conditioning (down-conversion, filtering and decimation):
   * - if we updated the signal snapshot or
   * - if the last searched `me_gnss_signal_t` was not compatible
   *   with the current one
   * - for Glonass, `sat` holds the FCN and we might want to do this again
   *  */
  if ((uTag) || (!code_equiv(sLastMesId.code, _sMeSid.code)) ||
      ((sLastMesId.code == CODE_GLO_L1CA) && (sLastMesId.sat != _sMeSid.sat))) {
    /** perform again baseband down-conversion and decimation depending on
     * _sMeSid */
    BbMixAndDecimate(_sMeSid);
  }
  /** store now last used mesid */
  sLastMesId = _sMeSid;

  /** call DBZP-like acquisition with current sensitivity parameters
   *
   * NOTE: right now this is just to have he compiler going down
   * this route, but eventually could swap serial search */

  /** call serial-frequency search acquisition with current sensitivity
   * parameters */
  return SoftMacqSerial(_sMeSid, _fCarrFreqMin, _fCarrFreqMax, _psAcqResult);
}

/**********************************
 * STATIC FUNCTION DEFINITIONS
 ***********************************/

/************* Serial frequency search *****************/

static bool SoftMacqSerial(const me_gnss_signal_t _sMeSid,
                           float _fCarrFreqMin,
                           float _fCarrFreqMax,
                           acq_result_t *_psLegacyResult) {
  float cf_bin_width = soft_acq_bin_width();

  return soft_acq_search(pBaseBand,
                         _sMeSid,
                         _fCarrFreqMin,
                         _fCarrFreqMax,
                         cf_bin_width,
                         _psLegacyResult);
}

/*! \fn bool BbMixAndDecimate
 *  \brief
 **/
static bool BbMixAndDecimate(const me_gnss_signal_t _sMeSid) {
  uint32_t k, h, uDecFactor;
  uint32_t uNco, uNcoVal, uNcoStep = 0;
  uint8_t uSample;

  /** first of all reset the destination buffer */
  memset(pBaseBand, 0, SOFTMACQ_BASEBAND_SIZE * sizeof(sc16_t));

  switch (_sMeSid.code) {
    case CODE_GPS_L1CA:
    case CODE_SBAS_L1CA:
      uDecFactor = SOFTMACQ_DECFACT_GPSL1CA;
      iSamplesMs = SOFTMACQ_RAW_SPMS / uDecFactor;
      uNcoStep =
          CirclesToUint32((double)SOFTMACQ_FC_GPSL1 / (double)SOFTMACQ_RAW_FS);

      for (k = 0, uNco = 0; k < SOFTMACQ_SAMPLE_GRABBER_LENGTH; k++) {
        uSample = ((puSampleBuf[k] >> 0) & 0x3)
                  << BBNCO_CARRPH_BITS; /** two LSBs are Channel 1 */
        uNcoVal = (uNco >> (32 - BBNCO_CARRPH_BITS)) & BBNCO_CARRPH_MASK;

        h = k / uDecFactor;
        if (h == SOFTMACQ_BASEBAND_SIZE) break;

        pBaseBand[h].r += bbConvTable[(uSample | uNcoVal)].r;
        pBaseBand[h].i += bbConvTable[(uSample | uNcoVal)].i;
        uNco += uNcoStep;
      }
      break;

    case CODE_GLO_L1CA:
      uDecFactor = SOFTMACQ_DECFACT_GLOG1;
      iSamplesMs = SOFTMACQ_RAW_SPMS / uDecFactor;
      uNcoStep = CirclesToUint32(
          (double)(SOFTMACQ_FC_GLOG1 +
                   (_sMeSid.sat - GLO_FCN_OFFSET) * SOFTMACQ_GLOG1_FOFF) /
          (double)SOFTMACQ_RAW_FS);

      for (k = 0, h = 0, uNco = 0; k < SOFTMACQ_SAMPLE_GRABBER_LENGTH; k++) {
        uSample = ((puSampleBuf[k] >> 2) & 0x3)
                  << BBNCO_CARRPH_BITS; /** B3..2 are Channel 2 */
        uNcoVal = (uNco >> (32 - BBNCO_CARRPH_BITS)) & BBNCO_CARRPH_MASK;

        h = k / uDecFactor;
        if (h == SOFTMACQ_BASEBAND_SIZE) break;

        pBaseBand[h].r += bbConvTable[(uSample | uNcoVal)].r;
        pBaseBand[h].i += bbConvTable[(uSample | uNcoVal)].i;
        uNco += uNcoStep;
      }
      break;

    case CODE_GLO_L2CA:
    case CODE_GPS_L2CM:
    case CODE_GPS_L2CL:
    case CODE_INVALID:
    case CODE_GPS_L1P:
    case CODE_GPS_L2P:
    case CODE_COUNT:
    case CODE_GPS_L2CX:
    case CODE_GPS_L5I:
    case CODE_GPS_L5Q:
    case CODE_GPS_L5X:
    case CODE_BDS2_B11:
    case CODE_BDS2_B2:
    case CODE_GAL_E1B:
    case CODE_GAL_E1C:
    case CODE_GAL_E1X:
    case CODE_GAL_E6B:
    case CODE_GAL_E6C:
    case CODE_GAL_E6X:
    case CODE_GAL_E7I:
    case CODE_GAL_E7Q:
    case CODE_GAL_E7X:
    case CODE_GAL_E8:
    case CODE_GAL_E5I:
    case CODE_GAL_E5Q:
    case CODE_GAL_E5X:
    case CODE_QZS_L1CA:
    case CODE_QZS_L2CM:
    case CODE_QZS_L2CL:
    case CODE_QZS_L2CX:
    case CODE_QZS_L5I:
    case CODE_QZS_L5Q:
    case CODE_QZS_L5X:
    default:
      return false;
      break;
  }

  return true;
}
