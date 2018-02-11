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

#include "board/v3/nap/grabber.h"
#include "nap/nap_common.h"
#include "nap/nap_constants.h"

#include "lib/fixed_fft_r2.h"
#include "soft_macq_defines.h"
#include "soft_macq_mdbzp.h"
#include "soft_macq_serial.h"
#include "soft_macq_utils.h"

#define FAU_MAX_AGE_S (0.5)
#define FAU_MAX_AGE_SAMP (FAU_MAX_AGE_S * NAP_TRACK_SAMPLE_RATE_Hz)

#define FAU_SAMPLE_GRABBER_LENGTH (512 * 1024)
#define FAU_BASEBAND_SIZE (16 * 1024)

#if FAU_SAMPLE_GRABBER_LENGTH > FIXED_GRABBER_LENGTH
#error \
    "FAU_SAMPLE_GRABBER_LENGTH shouldn't be greater than FIXED_GRABBER_LENGTH"
#endif

/**! sample grabber leaves RAW F/E samples here */
static u8 *sample_buff;

/**! samples are down-converted to baseband and decimated here  */
static sc16_t pBaseBand[FAU_BASEBAND_SIZE] __attribute__((aligned(32)));

/**! here is where the code gets resampled */
static s8 pResampCode[FAU_SPMS] __attribute__((aligned(32)));

/** the last grabber acquisition time tag */
static u64 last_timetag;

/** the last acquired signal */
static me_gnss_signal_t mesid_last;

/** */
static u32 samples_ms;

static bool bModuleInit;

/********************************
 * STATIC FUNCTION DECLARATIONS
 *********************************/

static bool BbMixAndDecimate(const me_gnss_signal_t mesid);

static bool SoftMacqSerial(const me_gnss_signal_t mesid,
                           float _fCarrFreqMin,
                           float _fCarrFreqMax,
                           acq_result_t *_sAcqResult);

static bool SoftMacqMdbzp(const me_gnss_signal_t mesid,
                          acqResults_t *_sAcqResult);

/*********************************
 *      EXPOSED INTERFACES
 ********************************/
float soft_multi_acq_bin_width(void) {
  return (NAP_FRONTEND_RAW_SAMPLE_RATE_Hz / FAU_DECFACT) / (FAU_BASEBAND_SIZE);
}

/** old interface has cf_bin_width */
/** new interface shall be
 *
 * bool soft_multi_acq_search(const me_gnss_signal_t mesid, float
 * _fCarrFreqMin, float _fCarrFreqMax, enum sensitivity _eSense, acq_result_t
 * *_sAcqResult)
 *
 *  */
bool soft_multi_acq_search(const me_gnss_signal_t mesid,
                           float _fCarrFreqMin,
                           float _fCarrFreqMax,
                           acq_result_t *p_acqres) {
  u64 tmp_timetag = 0;
  u32 buff_size = 0;
  acqResults_t sLocalResult = {0};

  /** sanity checking input parameters */
  assert(NULL != p_acqres);

  memset(p_acqres, 0, sizeof(acq_result_t));

  if (!bModuleInit) {
    InitBBConvLut();
    bModuleInit = true;
    log_debug("InitBBConvLut()");
  }

  /** Check if the last grabbed signal snapshot isn't too old.
   * If yes, simply grab another one */
  u32 curr_timetag = NAP->TIMING_COUNT;
  if ((last_timetag == 0) ||
      ((curr_timetag - last_timetag) > FAU_MAX_AGE_SAMP)) {
    /** GRAB!!! */
    sample_buff = grab_samples(&buff_size, &tmp_timetag);
    if (NULL == sample_buff) {
      log_error("grabber failed, buff_size %" PRIu32 " tmp_timetag %" PRIu64,
                buff_size,
                tmp_timetag);
      assert(0);
      return false;
    }
    /** update signal time tag */
    last_timetag = tmp_timetag;
  }
  /** regardless of the result, store here the time tag */
  p_acqres->sample_count = last_timetag;
  sLocalResult.uFirstLocIdx = last_timetag;

  /** Perform signal conditioning (down-conversion, filtering and decimation):
   * - if we updated the signal snapshot or
   * - if the last searched `me_gnss_signal_t` was not compatible
   *   with the current one
   * - for Glonass, `sat` holds the FCN and we might want to do this again
   *  */
  if ((tmp_timetag) || (!code_equiv(mesid_last.code, mesid.code)) ||
      ((mesid_last.code == CODE_GLO_L1OF) && (mesid_last.sat != mesid.sat))) {
    /** perform again baseband down-conversion and decimation depending on
     * mesid */
    BbMixAndDecimate(mesid);
  }

  /** store now last used mesid */
  mesid_last = mesid;

  /** call DBZP-like acquisition with current sensitivity parameters
   *
   * NOTE: right now this is just to have he compiler going down
   * this route, but eventually could swap serial search */
  if ((_fCarrFreqMax - _fCarrFreqMin) > 5000) {
    bool ret = SoftMacqMdbzp(mesid, &sLocalResult);
    p_acqres->cp =
        (1.0f - sLocalResult.fCodeDelay) * code_to_chip_count(mesid.code);
    p_acqres->cf = sLocalResult.fDoppFreq;
    p_acqres->cn0 = ACQ_EARLY_THRESHOLD;
    return ret;
  }

  /** call serial-frequency search acquisition with current sensitivity
   * parameters */
  return SoftMacqSerial(mesid, _fCarrFreqMin, _fCarrFreqMax, p_acqres);
}

/**********************************
 * STATIC FUNCTION DEFINITIONS
 ***********************************/

/************* Serial frequency search *****************/

static bool SoftMacqSerial(const me_gnss_signal_t mesid,
                           float _fCarrFreqMin,
                           float _fCarrFreqMax,
                           acq_result_t *_psLegacyResult) {
  float cf_bin_width = soft_acq_bin_width();

  return soft_acq_search(pBaseBand,
                         mesid,
                         _fCarrFreqMin,
                         _fCarrFreqMax,
                         cf_bin_width,
                         _psLegacyResult);
}

/*! \fn bool BbMixAndDecimate
 *  \brief
 **/
static bool BbMixAndDecimate(const me_gnss_signal_t mesid) {
  u32 k, h;
  u32 uNco, uNcoVal, uNcoStep = 0;
  u8 uSample;

  /** first of all reset the destination buffer */
  memset(pBaseBand, 0, FAU_BASEBAND_SIZE * sizeof(sc16_t));

  switch (mesid.code) {
    case CODE_GPS_L1CA:
    case CODE_SBAS_L1CA:
    case CODE_QZS_L1CA:
      samples_ms = FAU_RAW_SPMS / FAU_DECFACT;
      uNcoStep = CirclesToUint32((double)FAU_FC_GPSL1 / (double)FAU_RAW_FS);

      for (k = 0, uNco = 0; k < FAU_SAMPLE_GRABBER_LENGTH; k++) {
        uSample = ((sample_buff[k] >> 0) & 0x3)
                  << BBNCO_CARRPH_BITS; /** two LSBs are Channel 1 */
        uNcoVal = (uNco >> (32 - BBNCO_CARRPH_BITS)) & BBNCO_CARRPH_MASK;

        h = k / FAU_DECFACT;
        if (FAU_BASEBAND_SIZE == h) break;

        pBaseBand[h].r += bbConvTable[(uSample | uNcoVal)].r;
        pBaseBand[h].i += bbConvTable[(uSample | uNcoVal)].i;
        uNco += uNcoStep;
      }
      break;

    case CODE_GLO_L1OF:
      samples_ms = FAU_RAW_SPMS / FAU_DECFACT;
      uNcoStep = CirclesToUint32(
          (double)(FAU_FC_GLOG1 +
                   (mesid.sat - GLO_FCN_OFFSET) * FAU_GLOG1_FOFF) /
          (double)FAU_RAW_FS);

      for (k = 0, h = 0, uNco = 0; k < FAU_SAMPLE_GRABBER_LENGTH; k++) {
        uSample = ((sample_buff[k] >> 2) & 0x3)
                  << BBNCO_CARRPH_BITS; /** B3..2 are Channel 2 */
        uNcoVal = (uNco >> (32 - BBNCO_CARRPH_BITS)) & BBNCO_CARRPH_MASK;

        h = k / FAU_DECFACT;
        if (FAU_BASEBAND_SIZE == h) break;

        pBaseBand[h].r += bbConvTable[(uSample | uNcoVal)].r;
        pBaseBand[h].i += bbConvTable[(uSample | uNcoVal)].i;
        uNco += uNcoStep;
      }
      break;

    case CODE_BDS2_B11:
      samples_ms = FAU_RAW_SPMS / FAU_DECFACT;
      uNcoStep = CirclesToUint32((double)(FAU_FC_BDSB1) / (double)FAU_RAW_FS);

      for (k = 0, h = 0, uNco = 0; k < FAU_SAMPLE_GRABBER_LENGTH; k++) {
        uSample = ((sample_buff[k] >> 0) & 0x3)
                  << BBNCO_CARRPH_BITS; /** B1..0 are Channel 1 */
        uNcoVal = (uNco >> (32 - BBNCO_CARRPH_BITS)) & BBNCO_CARRPH_MASK;

        h = k / FAU_DECFACT;
        if (FAU_BASEBAND_SIZE == h) break;

        pBaseBand[h].r += bbConvTable[(uSample | uNcoVal)].r;
        pBaseBand[h].i += bbConvTable[(uSample | uNcoVal)].i;
        uNco += uNcoStep;
      }
      break;

    case CODE_INVALID:
    case CODE_GLO_L2OF:
    case CODE_GPS_L2CM:
    case CODE_GPS_L2CL:
    case CODE_GPS_L1P:
    case CODE_GPS_L2P:
    case CODE_GPS_L2CX:
    case CODE_GPS_L5I:
    case CODE_GPS_L5Q:
    case CODE_GPS_L5X:
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
    case CODE_QZS_L2CM:
    case CODE_QZS_L2CL:
    case CODE_QZS_L2CX:
    case CODE_QZS_L5I:
    case CODE_QZS_L5Q:
    case CODE_QZS_L5X:
    case CODE_COUNT:
    default:
      return false;
      break;
  }

  return true;
}

/** Prepares for Modified Double Block Zero Padding
 *
 * \param mesid MESID of the acquisition
 * \param pacq_res Acquisition results.
 */
static bool SoftMacqMdbzp(const me_gnss_signal_t mesid,
                          acqResults_t *pacq_res) {
  u32 h;
  u32 uNco, uNcoStep, uChipInd;
  sFauParams_t sParams;
  const u8 *_pLocalCode = ca_code(mesid);

  /** sanity checks */
  assert(NULL != pacq_res);
  assert(mesid_valid(mesid));

  sParams.iSampMs = samples_ms;

  switch (mesid.code) {
    case CODE_GPS_L1CA:
    case CODE_QZS_L1CA:
      sParams.iNumCodeSlices = FAU_MDBZP_MS_SLICES * FAU_GPSL1CA_CODE_MS;
      sParams.iCodeTimeMs = FAU_GPSL1CA_CODE_MS;
      sParams.iCohCodes = FAU_GPSL1CA_COHE;
      sParams.iNcohAcc = FAU_GPSL1CA_NONC;
      sParams.uSecCodeLen = 0;

      uNcoStep = FAU_GPSL1CA_CODE_CHIPS;
      uChipInd = 0;
      for (h = 0, uNco = 0; h < samples_ms; h++) {
        pResampCode[h] = get_chip((u8 *)_pLocalCode, uChipInd);
        uNco += uNcoStep;
        if (uNco >= samples_ms) {
          uNco -= samples_ms;
          uChipInd = (uChipInd + 1) % FAU_GPSL1CA_CODE_CHIPS;
        }
      }
      break;

    case CODE_GLO_L1OF:
      sParams.iNumCodeSlices = FAU_MDBZP_MS_SLICES * FAU_GLOG1_CODE_MS;
      sParams.iCodeTimeMs = FAU_GLOG1_CODE_MS;
      sParams.iCohCodes = FAU_GLOG1_COHE;
      sParams.iNcohAcc = FAU_GLOG1_NONC;
      sParams.uSecCodeLen = 0;

      uNcoStep = FAU_GLOG1_CODE_CHIPS;
      uChipInd = 0;
      for (h = 0, uNco = 0; h < samples_ms; h++) {
        pResampCode[h] = get_chip((u8 *)_pLocalCode, uChipInd);
        uNco += uNcoStep;
        if (uNco >= samples_ms) {
          uNco -= samples_ms;
          uChipInd = (uChipInd + 1) % FAU_GLOG1_CODE_CHIPS;
        }
      }
      break;

    case CODE_SBAS_L1CA:
      sParams.iNumCodeSlices = FAU_MDBZP_MS_SLICES * FAU_SBASL1_CODE_MS;
      sParams.iCodeTimeMs = FAU_SBASL1_CODE_MS;
      sParams.iCohCodes = FAU_SBASL1_COHE;
      sParams.iNcohAcc = FAU_SBASL1_NONC;
      sParams.uSecCodeLen = 0;

      uNcoStep = FAU_SBASL1_CODE_CHIPS;
      uChipInd = 0;
      for (h = 0, uNco = 0; h < samples_ms; h++) {
        pResampCode[h] = get_chip((u8 *)_pLocalCode, uChipInd);
        uNco += uNcoStep;
        if (uNco >= samples_ms) {
          uNco -= samples_ms;
          uChipInd = (uChipInd + 1) % FAU_SBASL1_CODE_CHIPS;
        }
      }
      break;

    case CODE_BDS2_B11:
      sParams.iNumCodeSlices = FAU_MDBZP_MS_SLICES * FAU_BDSB11_CODE_MS;
      sParams.iCodeTimeMs = FAU_BDSB11_CODE_MS;
      sParams.iCohCodes = FAU_BDSB11_COHE;
      sParams.iNcohAcc = FAU_BDSB11_NONC;
      sParams.uSecCodeLen = 0;

      uNcoStep = FAU_BDSB11_CODE_CHIPS;
      uChipInd = 0;
      for (h = 0, uNco = 0; h < samples_ms; h++) {
        pResampCode[h] = get_chip((u8 *)_pLocalCode, uChipInd);
        uNco += uNcoStep;
        if (uNco >= samples_ms) {
          uNco -= samples_ms;
          uChipInd = (uChipInd + 1) % FAU_BDSB11_CODE_CHIPS;
        }
      }
      break;

    case CODE_INVALID:
    case CODE_GLO_L2OF:
    case CODE_GPS_L2CM:
    case CODE_GPS_L2CL:
    case CODE_GPS_L1P:
    case CODE_GPS_L2P:
    case CODE_GPS_L2CX:
    case CODE_GPS_L5I:
    case CODE_GPS_L5Q:
    case CODE_GPS_L5X:
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
    case CODE_QZS_L2CM:
    case CODE_QZS_L2CL:
    case CODE_QZS_L2CX:
    case CODE_QZS_L5I:
    case CODE_QZS_L5Q:
    case CODE_QZS_L5X:
    case CODE_COUNT:
    default:
      return false;
      break;
  }
  /** call now MDBZP */
  bool ret = mdbzp_static(pBaseBand, pResampCode, &sParams, pacq_res);
  if (ret) {
    log_debug_mesid(mesid,
                    "%16" PRIu64
                    "  fMaxCorr %.1e  fDoppFreq %.1f  fCodeDelay %.4f",
                    pacq_res->uFirstLocIdx,
                    pacq_res->fMaxCorr,
                    pacq_res->fDoppFreq,
                    pacq_res->fCodeDelay);
    /* ret = false; */
  }
  return ret;
}
