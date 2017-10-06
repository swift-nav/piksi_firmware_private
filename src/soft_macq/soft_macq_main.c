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

#define SOFTMACQ_MAX_AGE_S (0.5)
#define SOFTMACQ_MAX_AGE_SAMP (SOFTMACQ_MAX_AGE_S * NAP_FRONTEND_SAMPLE_RATE_Hz)

#define SOFTMACQ_SAMPLE_GRABBER_LENGTH (512 * 1024)
#define SOFTMACQ_BASEBAND_SIZE (16 * 1024)

#if SOFTMACQ_SAMPLE_GRABBER_LENGTH > FIXED_GRABBER_LENGTH
#error \
    "SOFTMACQ_SAMPLE_GRABBER_LENGTH shouldn't be greater than FIXED_GRABBER_LENGTH"
#endif

/**! sample grabber leaves RAW F/E samples here */
static u8 *sample_buff;

/**! samples are down-converted to baseband and decimated here  */
static sc16_t pBaseBand[SOFTMACQ_BASEBAND_SIZE] __attribute__((aligned(32)));

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

/*********************************
 *      EXPOSED INTERFACES
 ********************************/
float soft_multi_acq_bin_width(void) {
  return (NAP_FRONTEND_SAMPLE_RATE_Hz / SOFTMACQ_DECFACT_GPSL1CA) /
         (SOFTMACQ_BASEBAND_SIZE);
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
  /** sanity checking input parameters */
  assert(NULL != p_acqres);

  memset(p_acqres, 0, sizeof(acq_result_t));

  if (!bModuleInit) {
    InitBBConvLut();
    bModuleInit = true;
    log_info("InitBBConvLut()");
  }

  /** Check if the last grabbed signal snapshot isn't too old.
   * If yes, simply grab another one */
  u32 curr_timetag = NAP->TIMING_COUNT;
  if ((last_timetag == 0) ||
      ((curr_timetag - last_timetag) > SOFTMACQ_MAX_AGE_SAMP)) {
    /** GRAB!!! */
    sample_buff = grab_samples(&buff_size, &tmp_timetag);
    if (NULL == sample_buff) {
      log_warn("grabber failed, buff_size %" PRIu32 " tmp_timetag %" PRIu64,
               buff_size,
               tmp_timetag);
      return false;
    }
    /** update signal time tag */
    last_timetag = tmp_timetag;
  }
  /** regardless of the result, store here the time tag */
  p_acqres->sample_count = last_timetag;

  /** Perform signal conditioning (down-conversion, filtering and decimation):
   * - if we updated the signal snapshot or
   * - if the last searched `me_gnss_signal_t` was not compatible
   *   with the current one
   * - for Glonass, `sat` holds the FCN and we might want to do this again
   *  */
  if ((tmp_timetag) || (!code_equiv(mesid_last.code, mesid.code)) ||
      ((mesid_last.code == CODE_GLO_L1CA) && (mesid_last.sat != mesid.sat))) {
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
  u32 k, h, uDecFactor;
  u32 uNco, uNcoVal, uNcoStep = 0;
  u8 uSample;

  /** first of all reset the destination buffer */
  memset(pBaseBand, 0, SOFTMACQ_BASEBAND_SIZE * sizeof(sc16_t));

  switch (mesid.code) {
    case CODE_GPS_L1CA:
    case CODE_SBAS_L1CA:
      uDecFactor = SOFTMACQ_DECFACT_GPSL1CA;
      samples_ms = SOFTMACQ_RAW_SPMS / uDecFactor;
      uNcoStep =
          CirclesToUint32((double)SOFTMACQ_FC_GPSL1 / (double)SOFTMACQ_RAW_FS);

      for (k = 0, uNco = 0; k < SOFTMACQ_SAMPLE_GRABBER_LENGTH; k++) {
        uSample = ((sample_buff[k] >> 0) & 0x3)
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
      samples_ms = SOFTMACQ_RAW_SPMS / uDecFactor;
      uNcoStep = CirclesToUint32(
          (double)(SOFTMACQ_FC_GLOG1 +
                   (mesid.sat - GLO_FCN_OFFSET) * SOFTMACQ_GLOG1_FOFF) /
          (double)SOFTMACQ_RAW_FS);

      for (k = 0, h = 0, uNco = 0; k < SOFTMACQ_SAMPLE_GRABBER_LENGTH; k++) {
        uSample = ((sample_buff[k] >> 2) & 0x3)
                  << BBNCO_CARRPH_BITS; /** B3..2 are Channel 2 */
        uNcoVal = (uNco >> (32 - BBNCO_CARRPH_BITS)) & BBNCO_CARRPH_MASK;

        h = k / uDecFactor;
        if (h == SOFTMACQ_BASEBAND_SIZE) break;

        pBaseBand[h].r += bbConvTable[(uSample | uNcoVal)].r;
        pBaseBand[h].i += bbConvTable[(uSample | uNcoVal)].i;
        uNco += uNcoStep;
      }
      break;

    case CODE_BDS2_B11:
      uDecFactor = SOFTMACQ_DECFACT_BDS2B1;
      samples_ms = SOFTMACQ_RAW_SPMS / uDecFactor;
      uNcoStep = CirclesToUint32((double)(SOFTMACQ_FC_BDS2B1) /
                                 (double)SOFTMACQ_RAW_FS);

      for (k = 0, h = 0, uNco = 0; k < SOFTMACQ_SAMPLE_GRABBER_LENGTH; k++) {
        uSample = ((sample_buff[k] >> 0) & 0x3)
                  << BBNCO_CARRPH_BITS; /** B1..0 are Channel 1 */
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
