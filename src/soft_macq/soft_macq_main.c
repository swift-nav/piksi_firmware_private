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
#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include <swiftnav/linear_algebra.h>
#include <swiftnav/logging.h>

#include "board/v3/nap/grabber.h"
#include "nap/nap_common.h"
#include "nap/nap_constants.h"

#include "lib/fixed_fft_r2.h"
#include "prns.h"
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
static sc16_t baseband[FAU_BASEBAND_SIZE] __attribute__((aligned(32)));

/**! here is where the code gets resampled */
static s8 code_upsamp[FAU_BASEBAND_SIZE] __attribute__((aligned(32)));

/** the last grabber acquisition time tag */
static u64 last_timetag;

/** the last acquired signal */
static me_gnss_signal_t mesid_last;

static bool bModuleInit;

/********************************
 * STATIC FUNCTION DECLARATIONS
 *********************************/
static bool BbMixAndDecimate(const me_gnss_signal_t mesid);

static bool SoftMacqSerial(const me_gnss_signal_t mesid,
                           float doppler_min_hz,
                           float doppler_max_hz,
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
 * doppler_min_hz, float doppler_max_hz, enum sensitivity _eSense, acq_result_t
 * *_sAcqResult)
 *
 *  */
bool soft_multi_acq_search(const me_gnss_signal_t mesid,
                           float doppler_min_hz,
                           float doppler_max_hz,
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
   * - if the carrier frequency of the last searched `sid` does not match the
   * current one
   *  */
  if ((tmp_timetag) || !double_approx_eq(mesid_to_carr_freq(mesid_last),
                                         mesid_to_carr_freq(mesid))) {
    /** perform baseband down-conversion + decimation depending on mesid */
    BbMixAndDecimate(mesid);
  }

  /** store now last used mesid */
  mesid_last = mesid;

  /** call DBZP-like acquisition with current sensitivity parameters
   *
   * NOTE1: right now this is just to have he compiler going down
   * this route, but eventually could swap serial search
   *
   * NOTE2: the serial acquisition really does not work well with
   * BDS and Galileo because of the nav data transitions,
   * DBZP is slower in this case, but should give more chances of successful
   * acquisition
   * */
  if (((doppler_max_hz - doppler_min_hz) > 5000)) {
    bool ret = SoftMacqMdbzp(mesid, &sLocalResult);
    p_acqres->cp =
        (1.0f - sLocalResult.fCodeDelay) * code_to_chip_count(mesid.code);
    p_acqres->df_hz = sLocalResult.fDoppFreq;
    p_acqres->cn0 = ret ? ACQ_EARLY_THRESHOLD : sLocalResult.fMaxCorr;
    return ret;
  }

  /** call serial-frequency search acquisition with current sensitivity
   * parameters */
  return SoftMacqSerial(mesid, doppler_min_hz, doppler_max_hz, p_acqres);
}

/**********************************
 * STATIC FUNCTION DEFINITIONS
 ***********************************/

/************* Serial frequency search *****************/

static bool SoftMacqSerial(const me_gnss_signal_t mesid,
                           float doppler_min_hz,
                           float doppler_max_hz,
                           acq_result_t *_psLegacyResult) {
  float df_bin_width_hz = soft_acq_bin_width();

  return soft_acq_search(baseband,
                         mesid,
                         doppler_min_hz,
                         doppler_max_hz,
                         df_bin_width_hz,
                         _psLegacyResult);
}

/*! \fn bool BbMixAndDecimate
 *  \brief
 **/
static bool BbMixAndDecimate(const me_gnss_signal_t mesid) {
  u32 k, h;
  u32 carr_nco, nco_step = 0;
  u8 sample_bitmap, phase_bitmap;

  /** first of all reset the destination buffer */
  memset(baseband, 0, FAU_BASEBAND_SIZE * sizeof(sc16_t));

  switch ((s8)mesid.code) {
    case CODE_GPS_L1CA:
    case CODE_SBAS_L1CA:
    case CODE_QZS_L1CA:
    case CODE_GAL_E1B:
    case CODE_GAL_E1C:
    case CODE_GAL_E1X:
      nco_step = CirclesToUint32(GPS_L1_HZ / (double)FAU_RAW_FS);

      for (k = 0, carr_nco = 0; k < FAU_SAMPLE_GRABBER_LENGTH; k++) {
        /** B1-B0 are Channel 1 */
        sample_bitmap = ((sample_buff[k] >> 2) & 0x3) << BBNCO_CARRPH_BITS;
        phase_bitmap =
            (carr_nco >> (32 - BBNCO_CARRPH_BITS)) & BBNCO_CARRPH_MASK;

        h = k / FAU_DECFACT;
        if (FAU_BASEBAND_SIZE == h) break;

        baseband[h].r += bbConvTable[(sample_bitmap | phase_bitmap)].r;
        baseband[h].i += bbConvTable[(sample_bitmap | phase_bitmap)].i;
        carr_nco += nco_step;
      }
      break;

    case CODE_GLO_L1OF:
      nco_step = CirclesToUint32(
          (GLO_L1_HZ + (mesid.sat - GLO_FCN_OFFSET) * GLO_L1_DELTA_HZ) /
          (double)FAU_RAW_FS);

      for (k = 0, carr_nco = 0; k < FAU_SAMPLE_GRABBER_LENGTH; k++) {
        /** B3-B2 are Channel 2 */
        sample_bitmap = ((sample_buff[k] >> 0) & 0x3) << BBNCO_CARRPH_BITS;
        phase_bitmap =
            (carr_nco >> (32 - BBNCO_CARRPH_BITS)) & BBNCO_CARRPH_MASK;

        h = k / FAU_DECFACT;
        if (FAU_BASEBAND_SIZE == h) break;

        baseband[h].r += bbConvTable[(sample_bitmap | phase_bitmap)].r;
        baseband[h].i += bbConvTable[(sample_bitmap | phase_bitmap)].i;
        carr_nco += nco_step;
      }
      break;

    case CODE_BDS2_B1:
      nco_step = CirclesToUint32(BDS2_B11_HZ / (double)FAU_RAW_FS);

      for (k = 0, carr_nco = 0; k < FAU_SAMPLE_GRABBER_LENGTH; k++) {
        /** B1-B0 are Channel 1 */
        sample_bitmap = ((sample_buff[k] >> 2) & 0x3) << BBNCO_CARRPH_BITS;
        phase_bitmap =
            (carr_nco >> (32 - BBNCO_CARRPH_BITS)) & BBNCO_CARRPH_MASK;

        h = k / FAU_DECFACT;
        if (FAU_BASEBAND_SIZE == h) break;

        baseband[h].r += bbConvTable[(sample_bitmap | phase_bitmap)].r;
        baseband[h].i += bbConvTable[(sample_bitmap | phase_bitmap)].i;
        carr_nco += nco_step;
      }
      break;

    case CODE_GAL_E7I:
      nco_step = CirclesToUint32(GAL_E7_HZ / (double)FAU_RAW_FS);

      for (k = 0, carr_nco = 0; k < FAU_SAMPLE_GRABBER_LENGTH; k++) {
        /* B7-B6 are Channel 4 */
        sample_bitmap = ((sample_buff[k] >> 6) & 0x3) << BBNCO_CARRPH_BITS;
        phase_bitmap =
            (carr_nco >> (32 - BBNCO_CARRPH_BITS)) & BBNCO_CARRPH_MASK;

        h = k / FAU_DECFACT;
        if (FAU_BASEBAND_SIZE == h) break;

        baseband[h].r += bbConvTable[(sample_bitmap | phase_bitmap)].r;
        baseband[h].i += bbConvTable[(sample_bitmap | phase_bitmap)].i;
        carr_nco += nco_step;
      }
      break;

    default:
      return false;
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
  sFauParams_t fau_conf;
  const u8 *local_code = ca_code(mesid);

  /** sanity checks */
  assert(NULL != pacq_res);
  assert(mesid_valid(mesid));

  fau_conf.iSampMs = FAU_SPMS;

  switch ((s8)mesid.code) {
    case CODE_GPS_L1CA:
    case CODE_QZS_L1CA:
      fau_conf.iNumCodeSlices = FAU_MDBZP_MS_SLICES * GPS_L1CA_PRN_PERIOD_MS;
      fau_conf.iCodeTimeMs = GPS_L1CA_PRN_PERIOD_MS;
      fau_conf.iCohCodes = FAU_GPSL1CA_COHE;
      fau_conf.iNcohAcc = FAU_GPSL1CA_NONC;
      fau_conf.uSecCodeLen = 0;

      code_resample(local_code,
                    GPS_L1CA_CHIPS_NUM,
                    GPS_CA_CHIPPING_RATE,
                    code_upsamp,
                    FAU_SPMS * GPS_L1CA_PRN_PERIOD_MS,
                    FAU_RAW_FS / FAU_DECFACT,
                    BPSK);
      break;

    case CODE_GLO_L1OF:
      fau_conf.iNumCodeSlices = FAU_MDBZP_MS_SLICES * GLO_PRN_PERIOD_MS;
      fau_conf.iCodeTimeMs = GLO_PRN_PERIOD_MS;
      fau_conf.iCohCodes = FAU_GLOG1_COHE;
      fau_conf.iNcohAcc = FAU_GLOG1_NONC;
      fau_conf.uSecCodeLen = 0;

      code_resample(local_code,
                    GLO_CA_CHIPS_NUM,
                    GLO_CA_CHIPPING_RATE,
                    code_upsamp,
                    FAU_SPMS * GLO_PRN_PERIOD_MS,
                    FAU_RAW_FS / FAU_DECFACT,
                    BPSK);
      break;

    case CODE_SBAS_L1CA:
      fau_conf.iNumCodeSlices = FAU_MDBZP_MS_SLICES * SBAS_L1CA_PRN_PERIOD_MS;
      fau_conf.iCodeTimeMs = SBAS_L1CA_PRN_PERIOD_MS;
      fau_conf.iCohCodes = FAU_SBASL1_COHE;
      fau_conf.iNcohAcc = FAU_SBASL1_NONC;
      fau_conf.uSecCodeLen = 0;

      code_resample(local_code,
                    SBAS_L1CA_CHIPS_NUM,
                    SBAS_L1CA_CHIPPING_RATE,
                    code_upsamp,
                    FAU_SPMS * SBAS_L1CA_PRN_PERIOD_MS,
                    FAU_RAW_FS / FAU_DECFACT,
                    BPSK);
      break;

    case CODE_BDS2_B1:
      fau_conf.iNumCodeSlices = FAU_MDBZP_MS_SLICES * BDS2_B11_SYMB_LENGTH_MS;
      fau_conf.iCodeTimeMs = BDS2_B11_SYMB_LENGTH_MS;
      fau_conf.iCohCodes = FAU_BDSB11_COHE;
      fau_conf.iNcohAcc = FAU_BDSB11_NONC;
      fau_conf.uSecCodeLen = 0;

      code_resample(local_code,
                    BDS2_B11_CHIPS_NUM,
                    BDS2_B11_CHIPPING_RATE,
                    code_upsamp,
                    FAU_SPMS * BDS2_B11_PRN_PERIOD_MS,
                    FAU_RAW_FS / FAU_DECFACT,
                    BPSK);
      break;

    case CODE_GAL_E1B:
    case CODE_GAL_E1C:
    case CODE_GAL_E1X:
      fau_conf.iNumCodeSlices = FAU_MDBZP_MS_SLICES * GAL_E1B_PRN_PERIOD_MS;
      fau_conf.iCodeTimeMs = GAL_E1B_PRN_PERIOD_MS;
      fau_conf.iCohCodes = FAU_GALE1_COHE;
      fau_conf.iNcohAcc = FAU_GALE1_NONC;
      fau_conf.uSecCodeLen = 0;
      code_resample(local_code,
                    GAL_E1B_CHIPS_NUM,
                    GAL_E1_CHIPPING_RATE,
                    code_upsamp,
                    FAU_SPMS * GAL_E1B_PRN_PERIOD_MS,
                    FAU_RAW_FS / FAU_DECFACT,
                    BOC_N1);
      break;

    case CODE_GAL_E5I:
    case CODE_GAL_E5Q:
    case CODE_GAL_E5X:
      fau_conf.iNumCodeSlices = FAU_MDBZP_MS_SLICES * GAL_E5Q_PRN_PERIOD_MS;
      fau_conf.iCodeTimeMs = GAL_E5Q_PRN_PERIOD_MS;
      fau_conf.iCohCodes = FAU_GALE5_COHE;
      fau_conf.iNcohAcc = FAU_GALE5_NONC;
      fau_conf.uSecCodeLen = 0;
      code_resample(local_code,
                    GAL_E5_CHIPS_NUM,
                    GAL_E5_CHIPPING_RATE,
                    code_upsamp,
                    FAU_SPMS * GAL_E5Q_PRN_PERIOD_MS,
                    FAU_RAW_FS / FAU_DECFACT,
                    BPSK);
      break;

    case CODE_GAL_E7I:
      fau_conf.iNumCodeSlices = FAU_MDBZP_MS_SLICES * GAL_E7Q_PRN_PERIOD_MS;
      fau_conf.iCodeTimeMs = GAL_E7Q_PRN_PERIOD_MS;
      fau_conf.iCohCodes = FAU_GALE7_COHE;
      fau_conf.iNcohAcc = FAU_GALE7_NONC;
      fau_conf.uSecCodeLen = 0;

      code_resample(local_code,
                    GAL_E7_CHIPS_NUM,
                    GAL_E7_CHIPPING_RATE,
                    code_upsamp,
                    FAU_SPMS * GAL_E7Q_PRN_PERIOD_MS,
                    FAU_RAW_FS / FAU_DECFACT,
                    BPSK);
      break;

    default:
      return false;
  }
  /** call now MDBZP */
  bool ret = mdbzp_static(baseband, code_upsamp, &fau_conf, pacq_res);
  if (ret) {
    log_debug_mesid(mesid,
                    "%16" PRIu64
                    "  fMaxCorr %4.1f  fDoppFreq %.1f  fCodeDelay %.4f",
                    pacq_res->uFirstLocIdx,
                    pacq_res->fMaxCorr,
                    pacq_res->fDoppFreq,
                    pacq_res->fCodeDelay);
  }
  return ret;
}
