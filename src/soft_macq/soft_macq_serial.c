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
#include <string.h>
#include <swiftnav/constants.h>
#include <swiftnav/logging.h>
#include <swiftnav/memcpy_s.h>

#include "system_monitor/system_monitor.h"

#include "lib/fixed_fft_r2.h"

#include "platform_cn0.h"
#include "prns.h"
#include "soft_macq_defines.h"
#include "soft_macq_main.h"
#include "soft_macq_utils.h"

#define FAU_SAMPLE_RATE_Hz (FAU_RAW_FS / FAU_DECFACT)
#define CODE_SPMS (FAU_SAMPLE_RATE_Hz / 1000)

#define FAU_FFTLEN_LOG2 14
#define FAU_FFTLEN (1 << FAU_FFTLEN_LOG2)
#define FAU_BIN_WIDTH (FAU_SAMPLE_RATE_Hz / FAU_FFTLEN)
#define CODE_MULT 16384
#define RESULT_DIV 2048
#define FFT_SCALE_SCHED_CODE (0x01555555)
#define FFT_SCALE_SCHED_SAMPLES (0x01111111)
#define FFT_SCALE_SCHED_INV (0x01111111)

static bool get_bin_min_max(const me_gnss_signal_t mesid,
                            float df_min,
                            float df_max,
                            float df_bin_width,
                            s16 *doppler_bin_min,
                            s16 *doppler_bin_max);
static void ifft_operations(s16 doppler_bin,
                            float df_bin_width,
                            u32 fft_len,
                            float fft_bin_width,
                            const sc16_t *code_fft,
                            const sc16_t *sample_fft,
                            float *doppler);
static bool peak_search(const me_gnss_signal_t mesid,
                        sc16_t *c_array,
                        const u32 array_size,
                        const float doppler,
                        const float fft_bin_width,
                        acq_peak_search_t *peak);

static void GetFourMaxes(const u32 *_pcVec, u32 _uSize);

static s8 code_resamp[INTFFT_MAXSIZE] __attribute__((aligned(32)));

static sc16_t code_fft[INTFFT_MAXSIZE] __attribute__((aligned(32)));

static sc16_t sample_fft[INTFFT_MAXSIZE] __attribute__((aligned(32)));

static sc16_t result_fft[INTFFT_MAXSIZE] __attribute__((aligned(32)));

static FFT_DECL(FAU_FFTLEN, sFftConfig);

static u32 puMaxIdx[4];
static u32 puMaxVal[4];
static u32 puSumVal[4];

float soft_acq_bin_width(void) { return FAU_BIN_WIDTH; }

bool soft_acq_search(const sc16_t *_cSignal,
                     const me_gnss_signal_t mesid,
                     float df_min,
                     float df_max,
                     float df_bin_width,
                     acq_result_t *acq_result) {
  /* Configuration */
  if (sFftConfig.N != FAU_FFTLEN) {
    InitIntFFTr2(&sFftConfig, FAU_FFTLEN);
  }

  const u8 *local_code = ca_code(mesid);
  u32 code_length = code_to_chip_count(mesid.code);
  double chip_rate = code_to_chip_rate(mesid.code);
  memset(code_resamp, 0, sizeof(s8) * INTFFT_MAXSIZE);
  if ((CODE_SBAS_L1CA == mesid.code) || (CODE_BDS2_B1 == mesid.code) ||
      (CODE_GAL_E7I == mesid.code) || (CODE_GAL_E5I == mesid.code)) {
    /* For constellations with frequent symbol transitions, do 1x4 CxNC */
    code_resample(local_code,
                  code_length,
                  chip_rate,
                  code_resamp + CODE_SPMS * (FAU_FFTLEN / CODE_SPMS - 1),
                  CODE_SPMS,
                  FAU_SAMPLE_RATE_Hz,
                  BPSK);
  } else if (CODE_GAL_E1B == mesid.code) {
    code_resample(local_code,
                  code_length,
                  chip_rate,
                  code_resamp,
                  FAU_FFTLEN,
                  FAU_SAMPLE_RATE_Hz,
                  BOC_N1);
  } else {
    code_resample(local_code,
                  code_length,
                  chip_rate,
                  code_resamp,
                  FAU_FFTLEN,
                  FAU_SAMPLE_RATE_Hz,
                  BPSK);
  }

  for (u32 k = 0; k < FAU_FFTLEN; k++) {
    code_fft[k].r = CODE_MULT * code_resamp[k];
    code_fft[k].i = 0;
  }

  DoFwdIntFFTr2(&sFftConfig, code_fft, FFT_SCALE_SCHED_CODE, 1);

  /** Perform the FFT samples without over-writing the input buffer */
  MEMCPY_S(
      sample_fft, sizeof(sample_fft), _cSignal, sizeof(sc16_t) * FAU_FFTLEN);
  DoFwdIntFFTr2(&sFftConfig, sample_fft, FFT_SCALE_SCHED_SAMPLES, 1);

  /* simple notch filter */
  if (CODE_BDS2_B1 == mesid.code) {
    sample_fft[11857].r = 0;
    sample_fft[11857].i = 0;
    sample_fft[11858].r = 0;
    sample_fft[11858].i = 0;
    sample_fft[11859].r = 0;
    sample_fft[11859].i = 0;
  }

  /* Search for peak */
  acq_peak_search_t peak = {0};
  s16 doppler_bin_min = 0;
  s16 doppler_bin_max = 0;
  float doppler = 0.0f;

  /* Find minimum and maximum doppler bin index */
  if (!get_bin_min_max(mesid,
                       df_min,
                       df_max,
                       df_bin_width,
                       &doppler_bin_min,
                       &doppler_bin_max)) {
    return false;
  }

  /* Start bin is in middle of doppler_bin_min and  doppler_bin_max.
   * If odd number of bins, start from mid bin. [ ][x][ ]
   * If even number of bins, start from (mid + 0.5) bin. [ ][ ][x][ ]
   */
  s16 start_bin = doppler_bin_min + (doppler_bin_max - doppler_bin_min + 1) / 2;
  s16 doppler_bin = start_bin;
  s8 ind1 = 1;                      /* Used to flip between +1 and -1 */
  s16 ind2 = 1;                     /* Used to compute bin index with (ind2 / 2)
                                     * resulting in sequence
                                     * 0,1,1,2,2,3,3,... */
  bool peak_found = false;          /* Stop freq sweep when peak is found.
                                     * Adjacent freq bins are still searched. */
  s16 loop_index = doppler_bin_min; /* Make frequency searches from
                                     * doppler_bin_min to doppler_bin_max */

  while (loop_index <= doppler_bin_max) {
    watchdog_notify(WD_NOTIFY_ACQ_MGMT);
    doppler_bin = start_bin + ind1 * (ind2 / 2);
    ind1 *= -1;
    ind2 += 1;

    /* If frequency range reached, continue the other frequency side. */
    if (doppler_bin > doppler_bin_max || doppler_bin < doppler_bin_min) {
      continue;
    }
    loop_index += 1;

    /* Multiply and do IFFT */
    ifft_operations(doppler_bin,
                    df_bin_width,
                    FAU_FFTLEN,
                    FAU_BIN_WIDTH,
                    code_fft,
                    sample_fft,
                    &doppler);

    /* blank edges of FFT to compensate for numerical instability of the fixed
     * point FFT */
    result_fft[0].r = 0;
    result_fft[0].i = 0;
    result_fft[FAU_FFTLEN - 1].r = 0;
    result_fft[FAU_FFTLEN - 1].i = 0;

    /* Find highest peak of the current doppler bin */
    if (!peak_search(
            mesid, result_fft, FAU_FFTLEN, doppler, FAU_BIN_WIDTH, &peak)) {
      return false;
    }

    /* Check if peak strong enough to trigger early exit */
    if (peak.cn0 > ACQ_EARLY_THRESHOLD && !peak_found) {
      peak_found = true; /* Mark peak as found */

      /* IF peak was found on the starting bin,
       * then need to check both sides of the starting bin. */
      if (doppler_bin == start_bin) {
        loop_index = doppler_bin_max - 3; /* Make 4 more searches */
      }
      /* ELSE peak was found on other than starting bin ,
       * then need to check one more bin from the same side. */
      else {
        /* Extend bin boundaries to handle situation
         * where peak is found on last positive or negative bin. */
        doppler_bin_max += 2;
        doppler_bin_min -= 2;
        loop_index = doppler_bin_max - 2; /* Make 3 more search */
        /* Adjust ind1 and ind2 so that same frequency side is searched */
        ind1 *= -1;
        ind2 += 1;
      }
    }
  }

  /* Compute code phase */
  float chips_per_sample = chip_rate / FAU_SAMPLE_RATE_Hz;
  float cp = chips_per_sample * (peak.sample_offset);

  /* Set output */
  acq_result->cp = cp;
  acq_result->df = peak.doppler;
  acq_result->cn0 = peak.cn0;
  return true;
}

/** Find dopper_bin_min and doppler_bin_max,
 *  given uncertainty range and bin_width.
 * \param[in]     mesid           ME signal id
 * \param[in]     df_min          Uncertainty range minimum [Hz]
 * \param[in]     df_max          Uncertainty range maximum [Hz]
 * \param[in]     df_bin_width    Doppler bin width [Hz]
 * \param[in,out] doppler_bin_min Minimum doppler bin
 * \param[in,out] doppler_bin_max Maximum doppler bin
 * \retval true  Success
 * \retval false Failure
 */
static bool get_bin_min_max(const me_gnss_signal_t mesid,
                            float df_min,
                            float df_max,
                            float df_bin_width,
                            s16 *doppler_bin_min,
                            s16 *doppler_bin_max) {
  /* Loop over Doppler bins */
  *doppler_bin_min = (s16)floorf(df_min / df_bin_width);
  *doppler_bin_max = (s16)floorf(df_max / df_bin_width);

  /* Check that bin_max >= bin_min. */
  if (*doppler_bin_min > *doppler_bin_max) {
    log_error_mesid(mesid,
                    "Acq_search: caught bogus dopp_hints (%lf, %lf)",
                    df_min,
                    df_max);
    return false;
  }

  /* Check that at least 5 doppler bins are provided,
   * since minimum of 5 bins are searched.
   * If less than 5, just add 4 more. */
  if ((*doppler_bin_max - *doppler_bin_min + 1) < 5) {
    *doppler_bin_max += 2;
    *doppler_bin_min -= 2;
  }
  return true;
}

/** Multiply sample FFT by shifted conjugate code FFT. Perform inverse FFT.
 * \param[in]     mesid         ME signal id
 * \param[in]     doppler_bin   Current doppler bin
 * \param[in]     df_bin_width  Doppler bin width [Hz]
 * \param[in]     fft_len       FFT length
 * \param[in]     fft_bin_width Doppler bin width [Hz]
 * \param[in]     _pCodeFft      Conjugate code FFT samples
 * \param[in]     _pSampleFft    Sample FFT
 * \param[in]     fft_len_log2  FFT length
 * \param[in,out] doppler       Actual doppler of current frequency bin [Hz]
 */
static void ifft_operations(s16 doppler_bin,
                            float df_bin_width,
                            u32 fft_len,
                            float fft_bin_width,
                            const sc16_t *_pCodeFft,
                            const sc16_t *_pSampleFft,
                            float *doppler) {
  s32 sample_offset = (s32)round((doppler_bin * df_bin_width) / fft_bin_width);
  /* Actual computed Doppler */
  *doppler = doppler_bin * df_bin_width;

  /* Multiply sample FFT by shifted conjugate code FFT */
  for (u32 i = 0; i < fft_len; i++) {
    const sc16_t *a = &_pCodeFft[i];
    const sc16_t *b = &_pSampleFft[(i + sample_offset) & (fft_len - 1)];
    sc16_t *r = &result_fft[i];

    s32 a_re = (s32)a->r;
    s32 a_im = (s32)a->i;
    s32 b_re = (s32)b->r;
    s32 b_im = (s32)b->i;

    r->r = ((a_re * b_re) + (a_im * b_im)) / RESULT_DIV;
    r->i = ((a_re * -b_im) + (a_im * b_re)) / RESULT_DIV;
  }

  /* Inverse FFT */
  DoBwdIntFFTr2(&sFftConfig, result_fft, FFT_SCALE_SCHED_INV, 1);
}

/** Read IFFT results from NAP and compute cn0 of highest peak.
 *  If cn0 is new maximum cn0, save cn0, doppler and sample_offset.
 * \param[in]     mesid         ME signal id
 * \param[in]     c_array       Complex input array
 * \param[in]     array_sz      Array size
 * \param[in]     doppler       Actual doppler of current frequency bin [Hz]
 * \param[in]     fft_len       FFT length
 * \param[in]     fft_bin_width Doppler bin width [Hz]
 * \param[in,out] peak          Max peak parameters
 * \retval true  Success
 * \retval false Failure
 */
static bool peak_search(const me_gnss_signal_t mesid,
                        sc16_t *c_array,
                        const u32 array_sz,
                        const float doppler,
                        const float fft_bin_width,
                        acq_peak_search_t *peak) {
  u32 k = 0, kmax = 0;
  u32 peak_index;
  u32 peak_mag_sq;
  u32 sum_mag_sq;
  float snr = 0.0f;
  float cn0 = 0.0f;
  u32 *result_mag = (u32 *)c_array;

  /* In place magnitude */
  for (k = 0; k < array_sz; k++) {
    s32 re = c_array[k].r;
    s32 im = c_array[k].i;
    result_mag[k] = (u32)(re * re) + (u32)(im * im);
  }

  /* For constellations with frequent symbol transitions,
   * accumulate non-coherently */
  if ((CODE_SBAS_L1CA == mesid.code) || (CODE_BDS2_B1 == mesid.code)) {
    result_mag[0] = 0;
    result_mag[1] = 0;
    u8 non_coh = array_sz / CODE_SPMS;
    for (u32 m = 1; m < non_coh; m++) {
      for (u32 h = 0; h < CODE_SPMS; h++) {
        u32 src_idx = m * CODE_SPMS + h;
        if (src_idx >= array_sz) break;
        result_mag[h] += result_mag[src_idx];
      }
    }
  }

  GetFourMaxes(result_mag, CODE_SPMS);
  peak_mag_sq = 0;
  peak_index = 0;
  for (k = 0; k < 4; k++) {
    if (puMaxVal[k] > peak_mag_sq) {
      peak_mag_sq = puMaxVal[k];
      peak_index = puMaxIdx[k];
      kmax = k;
    }
  }
  sum_mag_sq = puSumVal[(kmax + 2) % 4];

  if (sum_mag_sq == 0) {
    log_error_mesid(mesid, "Acq_search: zero_noise (%" PRIu32 ")", sum_mag_sq);
    return false;
  }

  /* Compute C/N0 */
  snr = (float)peak_mag_sq / ((float)sum_mag_sq / (CODE_SPMS / 4));
  cn0 = 10.0f * log10f(snr * PLATFORM_CN0_EST_BW_HZ * fft_bin_width);

  /* artificially pump the C/N0 for non-coherent as MEAN is not STD */
  if (CODE_SBAS_L1CA == mesid.code) {
    cn0 += 4.0;
  }
  if (CODE_BDS2_B1 == mesid.code) {
    cn0 += 4.0;
  }

  if (cn0 > peak->cn0) {
    /* New max peak found */
    peak->cn0 = cn0;
    peak->doppler = doppler;
    peak->sample_offset = peak_index;
  }

  return true;
}

/**
 * Gets the maximums in the four quarter-slices of the input array
 * \param[in]     _puVec       Input array of magnitude values
 * \param[in]     _uSize       Array size
 */
static void GetFourMaxes(const u32 *_puVec, u32 _uSize) {
  u32 k, uTmpMag, uSz4th;

  if (NULL == _puVec) return;
  if (_uSize == 0) return;

  memset(puMaxIdx, 0, 4 * sizeof(u32));
  memset(puMaxVal, 0, 4 * sizeof(u32));
  memset(puSumVal, 0, 4 * sizeof(u32));
  uSz4th = _uSize / 4;

  for (k = 0; k < 1 * uSz4th; k++) {
    uTmpMag = _puVec[k];
    puSumVal[0] += uTmpMag;
    if (uTmpMag > puMaxVal[0]) {
      puMaxVal[0] = uTmpMag;
      puMaxIdx[0] = k;
    }
  }
  for (k = 1 * uSz4th; k < 2 * uSz4th; k++) {
    uTmpMag = _puVec[k];
    puSumVal[1] += uTmpMag;
    if (uTmpMag > puMaxVal[1]) {
      puMaxVal[1] = uTmpMag;
      puMaxIdx[1] = k;
    }
  }
  for (k = 2 * uSz4th; k < 3 * uSz4th; k++) {
    uTmpMag = _puVec[k];
    puSumVal[2] += uTmpMag;
    if (uTmpMag > puMaxVal[2]) {
      puMaxVal[2] = uTmpMag;
      puMaxIdx[2] = k;
    }
  }
  for (k = 3 * uSz4th; k < _uSize; k++) {
    uTmpMag = _puVec[k];
    puSumVal[3] += uTmpMag;
    if (uTmpMag > puMaxVal[3]) {
      puMaxVal[3] = uTmpMag;
      puMaxIdx[3] = k;
    }
  }
}
