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
#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/prns.h>
#include <math.h>
#include <string.h>

#include "lib/fixed_fft_r2.h"

#include "platform_cn0.h"
#include "soft_macq_defines.h"
#include "soft_macq_main.h"

#define SOFTMACQ_FFTLEN_LOG2 (14)
#define CODE_MULT (16384)
#define RESULT_DIV (2048)
#define FFT_SCALE_SCHED_CODE (0x01555555)
#define FFT_SCALE_SCHED_SAMPLES (0x01111111)
#define FFT_SCALE_SCHED_INV (0x01111111)

//~ #define FFT_SAMPLES_INPUT FFT_SAMPLES_INPUT_RF1

#define SOFTMACQ_SAMPLE_RATE_Hz (SOFTMACQ_RAW_FS / SOFTMACQ_DECFACT_GPSL1CA)
#define CODE_SMPS (SOFTMACQ_SAMPLE_RATE_Hz / 1000)

static void code_resample(const me_gnss_signal_t mesid,
                          float chips_per_sample,
                          sc16_t *resampled,
                          u32 resampled_length);
static bool get_bin_min_max(const me_gnss_signal_t mesid,
                            float cf_min,
                            float cf_max,
                            float cf_bin_width,
                            s16 *doppler_bin_min,
                            s16 *doppler_bin_max);
static bool ifft_operations(s16 doppler_bin,
                            float cf_bin_width,
                            u32 fft_len,
                            float fft_bin_width,
                            const sc16_t *code_fft,
                            const sc16_t *sample_fft,
                            float *doppler);
static bool acq_peak_search(const me_gnss_signal_t mesid,
                            float doppler,
                            float fft_bin_width,
                            acq_peak_search_t *peak);

static void GetFourMaxes(const sc16_t *_pcVec, u32 _uSize);

static sc16_t code_fft[INTFFT_MAXSIZE] __attribute__((aligned(32)));
;
static sc16_t sample_fft[INTFFT_MAXSIZE] __attribute__((aligned(32)));
;
static sc16_t result_fft[INTFFT_MAXSIZE] __attribute__((aligned(32)));
;
intFFTr2_t sFftConfig;

static u32 puMaxIdx[4];
static u32 puMaxVal[4];
static u32 puSumVal[4];

float soft_acq_bin_width(void) {
  return SOFTMACQ_SAMPLE_RATE_Hz / (1 << SOFTMACQ_FFTLEN_LOG2);
}

bool soft_acq_search(const sc16_t *_cSignal,
                     const me_gnss_signal_t mesid,
                     float cf_min,
                     float cf_max,
                     float cf_bin_width,
                     acq_result_t *acq_result) {
  /* Configuration */
  u32 fft_len_log2 = SOFTMACQ_FFTLEN_LOG2;
  u32 fft_len = 1 << fft_len_log2;
  assert(fft_len <= INTFFT_MAXSIZE);

  /** init soft FFT */
  if (sFftConfig.N != fft_len) {
    InitIntFFTr2(&sFftConfig, fft_len);
    log_info("InitIntFFTr2()");
  }

  float fft_bin_width = SOFTMACQ_SAMPLE_RATE_Hz / fft_len;
  float chips_per_sample =
      code_to_chip_rate(mesid.code) / SOFTMACQ_SAMPLE_RATE_Hz;

  /** Generate, resample, and FFT code */
  code_resample(mesid, chips_per_sample, code_fft, fft_len);
  DoFwdIntFFTr2(&sFftConfig, code_fft, FFT_SCALE_SCHED_CODE, 1);

  /** Perform the FFT samples without over-writing the input buffer */
  memcpy(sample_fft, _cSignal, sizeof(sc16_t) * fft_len);
  DoFwdIntFFTr2(&sFftConfig, sample_fft, FFT_SCALE_SCHED_SAMPLES, 1);

  /* Search for peak */
  acq_peak_search_t peak = {0};
  s16 doppler_bin_min = 0;
  s16 doppler_bin_max = 0;
  float doppler = 0.0f;

  /* Find minimum and maximum doppler bin index */
  if (!get_bin_min_max(mesid,
                       cf_min,
                       cf_max,
                       cf_bin_width,
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
    doppler_bin = start_bin + ind1 * (ind2 / 2);
    ind1 *= -1;
    ind2 += 1;

    /* If frequency range reached, continue the other frequency side. */
    if (doppler_bin > doppler_bin_max || doppler_bin < doppler_bin_min) {
      continue;
    }
    loop_index += 1;

    /* Multiply and do IFFT */
    if (!ifft_operations(doppler_bin,
                         cf_bin_width,
                         fft_len,
                         fft_bin_width,
                         code_fft,
                         sample_fft,
                         &doppler)) {
      return false;
    }

    /* Find highest peak of the current doppler bin */
    if (!acq_peak_search(mesid, doppler, fft_bin_width, &peak)) {
      return false;
    }

    /* Check if peak strong enough to trigger early exit */
    if (peak.cn0 > ACQ_EARLY_THRESHOLD && !peak_found) {
      peak_found = true; /* Mark peak as found */

      /* IF peak was found on the starting bin,
       * then need to check both sides of the starting bin. */
      if (doppler_bin == start_bin) {
        loop_index = doppler_bin_max - 1; /* Make 2 more searches */
      }
      /* ELSE peak was found on other than starting bin ,
       * then need to check one more bin from the same side. */
      else {
        /* Extend bin boundaries to handle situation
         * where peak is found on last positive or negative bin. */
        doppler_bin_max += 1;
        doppler_bin_min -= 1;
        loop_index = doppler_bin_max; /* Make 1 more search */
        /* Adjust ind1 and ind2 so that same frequency side is searched */
        ind1 *= -1;
        ind2 += 1;
      }
    }
  }

  /* False acquisition code phase hack. The vast majority of our false
   * acquisitions return a code phase zero. Not allowing zero code phase
   * will reject a small number of true acquisitions, but prevents nearly
   * all of the false acquisitions. */
  /* TODO: Check later if this can be removed. */
  if (0 == peak.sample_offset) {
    return false;
  }

  s32 corrected_sample_offset = peak.sample_offset;

  /* Compute code phase */
  float cp = chips_per_sample * corrected_sample_offset;

  /* Set output */
  acq_result->cp = cp;
  acq_result->cf = peak.doppler;
  acq_result->cn0 = peak.cn0;
  return true;
}

/** Resample PRN code for the given ME sid.
 * \param[in] mesid            ME signal id
 * \param[in] chips_per_sample Number of chips per sample.
 * \param[in] resampled        Resampled PRN code
 * \param[in] resampled_length Length of resampled code.
 */
static void code_resample(const me_gnss_signal_t mesid,
                          float chips_per_sample,
                          sc16_t *resampled,
                          u32 resampled_length) {
  const u8 *pCode = ca_code(mesid);
  u32 code_length = code_to_chip_count(mesid.code);

  float chip_offset = 0.0f;
  for (u32 i = 0; i < resampled_length; i++) {
    u32 code_index = (u32)floorf(chip_offset);
    resampled[i] = (sc16_t){
        .r = CODE_MULT * get_chip((u8 *)pCode, code_index % code_length),
        .i = 0};
    chip_offset += chips_per_sample;
  }
}

/** Find dopper_bin_min and doppler_bin_max,
 *  given uncertainty range and bin_width.
 * \param[in]     mesid           ME signal id
 * \param[in]     cf_min          Uncertainty range minimum [Hz]
 * \param[in]     cf_max          Uncertainty range maximum [Hz]
 * \param[in]     cf_bin_width    Doppler bin width [Hz]
 * \param[in,out] doppler_bin_min Minimum doppler bin
 * \param[in,out] doppler_bin_max Maximum doppler bin
 * \retval true  Success
 * \retval false Failure
 */
static bool get_bin_min_max(const me_gnss_signal_t mesid,
                            float cf_min,
                            float cf_max,
                            float cf_bin_width,
                            s16 *doppler_bin_min,
                            s16 *doppler_bin_max) {
  /* Loop over Doppler bins */
  *doppler_bin_min = (s16)floorf(cf_min / cf_bin_width);
  *doppler_bin_max = (s16)floorf(cf_max / cf_bin_width);

  /* Check that bin_max >= bin_min. */
  if (*doppler_bin_min > *doppler_bin_max) {
    log_error_mesid(mesid,
                    "Acq_search: caught bogus dopp_hints (%lf, %lf)",
                    cf_min,
                    cf_max);
    return false;
  }

  /* Check that at least 3 doppler bins are provided,
   * since minimum of 3 bins are searched.
   * If less than 3, just add 2 more. */
  if ((*doppler_bin_max - *doppler_bin_min + 1) < 3) {
    *doppler_bin_max += 1;
    *doppler_bin_min -= 1;
  }
  return true;
}

/** Multiply sample FFT by shifted conjugate code FFT. Perform inverse FFT.
 * \param[in]     mesid         ME signal id
 * \param[in]     doppler_bin   Current doppler bin
 * \param[in]     cf_bin_width  Doppler bin width [Hz]
 * \param[in]     fft_len       FFT length
 * \param[in]     fft_bin_width Doppler bin width [Hz]
 * \param[in]     _pCodeFft      Conjugate code FFT samples
 * \param[in]     _pSampleFft    Sample FFT
 * \param[in]     fft_len_log2  FFT length
 * \param[in,out] doppler       Actual doppler of current frequency bin [Hz]
 * \retval true  Success
 * \retval false Failure
 */
static bool ifft_operations(s16 doppler_bin,
                            float cf_bin_width,
                            u32 fft_len,
                            float fft_bin_width,
                            const sc16_t *_pCodeFft,
                            const sc16_t *_pSampleFft,
                            float *doppler) {
  s32 sample_offset = (s32)round((doppler_bin * cf_bin_width) / fft_bin_width);
  /* Actual computed Doppler */
  *doppler = doppler_bin * cf_bin_width;

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

  return true;
}

/** Read IFFT results from NAP and compute cn0 of highest peak.
 *  If cn0 is new maximum cn0, save cn0, doppler and sample_offset.
 * \param[in]     mesid         ME signal id
 * \param[in]     doppler       Actual doppler of current frequency bin [Hz]
 * \param[in]     fft_len       FFT length
 * \param[in]     fft_bin_width Doppler bin width [Hz]
 * \param[in,out] peak          Max peak parameters
 * \retval true  Success
 * \retval false Failure
 */
static bool acq_peak_search(const me_gnss_signal_t mesid,
                            float doppler,
                            float fft_bin_width,
                            acq_peak_search_t *peak) {
  uint32_t k = 0, kmax = 0;
  u32 peak_index;
  u32 peak_mag_sq;
  u32 sum_mag_sq;
  float snr = 0.0f;
  float cn0 = 0.0f;

  GetFourMaxes(result_fft, CODE_SMPS);
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
    log_error_mesid(mesid, "Acq_search: zero_noise (%u)", sum_mag_sq);
    return false;
  }

  /* Compute C/N0 */
  snr = (float)peak_mag_sq / ((float)sum_mag_sq / (CODE_SMPS / 4));
  cn0 = 10.0f * log10f(snr * PLATFORM_CN0_EST_BW_HZ * fft_bin_width);

  if (cn0 > peak->cn0) {
    /* New max peak found */
    peak->cn0 = cn0;
    peak->doppler = doppler;
    peak->sample_offset = peak_index;
  }

  return true;
}

static void GetFourMaxes(const sc16_t *_pcVec, u32 _uSize) {
  u32 k, uTmpMag, uSz4th;

  if (NULL == _pcVec) return;
  if (_uSize == 0) return;

  puMaxIdx[0] = 0;
  puMaxIdx[1] = 0;
  puMaxIdx[2] = 0;
  puMaxIdx[3] = 0;
  puMaxVal[0] = 0;
  puMaxVal[1] = 0;
  puMaxVal[2] = 0;
  puMaxVal[3] = 0;
  puSumVal[0] = 0;
  puSumVal[1] = 0;
  puSumVal[2] = 0;
  puSumVal[3] = 0;
  uSz4th = _uSize / 4;

  for (k = 0; k < 1 * uSz4th; k++) {
    uTmpMag = ((s32)_pcVec[k].r * (s32)_pcVec[k].r) +
              ((s32)_pcVec[k].i * (s32)_pcVec[k].i);
    puSumVal[0] += uTmpMag;
    if (uTmpMag > puMaxVal[0]) {
      puMaxVal[0] = uTmpMag;
      puMaxIdx[0] = k;
    }
  }
  for (k = 1 * uSz4th; k < 2 * uSz4th; k++) {
    uTmpMag = ((s32)_pcVec[k].r * (s32)_pcVec[k].r) +
              ((s32)_pcVec[k].i * (s32)_pcVec[k].i);
    puSumVal[1] += uTmpMag;
    if (uTmpMag > puMaxVal[1]) {
      puMaxVal[1] = uTmpMag;
      puMaxIdx[1] = k;
    }
  }
  for (k = 2 * uSz4th; k < 3 * uSz4th; k++) {
    uTmpMag = ((s32)_pcVec[k].r * (s32)_pcVec[k].r) +
              ((s32)_pcVec[k].i * (s32)_pcVec[k].i);
    puSumVal[2] += uTmpMag;
    if (uTmpMag > puMaxVal[2]) {
      puMaxVal[2] = uTmpMag;
      puMaxIdx[2] = k;
    }
  }
  for (k = 3 * uSz4th; k < _uSize; k++) {
    uTmpMag = ((s32)_pcVec[k].r * (s32)_pcVec[k].r) +
              ((s32)_pcVec[k].i * (s32)_pcVec[k].i);
    puSumVal[3] += uTmpMag;
    if (uTmpMag > puMaxVal[3]) {
      puMaxVal[3] = uTmpMag;
      puMaxIdx[3] = k;
    }
  }
}
