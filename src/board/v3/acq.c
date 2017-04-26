/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "acq.h"

#include <ch.h>
#include <assert.h>
#include <math.h>
#include <libswiftnav/prns.h>
#include <libswiftnav/logging.h>

#include "nap/nap_constants.h"
#include "nap/fft.h"

#include "platform_cn0.h"

#define CHIP_RATE 1.023e6f
#define CODE_LENGTH 1023
#define CODE_MULT 16384
#define RESULT_DIV 4096
#define FFT_SCALE_SCHED_CODE 0x15555555
#define FFT_SCALE_SCHED_SAMPLES 0x11111111
#define FFT_SCALE_SCHED_INV 0x11110000
#define FFT_SAMPLES_INPUT FFT_SAMPLES_INPUT_RF1

static void code_resample(gnss_signal_t sid, float chips_per_sample,
                          fft_cplx_t *resampled, u32 resampled_length);
static bool get_bin_min_max(gnss_signal_t sid, float cf_min, float cf_max,
                            float cf_bin_width, s16 *doppler_bin_min,
                            s16 *doppler_bin_max);
static bool ifft_operations(s16 doppler_bin, float cf_bin_width,
                            u32 fft_len, float fft_bin_width,
                            const fft_cplx_t *code_fft,
                            const fft_cplx_t *sample_fft,
                            u32 fft_len_log2, float *doppler);
static bool acq_peak_search(gnss_signal_t sid, float doppler, float fft_len,
                            float fft_bin_width, acq_peak_search_t *peak);

float acq_bin_width(void)
{
  return NAP_ACQ_SAMPLE_RATE_Hz / (1 << FFT_LEN_LOG2_MAX);
}

bool acq_search(gnss_signal_t sid, float cf_min, float cf_max,
                float cf_bin_width, acq_result_t *acq_result)
{
  /* Configuration */
  u32 fft_len_log2 = FFT_LEN_LOG2_MAX;
  u32 fft_len = 1 << fft_len_log2;
  float fft_bin_width = NAP_ACQ_SAMPLE_RATE_Hz / fft_len;
  float chips_per_sample = CHIP_RATE / NAP_ACQ_SAMPLE_RATE_Hz;

  /* Generate, resample, and FFT code */
  static FFT_BUFFER(code_fft, fft_cplx_t, FFT_LEN_MAX);
  code_resample(sid, chips_per_sample, code_fft, fft_len);
  if (!fft(code_fft, code_fft, fft_len_log2,
           FFT_DIR_FORWARD, FFT_SCALE_SCHED_CODE)) {
    return false;
  }

  /* FFT samples */
  u32 sample_count;
  static FFT_BUFFER(sample_fft, fft_cplx_t, FFT_LEN_MAX);
  if(!fft_samples(FFT_SAMPLES_INPUT, sample_fft, fft_len_log2,
                  FFT_DIR_FORWARD, FFT_SCALE_SCHED_SAMPLES, &sample_count)) {
    return false;
  }

  /* Search for peak */
  acq_peak_search_t peak = {0};
  s16 doppler_bin_min = 0;
  s16 doppler_bin_max = 0;
  float doppler = 0.0f;

  /* Find minimum and maximum doppler bin index */
  if (!get_bin_min_max(sid, cf_min, cf_max, cf_bin_width,
                       &doppler_bin_min, &doppler_bin_max)) {
    return false;
  }

  /* Start bin is in middle of doppler_bin_min and  doppler_bin_max.
   * If odd number of bins, start from mid bin. [ ][x][ ]
   * If even number of bins, start from (mid + 0.5) bin. [ ][ ][x][ ]
   */
  s16 start_bin = doppler_bin_min
                + (doppler_bin_max - doppler_bin_min + 1) / 2;
  s16 doppler_bin = start_bin;
  s8  ind1 = 1;                     /* Used to flip between +1 and -1 */
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
    if (doppler_bin > doppler_bin_max ||
        doppler_bin < doppler_bin_min) {
      continue;
    }
    loop_index += 1;

    /* Multiply and do IFFT */
    if (!ifft_operations(doppler_bin, cf_bin_width, fft_len, fft_bin_width,
                         code_fft, sample_fft, fft_len_log2, &doppler)) {
      return false;
    }

    /* Find highest peak of the current doppler bin */
    if (!acq_peak_search(sid, doppler, fft_len, fft_bin_width,  &peak)) {
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

  /* Account for non-integer number of codes and circular convolution:
   * If correlation peak is in the first half of the buffer, most samples
   * have NOT wrapped, so assume a positive shift.
   * If correlation peak is in the second half of the buffer, most samples
   * HAVE wrapped, so assume a negative shift. */
  s32 corrected_sample_offset = (peak.sample_offset < fft_len/2) ?
                                (s32)peak.sample_offset :
                                (s32)peak.sample_offset - (s32)fft_len;

  /* Compute code phase */
  float cp = chips_per_sample * corrected_sample_offset;
  /* Modulus code length */
  cp -= CODE_LENGTH * floorf(cp / CODE_LENGTH);

  /* Set output */
  acq_result->sample_count = sample_count;
  acq_result->cp = cp;
  acq_result->cf = peak.doppler;
  acq_result->cn0 = peak.cn0;

  /* False acquisition code phase hack (Michele). The vast majority of our
   * false acquisitions return a code phase within 0.5 chip of 0. Not allowing
   * these code phases will reject a small number of true acquisitions but
   * prevents nearly all the false acquisitions.
   * TODO: Remove this once we move to soft FFT based acquisition. */
  if ((cp<=0.5) || (cp>=1022.5)) return false;

  return true;
}

static void code_resample(gnss_signal_t sid, float chips_per_sample,
                          fft_cplx_t *resampled, u32 resampled_length)
{
  const u8 *code = ca_code(sid);
  u32 code_length = CODE_LENGTH;

  float chip_offset = 0.0f;
  for (u32 i=0; i<resampled_length; i++) {
    u32 code_index = (u32)floorf(chip_offset);
    resampled[i] = (fft_cplx_t) {
      .re = CODE_MULT * get_chip((u8 *)code, code_index % code_length),
      .im = 0
    };
    chip_offset += chips_per_sample;
  }
}

/** Find dopper_bin_min and doppler_bin_max,
 *  given uncertainty range and bin_width.
 * \param[in]     sid             Signal id pointer
 * \param[in]     cf_min          Uncertainty range minimum [Hz]
 * \param[in]     cf_max          Uncertainty range maximum [Hz]
 * \param[in]     cf_bin_width    Doppler bin width [Hz]
 * \param[in,out] doppler_bin_min Minimum doppler bin
 * \param[in,out] doppler_bin_max Maximum doppler bin
 * \retval true  Success
 * \retval false Failure
 */
static bool get_bin_min_max(gnss_signal_t sid, float cf_min, float cf_max,
                            float cf_bin_width, s16 *doppler_bin_min,
                            s16 *doppler_bin_max)
{
  /* Loop over Doppler bins */
  *doppler_bin_min = (s16)floorf(cf_min / cf_bin_width);
  *doppler_bin_max = (s16)floorf(cf_max / cf_bin_width);

  /* Check that bin_max >= bin_min. */
  if (*doppler_bin_min > *doppler_bin_max) {
    log_error_sid(sid, "Acq_search: caught bogus dopp_hints (%lf, %lf)",
                  cf_min, cf_max);
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
 * \param[in]     doppler_bin   Current doppler bin
 * \param[in]     cf_bin_width  Doppler bin width [Hz]
 * \param[in]     fft_len       FFT length
 * \param[in]     fft_bin_width Doppler bin width [Hz]
 * \param[in]     code_fft      Conjugate code FFT samples
 * \param[in]     sample_fft    Sample FFT
 * \param[in]     fft_len_log2  FFT length
 * \param[in,out] doppler       Actual doppler of current frequency bin [Hz]
 * \retval true  Success
 * \retval false Failure
 */
static bool ifft_operations(s16 doppler_bin, float cf_bin_width,
                            u32 fft_len, float fft_bin_width,
                            const fft_cplx_t *code_fft,
                            const fft_cplx_t *sample_fft,
                            u32 fft_len_log2, float *doppler)
{
  s32 sample_offset = (s32)roundf(doppler_bin * cf_bin_width / fft_bin_width);
  /* Actual computed Doppler */
  *doppler = sample_offset * fft_bin_width;

  /* Multiply sample FFT by shifted conjugate code FFT */
  static FFT_BUFFER(result_fft, fft_cplx_t, FFT_LEN_MAX);
  for (u32 i = 0; i < fft_len; i++) {
    const fft_cplx_t *a = &code_fft[i];
    const fft_cplx_t *b = &sample_fft[(i + sample_offset) & (fft_len - 1)];
    fft_cplx_t *r = &result_fft[i];

    s32 a_re = (s32)a->re;
    s32 a_im = (s32)a->im;
    s32 b_re = (s32)b->re;
    s32 b_im = (s32)b->im;

    r->re = ((a_re * b_re) + (a_im * b_im)) / RESULT_DIV;
    r->im = ((a_re * -b_im) + (a_im * b_re)) / RESULT_DIV;
  }

  /* Inverse FFT */
  if (!fft(result_fft, result_fft, fft_len_log2,
           FFT_DIR_BACKWARD, FFT_SCALE_SCHED_INV)) {
    return false;
  }
  return true;
}

/** Read IFFT results from NAP and compute cn0 of highest peak.
 *  If cn0 is new maximum cn0, save cn0, doppler and sample_offset.
 * \param[in]     sid           Signal id pointer
 * \param[in]     doppler       Actual doppler of current frequency bin [Hz]
 * \param[in]     fft_len       FFT length
 * \param[in]     fft_bin_width Doppler bin width [Hz]
 * \param[in,out] peak          Max peak parameters
 * \retval true  Success
 * \retval false Failure
 */
static bool acq_peak_search(gnss_signal_t sid, float doppler, float fft_len,
                            float fft_bin_width, acq_peak_search_t *peak)
{
  u32 peak_index;
  u32 peak_mag_sq;
  u32 sum_mag_sq;
  float snr = 0.0f;
  float cn0 = 0.0f;

  fft_results_get(&peak_index, &peak_mag_sq, &sum_mag_sq);

  if (sum_mag_sq == 0) {
    log_error_sid(sid, "Acq_search: zero_noise (%u)", sum_mag_sq);
    return false;
  }

  /* Compute C/N0 */
  snr = (float)peak_mag_sq / ((float)sum_mag_sq / fft_len);
  cn0 = 10.0f * log10f(snr * PLATFORM_CN0_EST_BW_HZ * fft_bin_width);

  if (cn0 > peak->cn0) {
    /* New max peak found */
    peak->cn0 = cn0;
    peak->doppler = doppler;
    peak->sample_offset = peak_index;
  }

  return true;
}
