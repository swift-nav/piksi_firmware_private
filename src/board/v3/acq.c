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
#define RESULT_DIV 32
#define FFT_SCALE_SCHED_CODE 0x15555555
#define FFT_SCALE_SCHED_SAMPLES 0x15555555
#define FFT_SCALE_SCHED_INV 0x15550000
#define FFT_SAMPLES_INPUT FFT_SAMPLES_INPUT_RF1

static void code_resample(gnss_signal_t sid, float chips_per_sample,
                          fft_cplx_t *resampled, u32 resampled_length);

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

  /* Search for results */
  float best_cn0 = 0.0f;
  float best_doppler = 0.0f;
  u32 best_sample_offset = 0;
  float snr = 0.0f;
  float cn0 = 0.0f;

  /* Loop over Doppler bins */
  s32 doppler_bin_min = (s32)floorf(cf_min / cf_bin_width);
  s32 doppler_bin_max = (s32)floorf(cf_max / cf_bin_width);
  s32 start_bin = 0;                /* Start search from center bin */
  s32 doppler_bin = start_bin;
  s32 ind1 = 1;                     /* Used to flip between +1 and -1 */
  s32 ind2 = 1;                     /* Used to compute bin index with (ind2 / 2)
                                     * resulting in sequence
                                     * 0,1,1,2,2,3,3,... */
  bool peak_found = false;          /* Stop freq sweep when peak is found.
                                     * Adjacent freq bins are still searched. */
  s32 loop_index = doppler_bin_min; /* Make frequency searches from
                                     * dopple_bin_min to doppler_bin_max */

  while (loop_index <= doppler_bin_max) {
    doppler_bin = start_bin + ind1 * (ind2 / 2);
    ind1 *= -1;
    ind2 += 1;
    if (doppler_bin > doppler_bin_max || doppler_bin < doppler_bin_min) {
      /* If frequency range reached, continue the other frequency side. */
      continue;
    }
    loop_index += 1;
    s32 sample_offset = (s32)roundf(doppler_bin * cf_bin_width / fft_bin_width);
    /* Actual computed Doppler */
    float doppler = sample_offset * fft_bin_width;

    /* Multiply sample FFT by shifted conjugate code FFT */
    static FFT_BUFFER(result_fft, fft_cplx_t, FFT_LEN_MAX);
    for (u32 i=0; i<fft_len; i++) {
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

    /* Peak search */
    u32 peak_index;
    u32 peak_mag_sq;
    u32 sum_mag_sq;
    fft_results_get(&peak_index, &peak_mag_sq, &sum_mag_sq);

    /* Compute C/N0 */
    snr = (float)peak_mag_sq / ((float)sum_mag_sq / fft_len);
    cn0 = 10.0f * log10f(snr * PLATFORM_CN0_EST_BW_HZ * fft_bin_width);
    if (cn0 > ACQ_THRESHOLD) {
      /* Peak found - save results */
      best_cn0 = cn0;
      best_doppler = doppler;
      best_sample_offset = peak_index;

      /* If peak was found on the starting bin,
       * then need to check both sides of the starting bin */
      if (doppler_bin == start_bin) {
        loop_index = doppler_bin_max - 1; /* Make 2 more searches */
        peak_found = true;                /* Mark peak as found */
        continue;
      }

      /* IF no Peak has been found previously.
       *    AND
       *    (
       *      (Peak is now found on positive side AND
       *       there is one more bin to search on that side)
       *      OR
       *      (Peak is now found on negative side AND
       *       there is one more bin to search on that side)
       *    )
       */
      if (
          (!peak_found)  &&
          (((ind1 == -1) && (doppler_bin < doppler_bin_max)) ||
           ((ind1 == 1)  && (doppler_bin > doppler_bin_min)))
         ) {
        loop_index = doppler_bin_max;  /* Make 1 more search */
        peak_found = true;             /* Mark peak as found */
        /* Adjust ind1 and ind2 so that same frequency side is searched */
        ind1 *= -1;
        ind2 += 1;
        continue;
      }
    }
  }

  /* Account for non-integer number of codes and circular convolution:
   * If correlation peak is in the first half of the buffer, most samples
   * have NOT wrapped, so assume a positive shift.
   * If correlation peak is in the second half of the buffer, most samples
   * HAVE wrapped, so assume a negative shift. */
  s32 corrected_sample_offset = (best_sample_offset < fft_len/2) ?
                                (s32)best_sample_offset :
                                (s32)best_sample_offset - (s32)fft_len;

  /* Compute code phase */
  float cp = chips_per_sample * corrected_sample_offset;
  /* Modulus code length */
  cp -= CODE_LENGTH * floorf(cp / CODE_LENGTH);

  /* Set output */
  acq_result->sample_count = sample_count;
  acq_result->cp = cp;
  acq_result->cf = best_doppler;
  acq_result->cn0 = best_cn0;
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
