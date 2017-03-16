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

#ifndef SWIFTNAV_FFT_H
#define SWIFTNAV_FFT_H

#include <libswiftnav/common.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/constants.h>

#define FFT_LEN_LOG2_MIN 10
#define FFT_LEN_LOG2_MAX 15

#define FFT_LEN_MIN (1 << FFT_LEN_LOG2_MIN)
#define FFT_LEN_MAX (1 << FFT_LEN_LOG2_MAX)

#define FFT_BUFFER_ALIGN 32
#define FFT_LENGTH_ALIGN 32
#define FFT_CEIL_DIV(a,b) (((a) + (b) - 1) / (b))

#define FFT_BUFFER_LENGTH_BYTES(type, count)                                  \
    (FFT_LENGTH_ALIGN * FFT_CEIL_DIV(count * sizeof(type), FFT_LENGTH_ALIGN))
#define FFT_BUFFER_LENGTH_ELEMENTS(type, count)                               \
    (FFT_CEIL_DIV(FFT_BUFFER_LENGTH_BYTES(type, count), sizeof(type)))

#define FFT_BUFFER(name, type, count)                                         \
    type name[FFT_BUFFER_LENGTH_ELEMENTS(type, count)]                        \
              __attribute__((aligned(FFT_BUFFER_ALIGN)))

typedef struct __attribute__((packed)) {
  s16 re;
  s16 im;
} fft_cplx_t;

typedef enum {
  FFT_DIR_BACKWARD = 0,
  FFT_DIR_FORWARD = 1
} fft_dir_t;

typedef enum {
  FFT_SAMPLES_INPUT_RF1 = 0,
  FFT_SAMPLES_INPUT_RF2 = 1,
  FFT_SAMPLES_INPUT_RF3 = 2,
  FFT_SAMPLES_INPUT_RF4 = 3,
} fft_samples_input_t;

bool fft(const fft_cplx_t *in, fft_cplx_t *out, u32 len_log2,
         fft_dir_t dir, u32 scale_schedule);

bool fft_samples(me_gnss_signal_t mesid, fft_cplx_t *out,
                 u32 len_log2, fft_dir_t dir, u32 scale_schedule,
                 u32 *sample_count);

void fft_results_get(u32 *peak_index, u32 *peak_mag_sq, u32 *sum_mag_sq);

bool raw_samples(u8 *out, u32 len_words, u32 *sample_count);

#endif /* SWIFTNAV_FFT_H */
