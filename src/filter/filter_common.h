/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_FILTER_COMMON_H
#define SWIFTNAV_FILTER_COMMON_H

/** Parameters structure for first order low-pass filter.
 *
 * \see lp1_filter_init
 * \see lp1_filter_update
 */
typedef struct {
  float b; /**< IIR filter coeff. */
  float a; /**< IIR filter coeff. */
} lp1_filter_params_t;

/** State structure for first order low-pass filter.
 *
 * \see lp1_filter_init
 * \see lp1_filter_update
 */
typedef struct {
  float xn; /**< Last pre-filter sample. */
  float yn; /**< Last post-filter sample. */
} lp1_filter_t;

/**
 * Second order Butterworth filter parameters.
 *
 * \see bw2_filter_compute_params
 * \see bw2_filter_init
 * \see bw2_filter_update
 */
typedef struct {
  float b;  /**< IIR filter coeff. */
  float a2; /**< IIR filter coeff. */
  float a3; /**< IIR filter coeff. */
} bw2_filter_params_t;

/**
 * Second order Butterworth filter object.
 *
 * Structure for filtering CN0 values using 2nd order Butterworth filter.
 *
 * \see bw2_filter_init
 * \see bw2_filter_update
 */
typedef struct {
  float yn;      /**< Last post-filter sample. */
  float yn_prev; /**< Previous post-filter sample. */
  float xn;      /**< Last pre-filter sample. */
  float xn_prev; /**< Previous pre-filter sample. */
} bw2_filter_t;

#ifdef __cplusplus
extern "C" {
#endif

/* General purpose filters */
void lp1_filter_compute_params(lp1_filter_params_t *p,
                               float cutoff_freq,
                               float loop_freq);
void lp1_filter_init(lp1_filter_t *f,
                     const lp1_filter_params_t *p,
                     float initial);
float lp1_filter_update(lp1_filter_t *f,
                        const lp1_filter_params_t *p,
                        float value);

void bw2_filter_compute_params(bw2_filter_params_t *p,
                               float cutoff_freq,
                               float loop_freq);
void bw2_filter_init(bw2_filter_t *f,
                     const bw2_filter_params_t *p,
                     float initial);
float bw2_filter_update(bw2_filter_t *f,
                        const bw2_filter_params_t *p,
                        float value);

#ifdef __cplusplus
}
#endif

#endif /* SWIFTNAV_FILTER_COMMON_H */
