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

#ifndef CN0_EST_COMMON_H
#define CN0_EST_COMMON_H

#include <swiftnav/common.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define CN0_THRES_MIN (0.0f)
#define CN0_THRES_MAX (255.0f / 4.0f)

typedef struct {
  u8 count;       /**< Moments estimator running count */
  float M2;       /**< Running sum of second order moments. */
  float M4;       /**< Running sum of fourth order moments. */
  float Pn;       /**< Running sum of noise power. */
  float cn0_dbhz; /**< Carrier to noise ratio in dB/Hz. */
} cn0_est_mm_state_t;

/**
 * Common C/N0 estimator parameters.
 *
 * General C/N0 estimator parameters.
 *
 * \sa c0_est_compute_params
 */
typedef struct {
  float log_bw;        /**< Noise bandwidth in dBHz. */
  u8 t_int_ms;         /**< Integration time */
  float cn0_offset_db; /** < Shift for C/No in dBHz */
} cn0_est_params_t;

/* C/N0 estimators */
void cn0_est_compute_params(cn0_est_params_t *p, u8 loop_dt_ms);

void cn0_est_mm_init(cn0_est_mm_state_t *s, float cn0_0);
void cn0_est_mm_update(cn0_est_mm_state_t *s,
                       const cn0_est_params_t *p,
                       float I,
                       float Q);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* CN0_EST_COMMON_H */
