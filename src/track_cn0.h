/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Valeri Atamaniouk <valeri@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_TRACK_CN0_H
#define SWIFTNAV_TRACK_CN0_H

#include <libswiftnav/track.h>
#include <nap/nap_constants.h>

/* C/N0 estimator IIR averaging coefficient */
/* See http://www.insidegnss.com/auto/IGM_gnss-sol-janfeb10.pdf p. 22 */
/* See http://dsp.stackexchange.com/questions/378/ */
//#define CN0_EST_LPF_ALPHA     (0.016666667f) /* N=72 */
//#define CN0_EST_LPF_ALPHA     (0.005555556f) /* N=200 */
#define CN0_EST_LPF_ALPHA     (.1f)
/* C/N0 LPF cutoff frequency. The lower it is, the more stable CN0 looks like */
#define CN0_EST_LPF_CUTOFF_HZ (.1f)
/* Noise bandwidth: GPS L1 1.023 * 2. Make 16dB offset. */
#define CN0_EST_BW_HZ         (float)(1e6 * 2 / NAP_FRONTEND_SAMPLE_RATE_Hz * 40)
// #define CN0_EST_BW_HZ         (float)(1e6 * 2 / NAP_FRONTEND_SAMPLE_RATE_Hz * 40 * 1000)
//#define CN0_EST_BW_HZ         (2e6f * 4e1f)

/* Configure C/N0 estimator algorithm */
#define cn0_est_state_t           cn0_est_state_t
#define cn0_est_compute_params    cn0_est_compute_params
#define cn0_est_init              cn0_est_bl_init
#define cn0_est_update            cn0_est_bl_update
/* Configure C/N0 value filter algorithm */
#define cn0_filter_params_t       lp1_filter_params_t
#define cn0_filter_compute_params lp1_filter_compute_params
#define cn0_filter_t              lp1_filter_t
#define cn0_filter_init           lp1_filter_init
#define cn0_filter_update         lp1_filter_update

/**
 * C/N0 estimator types
 */
typedef enum {
  TRACK_CN0_EST_BL,  /**< */
  TRACK_CN0_EST_SNV, /**< */
  TRACK_CN0_EST_DEFAULT = TRACK_CN0_EST_BL
} track_cn0_est_e;

/**
 * C/N0 estimator state
 */
typedef struct
{
  cn0_est_params_t    est_params;    /**< C/N0 estimator algorithm */
  cn0_filter_params_t filter_params; /**< Additional C/N0 value LP filter */
} track_cn0_params_t;

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void track_cn0_params_init(void);
const track_cn0_params_t *track_cn0_get_params(track_cn0_est_e t, u8 int_ms,
                                               track_cn0_params_t *p);

void track_cn0_init(track_cn0_est_e t, u8 int_ms, cn0_est_state_t *e,
                    cn0_filter_t *f, float cn0_0);
float track_cn0_update(track_cn0_est_e t, u8 int_ms, cn0_est_state_t *e,
                       cn0_filter_t *f, float I, float Q);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* SWIFTNAV_TRACK_CN0_H */
