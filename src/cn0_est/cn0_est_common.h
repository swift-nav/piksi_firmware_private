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
#include <swiftnav/signal.h>
#include "signal_db/signal_db.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

typedef struct {
  float I_prev_abs; /**< Abs. value of the previous in-phase correlation. */
  float Q_prev_abs; /**< Abs. value of the previous quadrature correlation. */
  float nsr;        /**< Noise to signal ratio. */
} cn0_est_bl_state_t;

typedef struct {
  float I_sum;  /**< Abs. value of the previous in-phase correlation. */
  float P_tot;  /**< Abs. value of the previous quadrature correlation. */
  float cn0_db; /**< Signal to noise ratio in dB/Hz. */
  u32 cnt;
} cn0_est_snv_state_t;

typedef struct {
  float Q_sum;  /**< Running sum of quadrature correlation. */
  float P_tot;  /**< Running sum of combined data and noise power. */
  float cn0_db; /**< Signal to noise ratio in dB/Hz. */
  u32 cnt;
} cn0_est_rscn_state_t;

typedef struct {
  float M2;       /**< Running sum of second order moments. */
  float M4;       /**< Running sum of fourth order moments. */
  float Pn;       /**< Running sum of noise power. */
  float cn0_dbhz; /**< Carrier to noise ratio in dB/Hz. */
} cn0_est_mm_state_t;

typedef struct {
  float WBP;
  float NBP_I;
  float NBP_Q;
  float mu;
  float cn0_db;
  u16 cnt_m;
  u16 cnt_k;
} cn0_est_nwpr_state_t;

typedef struct {
  float I_prev_abs; /**< Abs. value of the previous in-phase correlation. */
  float Q_prev_abs; /**< Abs. value of the previous quadrature correlation. */
  float P_s;
  float P_tot;
  float cn0_db; /**< Noise to signal ratio. */
} cn0_est_svr_state_t;

typedef struct {
  float Q_sum; /**< Running sum of quadrature correlation. */
  float I_sum; /**< Running sum of in-phase correlation. */
  float N_tot;
  float cn0_db; /**< Signal to noise ratio in dB/Hz. */
  u32 cnt;
} cn0_est_ch_state_t;

/**
 * Basic C/No estimator state structure
 */
typedef struct {
  float cn0_db;      /**< Signal to noise ratio in dB/Hz. */
} cn0_est_basic_state_t;

/**
 * Common C/N0 estimator parameters.
 *
 * General C/N0 estimator parameters.
 *
 * \sa c0_est_compute_params
 */
typedef struct {
  float log_bw;    /**< Noise bandwidth in dBHz. */
  float alpha;     /**< IIR filter coeff. for averaging approximation */
  u32 t_int;       /**< Integration time */
  float scale;     /**< Scale for basic estimator */
  float cn0_shift; /**< Shift for C/No in dBHz */
} cn0_est_params_t;

/* C/N0 estimators */
void cn0_est_compute_params(cn0_est_params_t *p,
                            float alpha,
                            float loop_freq,
                            float scale,
                            float cn0_shift);
void cn0_est_bl_init(cn0_est_bl_state_t *s,
                     const cn0_est_params_t *p,
                     float cn0_0);
float cn0_est_bl_update(cn0_est_bl_state_t *s,
                        const cn0_est_params_t *p,
                        float I,
                        float Q);
float cn0_est_bl_update_q(cn0_est_bl_state_t *s,
                          const cn0_est_params_t *p,
                          float I,
                          float Q);
void cn0_est_snv_init(cn0_est_snv_state_t *s,
                      const cn0_est_params_t *p,
                      float cn0_0);
float cn0_est_snv_update(cn0_est_snv_state_t *s,
                         const cn0_est_params_t *p,
                         float I,
                         float Q);
void cn0_est_rscn_init(cn0_est_rscn_state_t *s,
                       const cn0_est_params_t *p,
                       float cn0_0);
float cn0_est_rscn_update(cn0_est_rscn_state_t *s,
                          const cn0_est_params_t *p,
                          float I,
                          float Q);
void cn0_est_mm_init(cn0_est_mm_state_t *s, float cn0_0);
float cn0_est_mm_update(cn0_est_mm_state_t *s,
                        const cn0_est_params_t *p,
                        float I,
                        float Q);
void cn0_est_nwpr_init(cn0_est_nwpr_state_t *s,
                       const cn0_est_params_t *p,
                       float cn0_0);
float cn0_est_nwpr_update(cn0_est_nwpr_state_t *s,
                          const cn0_est_params_t *p,
                          float I,
                          float Q);
void cn0_est_svr_init(cn0_est_svr_state_t *s,
                      const cn0_est_params_t *p,
                      float cn0_0);
float cn0_est_svr_update(cn0_est_svr_state_t *s,
                         const cn0_est_params_t *p,
                         float I,
                         float Q);
void cn0_est_ch_init(cn0_est_ch_state_t *s,
                     const cn0_est_params_t *p,
                     float cn0_0);
float cn0_est_ch_update(cn0_est_ch_state_t *s,
                        const cn0_est_params_t *p,
                        float I,
                        float Q);
void cn0_est_basic_init(cn0_est_basic_state_t *s, float cn0_0);
float cn0_est_basic_update(cn0_est_basic_state_t *s,
                           const cn0_est_params_t *p,
                           float I,
                           float Q,
                           float n);

void noise_calc(code_t code, u8 ms, s32 I, s32 Q);
float noise_get_estimation(code_t code);
void noise_update_mesid_status(me_gnss_signal_t mesid, bool intrack);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* CN0_EST_COMMON_H */
