/*
 * Copyright (C) 2016-2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <math.h>
#include <string.h>

#include "cn0_est_common.h"

/** \defgroup track Tracking
 * Functions used in tracking.
 * \{ */

/** Multiplier for checking out-of bounds NSR */
#define CN0_BL_NSR_MIN_MULTIPLIER (1e-6f)
/** Maximum supported NSR value (1/NSR_MIN_MULTIPLIER)*/
#define CN0_BL_NSR_MIN (1e6f)
/** Maximum estimator output value */
#define CN0_BL_DB_MAX (100.f)
/** Minimum estimator output value */
#define CN0_BL_DB_MIN (0.f)

static float limit_cn0(float cn0) {
  if (cn0 > CN0_BL_DB_MAX)
    return CN0_BL_DB_MAX;
  else if (cn0 < CN0_BL_DB_MIN)
    return CN0_BL_DB_MIN;
  else
    return cn0;
}

static float compute_nsr(float P_s, float P_n) {
  /* Ensure the NSR is within the limit */
  if (P_s < P_n * CN0_BL_NSR_MIN_MULTIPLIER)
    return CN0_BL_NSR_MIN;
  else
    return P_n / P_s;
}

/**
 * Initializes C/No filter parameters
 * \param[out] p Pointer to estimator parameters
 * \param[in]  bw Noise bandwidth, Hz
 * \param[in]  alpha Cutoff frequency
 * \param[in]  loop_freq Loop frequency, Hz
 * \param[in]  scale Scale coefficient for output C/No values
 * \param[in]  cn0_shift shift for output C/No values, dB-Hz
 */
void cn0_est_compute_params(cn0_est_params_t *p,
                            float bw,
                            float alpha,
                            float loop_freq,
                            float scale,
                            float cn0_shift) {
  (void)bw;
  memset(p, 0, sizeof(*p));

  p->log_bw = 10.f * log10f(loop_freq);
  p->alpha = alpha;
  p->scale = scale;
  p->cn0_shift = cn0_shift;
}

/** Initialize the \f$ C / N_0 \f$ estimator state.
 *
 * Initializes Beaulieu's method \f$ C / N_0 \f$ estimator.
 *
 * The method uses the function for C/N0 computation:
 *
 * \f[
 *    \frac{C}{N_0}(n) = \frac{1}{2}(\frac{P_n(n)}{P_s(n)})
 * \f]
 * where
 * \f[
 *    P_n(n) = (|Q(n)|-|Q(n-1)|)^2
 * \f]
 * for original BL method.
 * \f[
 *    P_n(n) = (|I(n)|-|I(n-1)|)^2
 * \f]
 * for modified BL method.
 * \f[
 *    P_s(n) = \frac{1}{2}(I(n)^2 + I(n-1)^2)
 * \f]
 *
 * \param s     C/N0 estimator tacking state.
 * \param p     C/N0 estimator parameters.
 * \param cn0_0 The initial value of \f$ C / N_0 \f$ in dBHz.
 *
 * \return None
 *
 * References:
 *    -# "Comparison of Four SNR Estimators for QPSK Modulations",
 *       Norman C. Beaulieu, Andrew S. Toms, and David R. Pauluzzi (2000),
 *       IEEE Communications Letters, Vol. 4, No. 2
 *    -# "Are Carrier-to-Noise Algorithms Equivalent in All Situations?"
 *       Inside GNSS, Jan / Feb 2010.
 *
 * \sa cn0_est_bl_update
 * \sa cn0_est_bl_update_q
 */
void cn0_est_bl_init(cn0_est_bl_state_t *s,
                     const cn0_est_params_t *p,
                     float cn0_0) {
  memset(s, 0, sizeof(*s));

  (void)p;

  /* Normalize by sampling frequency and integration period */
  s->I_prev_abs = -1.f;
  s->Q_prev_abs = -1.f;
  s->nsr = powf(10.f, 0.1f * (p->log_bw - cn0_0));
}

/**
 * Computes \f$ C / N_0 \f$ with Beaulieu's method.
 *
 * \param s Initialized estimator object.
 * \param p C/N0 estimator parameters.
 * \param I In-phase signal component
 * \param Q Quadrature phase signal component.
 *
 * \return Computed \f$ C / N_0 \f$ value
 */
float cn0_est_bl_update(cn0_est_bl_state_t *s,
                        const cn0_est_params_t *p,
                        float I,
                        float Q) {
  /* Compute values for this iteration */
  float I_abs = fabsf(I);
  (void)Q;
  float I_prev_abs = s->I_prev_abs;
  s->I_prev_abs = I_abs;

  if (I_prev_abs < 0.f) {
    /* This is the first iteration, just update the prev state. */
  } else {
    float P_n; /* Noise power */
    float P_s; /* Signal power */
    float nsr; /* Noise to signal ratio */

    P_n = I_abs - I_prev_abs;
    P_n *= P_n;
    P_s = 0.5f * (I * I + I_prev_abs * I_prev_abs);
    nsr = compute_nsr(P_s, P_n);

    s->nsr += (nsr - s->nsr) * p->alpha;
  }

  float res = limit_cn0(p->log_bw - 10.f * log10f(s->nsr));

  return res;
}

/**
 * Computes \f$ C / N_0 \f$ with Beaulieu's method.
 *
 * \param s Initialized estimator object.
 * \param p C/N0 estimator parameters.
 * \param I In-phase signal component
 * \param Q Quadrature phase signal component.
 *
 * \return Computed \f$ C / N_0 \f$ value
 */
float cn0_est_bl_update_q(cn0_est_bl_state_t *s,
                          const cn0_est_params_t *p,
                          float I,
                          float Q) {
  /* Compute values for this iteration */
  float I_abs = fabsf(I);
  float Q_abs = fabsf(Q);
  float I_prev_abs = s->I_prev_abs;
  float Q_prev_abs = s->Q_prev_abs;
  s->I_prev_abs = I_abs;
  s->Q_prev_abs = Q_abs;

  if (I_prev_abs < 0.f) {
    /* This is the first iteration, just update the prev state. */
  } else {
    float P_n; /* Noise power */
    float P_s; /* Signal power */
    float nsr; /* Noise to signal ratio */

    P_n = Q_abs - Q_prev_abs;
    P_n *= P_n;
    P_s = 0.5f * (I * I + I_prev_abs * I_prev_abs);
    nsr = compute_nsr(P_s, P_n);

    s->nsr += (nsr - s->nsr) * p->alpha;
  }

  float res = limit_cn0(p->log_bw - 10.f * log10f(s->nsr));

  return res;
}

/** \} */
