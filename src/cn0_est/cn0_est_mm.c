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

#define CN0_EST_MM_INIT_COUNT 200

/** \defgroup track Tracking
 * Functions used in tracking.
 * \{ */

/** Multiplier for checking out-of bounds NSR */
#define CN0_MM_NSR_MIN_MULTIPLIER (1e-6f)
/** Maximum supported NSR value (1/CN0_MM_NSR_MIN_MULTIPLIER)*/
#define CN0_MM_NSR_MIN (1e6f)

static float compute_cn0(const cn0_est_params_t *p, float M_2, float M_4) {
  float tmp = 2 * M_2 * M_2 - M_4;
  float nsr;

  if (0 > tmp) {
    nsr = CN0_MM_NSR_MIN;
  } else {
    float P_d = sqrtf(tmp);
    float P_n = M_2 - P_d;

    /* Ensure the NSR is within the limit */
    if (P_d < P_n * CN0_MM_NSR_MIN_MULTIPLIER)
      return 60;
    else
      nsr = P_n / P_d;
  }

  float nsr_db = 10.f * log10f(nsr);

  /* Compute CN0 */
  float x = p->log_bw - nsr_db;
  return x < 10 ? 10 : x > 60 ? 60 : x;
}

/** Initialize the \f$ C / N_0 \f$ estimator state.
 *
 * Initializes Moment method \f$ C / N_0 \f$ estimator.
 *
 * The method uses the function for C/N0 computation:
 *
 * \f[
 *    \frac{C}{N_0}(n) = \frac{P_d}{P_n}
 * \f]
 * where
 * \f[
 *    P_n(n) = M2(n) - P_d(n)
 * \f]
 * where
 * \f[
 *    P_d(n) = \sqrt{2 * M2(n)^2 - M4(n)}
 * \f]
 * where
 * \f[
 *    M2(n) = \frac{1}{2}(I(n)^2 + I(n-1)^2 + Q(n)^2 + Q(n-1)^2)
 * \f]
 * \f[
 *    M4(n) = \frac{1}{2}(I(n)^4 + I(n-1)^4 + Q(n)^4 + Q(n-1)^4)
 * \f]
 *
 * \param s     The estimator state struct to initialize.
 * \param p     Common C/N0 estimator parameters.
 * \param cn0_0 The initial value of \f$ C / N_0 \f$ in dBHz.
 *
 * \return None
 */
void cn0_est_mm_init(cn0_est_mm_state_t *s,
                     const cn0_est_params_t *p,
                     float cn0_0) {
  memset(s, 0, sizeof(*s));

  (void)p;

  /* Normalize by sampling frequency and integration period */
  s->M_2 = 0.f;
  s->M_4 = 0.f;
  s->cn0_db = cn0_0;
  s->cnt = 0;
  s->lim = (u16)(1 / p->alpha);
}

/**
 * Computes \f$ C / N_0 \f$ with Moment method.
 *
 * \param s Initialized estimator object.
 * \param p Common C/N0 estimator parameters.
 * \param I In-phase signal component
 * \param Q Quadrature phase signal component.
 *
 * \return Computed \f$ C / N_0 \f$ value
 */
float cn0_est_mm_update(cn0_est_mm_state_t *s,
                        const cn0_est_params_t *p,
                        float I,
                        float Q) {
  float m_2 = I * I + Q * Q;
  float m_4 = m_2 * m_2;

  if (s->cnt < s->lim) {
    s->M_2 += m_2;
    s->M_4 += m_4;
    s->cnt++;
    float avg = 1.f / s->lim;
    float M_2 = s->M_2 * avg;
    float M_4 = s->M_4 * avg;

    if (s->cnt == s->lim) {
      s->M_2 = M_2;
      s->M_4 = M_4;
    }

    float alpha = (float)s->cnt / s->lim;

    return s->cn0_db + alpha * (compute_cn0(p, M_2, M_4) - s->cn0_db);
  } else {
    s->M_2 += (m_2 - s->M_2) * p->alpha;
    s->M_4 += (m_4 - s->M_4) * p->alpha;

    /* Compute and store updated CN0 */
    s->cn0_db = compute_cn0(p, s->M_2, s->M_4);
  }

  return s->cn0_db;
}

/** \} */
