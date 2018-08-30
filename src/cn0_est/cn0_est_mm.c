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

/** Mean of N moments. Minimum value N = 2 for fastest response time */
#define CN0_MM_N (2)
/** Mean multiplier */
#define CN0_MM_MEAN_MULT (1.0f / CN0_MM_N)
/** CNO smoothing time in milliseconds. Removes CN0 bumps in the beginning */
#define CN0_MM_CNO_SMOOTH_MS (2000)

/** Initialize the \f$ C / N_0 \f$ estimator state.
 *
 * Initializes Moment method \f$ C / N_0 \f$ estimator.
 *
 * The method uses the function for C/N0 computation:
 *
 * \f[
 *    \frac{C}{N_0}(n) = \frac{Pd}{Pn}
 * \f]
 * where
 * \f[
 *    Pn(n) = M2(n) - Pd(n)
 * \f]
 * where
 * \f[
 *    Pd(n) = \sqrt{2 * M2(n)^2 - M4(n)}
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
 * \param cn0_0 The initial value of \f$ C / N_0 \f$ in dBHz.
 *
 * \return None
 */
void cn0_est_mm_init(cn0_est_mm_state_t *s, float cn0_0) {
  memset(s, 0, sizeof(*s));

  s->M2 = -1.0f; /* Set negative for first iteration */
  s->M4 = -1.0f;
  s->Pn = 0.0f;
  s->cnt_ms = 0;
  s->cn0_db = cn0_0;
  s->cn0_init = cn0_0;
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
  float m2 = I * I + Q * Q;
  float m4 = m2 * m2;

  if (s->M2 < 0.0f) {
    /* This is the first iteration, just initialize moments. */
    s->M2 = m2;
    s->M4 = m4;
  } else {
    s->M2 += (m2 - s->M2) * CN0_MM_MEAN_MULT;
    s->M4 += (m4 - s->M4) * CN0_MM_MEAN_MULT;
  }

  float tmp = 2.0f * s->M2 * s->M2 - s->M4;
  if (0.0f > tmp) {
    tmp = 0.0f;
  }

  float Pd = sqrtf(tmp);
  float Pn = s->M2 - Pd;
  s->Pn += (Pn - s->Pn) * p->alpha;

  float snr = Pd / s->Pn;

  if (!isfinite(snr) || (snr <= 0.0f)) {
    /* CN0 out of limits, no updates. */
  } else {
    float snr_db = 10.0f * log10f(snr);

    /* Compute CN0 */
    float cn0_dbhz = p->log_bw + snr_db;
    if (cn0_dbhz < 10.0f) {
      cn0_dbhz = 10.0f;
    } else if (cn0_dbhz > 60.0f) {
      cn0_dbhz = 60.0f;
    }
    s->cn0_db = cn0_dbhz;
  }

  /* Increment CN0 smoothing counter with integration period. */
  if (s->cnt_ms < CN0_MM_CNO_SMOOTH_MS) {
    s->cnt_ms += p->t_int;
    /* Limit CN0 smoothing counter to max. */
    if (s->cnt_ms > CN0_MM_CNO_SMOOTH_MS) {
      s->cnt_ms = CN0_MM_CNO_SMOOTH_MS;
    }
  }

  /* CN0 smoothing weight for init value. Goes from 1 to 0. */
  float w1 = (CN0_MM_CNO_SMOOTH_MS - s->cnt_ms) / CN0_MM_CNO_SMOOTH_MS;
  /* CN0 smoothing weight for computed value. Goes from 0 to 1. */
  float w2 = s->cnt_ms / CN0_MM_CNO_SMOOTH_MS;

  /* Smoothed CN0. Start with init CN0, and transition using computed CN0. */
  return s->cn0_init * w1 + s->cn0_db * w2;
}

/** \} */
