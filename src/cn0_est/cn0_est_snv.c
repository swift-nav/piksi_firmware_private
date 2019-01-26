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
#include "signal_db/signal_db.h"

/** \defgroup track Tracking
 * Functions used in tracking.
 * \{ */

/** Multiplier for checking out-of bounds NSR */
#define CN0_SNV_NSR_MIN_MULTIPLIER (1e-16f)
/** Maximum supported NSR value (1/NSR_MIN_MULTIPLIER)*/
#define CN0_SNV_NSR_MIN (1e16f)

/** Initialize the \f$ C / N_0 \f$ estimator state.
 *
 * Initializes Signal-to-Noise Variance method \f$ C / N_0 \f$ estimator.
 *
 * The method uses the function for C/N0 computation:
 *
 * \f[
 *    \frac{C}{N_0}(n) = \frac{P_d}{P_tot-P_d}
 * \f]
 * where
 * \f[
 *    P_d(n) = (\frac{1}{2}(|I(n)|+|I(n-1)|))^2
 * \f]
 * \f[
 *    P_tot(n) = \frac{1}{2}(I(n)^2 + I(n-1)^2 + Q(n)^2 + Q(n-1)^2)
 * \f]
 *
 * \param s     CN0 estimator tacking state.
 * \param p     C/N0 estimator parameters.
 * \param cn0_0 The initial value of \f$ C / N_0 \f$ in dBHz.
 *
 * \return None
 */
void cn0_est_snv_init(cn0_est_snv_state_t *s,
                      const cn0_est_params_t *p,
                      float cn0_0) {
  memset(s, 0, sizeof(*s));

  (void)p;

  /* Normalize by sampling frequency and integration period */
  s->I_sum = 0.f;
  s->P_tot = 0.f;
  s->cn0_db = cn0_0;
  s->cnt = 20;
}

/**
 * Computes \f$ C / N_0 \f$ with Signal-to-Noise Variance method.
 *
 * \param s Initialized estimator object.
 * \param p C/N0 estimator parameters.
 * \param I In-phase signal component
 * \param Q Quadrature phase signal component.
 *
 * \return Computed \f$ C / N_0 \f$ value
 */
float cn0_est_snv_update(cn0_est_snv_state_t *s,
                         const cn0_est_params_t *p,
                         float I,
                         float Q) {
  if (s->cnt >= 1) {
    s->I_sum += fabsf(I);
    s->P_tot += I * I + Q * Q;
    s->cnt--;
    if (s->cnt == 0) {
      s->I_sum *= 0.05f;
      s->P_tot *= 0.05f;
    }
  } else {
    float I_sum = s->I_sum += (fabsf(I) - s->I_sum) * p->alpha;
    float P_tot = s->P_tot += (I * I + Q * Q - s->P_tot) * p->alpha;

    float P_s = I_sum * I_sum;
    float P_n = P_tot - P_s;

    float nsr;
    float nsr_db;

    /* Ensure the NSR is within the limit */
    if (P_s < P_n * CN0_SNV_NSR_MIN_MULTIPLIER)
      nsr = CN0_SNV_NSR_MIN;
    else
      nsr = P_n / P_s;

    nsr_db = 10.f * log10f(nsr);
    /* Compute and store updated CN0 */
    s->cn0_db = p->log_bw - nsr_db;
  }

  return s->cn0_db;
}

/** \} */
