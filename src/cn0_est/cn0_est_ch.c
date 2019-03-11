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

#define CN0_EST_RSCN_INIT_COUNT 20.f

/** \defgroup track Tracking
 * Functions used in tracking.
 * \{ */

/** Multiplier for checking out-of bounds NSR */
#define CN0_RSCN_NSR_MIN_MULTIPLIER (1e-16f)
/** Maximum supported NSR value (1/CN0_RSCN_NSR_MIN_MULTIPLIER)*/
#define CN0_RSCN_NSR_MIN (1e16f)

/** Initialize the \f$ C / N_0 \f$ estimator state.
 *
 * https://www.computer.org/csdl/proceedings/chinacom/2013/9999/00/06694578.pdf
 *
 * \param s     The estimator state struct to initialize.
 * \param p     Common C/N0 estimator parameters.
 * \param cn0_0 The initial value of \f$ C / N_0 \f$ in dBHz.
 *
 * \return None
 */
void cn0_est_ch_init(cn0_est_ch_state_t *s,
                     const cn0_est_params_t *p,
                     float cn0_0) {
  memset(s, 0, sizeof(*s));

  (void)p;

  /* Normalize by sampling frequency and integration period */
  s->Q_sum = 0.f;
  s->I_sum = 0.f;
  s->N_tot = 0.f;
  s->cn0_db = cn0_0;
  s->cnt = CN0_EST_RSCN_INIT_COUNT;
}

/**
 * Computes \f$ C / N_0 \f$ with Real Signal-Complex Noise method.
 *
 * \param s Initialized estimator object.
 * \param p Common C/N0 estimator parameters.
 * \param I In-phase signal component
 * \param Q Quadrature phase signal component.
 *
 * \return Computed \f$ C / N_0 \f$ value
 */
float cn0_est_ch_update(cn0_est_ch_state_t *s,
                        const cn0_est_params_t *p,
                        float I,
                        float Q) {
  float I_abs = fabsf(I);
  float Q_abs = fabsf(Q);
  float N = (I_abs - Q_abs) * (I_abs - Q_abs);

  if (s->cnt >= 1) {
    s->Q_sum += Q_abs;
    s->I_sum += I_abs;
    s->N_tot += N;
    s->cnt--;
    if (s->cnt == 0) {
      s->Q_sum *= 1 / CN0_EST_RSCN_INIT_COUNT;
      s->I_sum *= 1 / CN0_EST_RSCN_INIT_COUNT;
      s->N_tot *= 1 / CN0_EST_RSCN_INIT_COUNT;
    }
  } else {
    float I_sum = s->I_sum += (I_abs - s->I_sum) * p->alpha;
    float Q_sum = s->Q_sum += (Q_abs - s->Q_sum) * p->alpha;
    float N_tot = s->N_tot += (N - s->N_tot) * p->alpha;

    float P_d = I_sum * I_sum + Q_sum * Q_sum;

    float nsr;
    float nsr_db;

    /* Ensure the NSR is within the limit */
    if (P_d < N_tot * CN0_RSCN_NSR_MIN_MULTIPLIER) {
      nsr = CN0_RSCN_NSR_MIN;
    } else {
      nsr = N_tot / P_d;
    }

    nsr_db = 10.f * log10f(nsr);
    /* Compute and store updated CN0 */
    s->cn0_db = p->log_bw - nsr_db;
  }

  return s->cn0_db;
}

/** \} */
