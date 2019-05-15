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

#define CN0_EST_NWPR_INIT_M 20
#define CN0_EST_NWPR_INIT_N 100
#define CN0_EST_NWPR_INIT_K (CN0_EST_NWPR_INIT_N / CN0_EST_NWPR_INIT_M)

/** \defgroup track Tracking
 * Functions used in tracking.
 * \{ */

/** Multiplier for checking out-of bounds NSR */
#define CN0_NWPR_NSR_MIN_MULTIPLIER (1e-16f)
/** Maximum supported NSR value (1/CN0_NWPR_NSR_MIN_MULTIPLIER) */
#define CN0_NWPR_NSR_MIN (1e16f)

/** Initialize the \f$ C / N_0 \f$ estimator state.
 *
 * Initializes Narrowband Wideband Power Ratio method \f$ C / N_0 \f$ estimator.
 * https://www.researchgate.net/publication/3007081_GPS_CN0_estimation_in_the_presence_of_interference_and_limited_quantization_levels
 * Pages 3-4.
 * or
 * http://plan.geomatics.ucalgary.ca/papers/ion%20navigation_muthuraman%20et%20al_57-4-2010.pdf
 *
 * \param s     The estimator state struct to initialize.
 * \param p     Common C/N0 estimator parameters.
 * \param cn0_0 The initial value of \f$ C / N_0 \f$ in dBHz.
 *
 * \return None
 */
void cn0_est_nwpr_init(cn0_est_nwpr_state_t *s,
                       const cn0_est_params_t *p,
                       float cn0_0) {
  memset(s, 0, sizeof(*s));

  (void)p;

  s->WBP = 0.f;
  s->NBP_I = 0.f;
  s->NBP_Q = 0.f;
  s->mu = 0.f;
  s->cn0_db = cn0_0;
  s->cnt_m = CN0_EST_NWPR_INIT_M;
  s->cnt_k = CN0_EST_NWPR_INIT_K;
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
float cn0_est_nwpr_update(cn0_est_nwpr_state_t *s,
                          const cn0_est_params_t *p,
                          float I,
                          float Q) {
  (void)p;

  s->WBP += I * I + Q * Q;
  s->NBP_I += I;
  s->NBP_Q += Q;
  s->cnt_m--;

  /* Inner loop ready */
  if (0 == s->cnt_m) {
    float WBP = s->WBP;
    float NBP = s->NBP_I * s->NBP_I + s->NBP_Q * s->NBP_Q;
    s->mu += NBP / WBP;
    s->cnt_k--;

    /* Restart inner loop */
    s->cnt_m = CN0_EST_NWPR_INIT_M;
    s->WBP = 0.f;
    s->NBP_I = 0.f;
    s->NBP_Q = 0.f;
  }

  /* Outer loop ready */
  if (0 == s->cnt_k) {
    float mu = s->mu / CN0_EST_NWPR_INIT_K;

    float P_d = mu - 1;
    float P_n = CN0_EST_NWPR_INIT_M - mu;

    /* Explain why this magic number 35 is needed. Tested against spirent,
       works fine. */
    if (0 != P_n && P_d > 0 && P_n > 0) {
      s->cn0_db = 35 + 10.f * log10f(P_d / P_n);
    }

    /* Restart outer loop */
    s->cnt_k = CN0_EST_NWPR_INIT_K;
    s->mu = 0.f;
  }

  return s->cn0_db;
}

/** \} */
