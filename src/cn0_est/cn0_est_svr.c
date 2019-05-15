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
 * Initializes Signal-to-Variation Ratio method \f$ C / N_0 \f$ estimator.
 *
 * The method uses the function for C/N0 computation:
 *
 * https://github.com/gnuradio/gnuradio/blob/master/gr-digital/lib/mpsk_snr_est.cc#L256
 *
 * \param s     The estimator state struct to initialize.
 * \param p     Common C/N0 estimator parameters.
 * \param cn0_0 The initial value of \f$ C / N_0 \f$ in dBHz.
 *
 * \return None
 */
void cn0_est_svr_init(cn0_est_svr_state_t *s,
                      const cn0_est_params_t *p,
                      float cn0_0) {
  memset(s, 0, sizeof(*s));

  (void)p;

  /* Normalize by sampling frequency and integration period */
  s->I_prev_abs = -1.f;
  s->Q_prev_abs = -1.f;
  s->cn0_db = cn0_0;
}

/**
 * Computes \f$ C / N_0 \f$ with Beaulieu's method.
 *
 * \param s Initialized estimator object.
 * \param p Common C/N0 estimator parameters.
 * \param I In-phase signal component
 * \param Q Quadrature phase signal component.
 *
 * \return Computed \f$ C / N_0 \f$ value
 */
float cn0_est_svr_update(cn0_est_svr_state_t *s,
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
    float P_s;   /* Signal power */
    float P_tot; /* Signal + noise power */
    float S_2 = I_abs * I_abs + Q_abs * Q_abs;
    float S_prev_2 = I_prev_abs * I_prev_abs + Q_prev_abs * Q_prev_abs;

    P_s = s->P_s += (S_2 * S_prev_2 - s->P_s) * p->alpha;
    P_tot = s->P_tot += (S_2 * S_2 - s->P_tot) * p->alpha;

    if (0 == P_s) {
      return s->cn0_db;
    }

    float nsr = (P_tot - P_s) / P_s;

    if (isnan(nsr) || nsr < 0) {
      return s->cn0_db;
    }

    s->cn0_db = p->log_bw - 10.f * log10f(nsr);
  }

  return s->cn0_db;
}

/** \} */
