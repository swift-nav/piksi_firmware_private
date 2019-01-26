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

#include <assert.h>
#include <math.h>
#include <string.h>
#include <swiftnav/logging.h>

#include "cn0_est_common.h"

/**
  * The function returns signal power
  * \param I In-phase component
  * \param Q Quadrature component
  * \return Signal power
  */
static float c_pwr(float I, float Q) { return I * I + Q * Q; }

/** Initialize the \f$ C / N_0 \f$ estimator state.
 *
 * Initializes Basic C/N0 \f$ C / N_0 \f$ estimator.
 *
 * \param s     The estimator state struct to initialize.
 * \param cn0_0 The initial value of \f$ C / N_0 \f$ in dBHz.
 *
 * \return None
 */
void cn0_est_basic_init(cn0_est_basic_state_t *s, float cn0_0) {
  memset(s, 0, sizeof(*s));

  s->cn0_db = cn0_0;
}

/**
 * Computes \f$ C / N_0 \f$ with the given noise power estimation
 *
 * \param s Initialized estimator object.
 * \param p Common C/N0 estimator parameters.
 * \param p_I Prompt in-phase accumulator
 * \param p_Q Prompt quadrature accumulator
 * \param n Noise power estimation
 *
 * \return Computed \f$ C / N_0 \f$ value
 */
float cn0_est_basic_update(cn0_est_basic_state_t *s,
                           const cn0_est_params_t *p,
                           float p_I,
                           float p_Q,
                           float n) {
  /* calculate signal power */
  float c = c_pwr(p_I, p_Q) / p->t_int;
  if (c > n) {
    c -= n;
  }
  float snr = c / n;
  float cn0_dbhz = 10.f * log10f(snr) + p->log_bw + p->cn0_shift;
  if (!isfinite(cn0_dbhz)) {
    log_warn(
        "cn0 estimator problem: c %.3e  n %.3e  p_I %.3e  p_Q "
        "%.3e snr %.3e",
        c,
        n,
        p_I,
        p_Q,
        snr);
  }
  s->cn0_db = cn0_dbhz;
  return s->cn0_db;
}
