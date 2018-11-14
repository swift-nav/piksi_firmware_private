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

/**
 * The function calculates noise power
 * \param s Estimator state
 * \param Q Quadrature component
 * \param p Estimator parameters
 * \return Noise power
 */
static float n0_pwr(cn0_est_basic_state_t *s,
                    float Q,
                    const cn0_est_params_t *p) {
  float Q_abs = s->noise_Q_abs * (1 - p->alpha) + p->alpha * fabsf(Q);

  s->noise_Q_abs = Q_abs;

  s->noise_n0 = Q_abs * Q_abs;

  return s->noise_n0;
}

/** Initialize the \f$ C / N_0 \f$ estimator state.
 *
 * Initializes Basic C/N0 \f$ C / N_0 \f$ estimator.
 *
 * \param s     The estimator state struct to initialize.
 * \param p     Common C/N0 estimator parameters.
 * \param cn0_0 The initial value of \f$ C / N_0 \f$ in dBHz.
 * \param q0    Initial value of noise in Q branch of VE correlator
 *
 * \return None
 */
void cn0_est_basic_init(cn0_est_basic_state_t *s,
                        const cn0_est_params_t *p,
                        float cn0_0,
                        float q0) {
  memset(s, 0, sizeof(*s));

  (void)p;

  s->cn0_db = cn0_0;
  s->noise_Q_abs = q0;
}

/**
 * Computes \f$ C / N_0 \f$ using Very Early tap for noise assess.
 *
 * \param s Initialized estimator object.
 * \param p Common C/N0 estimator parameters.
 * \param p_I Prompt in-phase accumulator
 * \param p_Q Prompt quadrature accumulator
 * \param ve_I Very Early in-phase accumulator (VE correlator)
 * \param ve_Q Very Early quadrature accumulator (VE correlator)
 *
 * \return Computed \f$ C / N_0 \f$ value
 */
float cn0_est_basic_update(cn0_est_basic_state_t *s,
                           const cn0_est_params_t *p,
                           float p_I,
                           float p_Q,
                           float ve_I,
                           float ve_Q) {
  (void)ve_I;
  /* calculate signal power */
  float c = c_pwr(p_I, p_Q);
  /* calculate noise power, use VE tap*/
  float n0 = n0_pwr(s, ve_Q, p);
  float loop_freq = 1000.f / p->t_int;
  float cn0 = c * loop_freq / n0;
  cn0 = p->scale * 10.f * log10f(cn0) + p->cn0_shift;
  if (!isnormal(cn0)) {
    log_warn(
        "cn0 estimator problem: c %.3e  loop_freq %.3e  n0 %.3e  p_I %.3e  p_Q "
        "%.3e  ve_Q %.3e",
        c,
        loop_freq,
        n0,
        p_I,
        p_Q,
        ve_Q);
  }
  s->cn0_db = cn0;
  return s->cn0_db;
}
