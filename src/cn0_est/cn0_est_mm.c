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
#include "swiftnav/constants.h"
#include "swiftnav/logging.h"

/** \defgroup track Tracking
 * Functions used in tracking.
 * \{ */

/** Boxcar window size for M2 and M4. */
#define MM_AVERAGES 20
/** Filter coefficient for Pn. */
#define CN0_MM_PN_ALPHA (0.2f)

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
  memset(s, 0, sizeof(cn0_est_mm_state_t));

  s->cn0_dbhz = cn0_0;
}

/**
 * Initializes C/No filter parameters
 * \param[out] p Pointer to estimator parameters
 * \param[in]  alpha Cutoff frequency
 * \param[in]  loop_freq Loop frequency, Hz
 * \param[in]  scale Scale coefficient for output C/No values
 * \param[in]  cn0_shift shift for output C/No values, dB-Hz
 */
void cn0_est_compute_params(cn0_est_params_t *p, u8 loop_dt_ms) {
  memset(p, 0, sizeof(cn0_est_params_t));
  const float loop_freq = 1.0e3f / loop_dt_ms;

  p->cn0_offset_db = 10.0f * log10f(loop_dt_ms);
  p->log_bw = 10.f * log10f(loop_freq);
  p->t_int_ms = loop_dt_ms;
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
void cn0_est_mm_update(cn0_est_mm_state_t *s,
                       const cn0_est_params_t *p,
                       float I,
                       float Q) {
  assert(p->t_int_ms > 0);
  float m2 = I * I + Q * Q;
  float m4 = m2 * m2;

  if (0 == s->count) {
    /* This is the first iteration, just initialize moments. */
    s->M2 = m2;
    s->M4 = m4;
  } else {
    s->M2 += m2;
    s->M4 += m4;
  }
  s->count++;

  if (MM_AVERAGES > s->count) {
    return;
  }

  s->M2 /= (s->count);
  s->M4 /= (s->count);
  s->count = 0;

  float pow_sq = (2.0f * s->M2 * s->M2) - s->M4;
  if (pow_sq < FLOAT_EQUALITY_EPS) {
    /* signal power square near or below zero isn't going to end well */
    log_debug("s->M2 %.3e  s->M4 %.3e  tmp %.3e", s->M2, s->M4, pow_sq);
    return;
  }

  float Pd = sqrtf(pow_sq);
  float Pn = s->M2 - Pd;
  if (Pn < FLOAT_EQUALITY_EPS) {
    /* noise power near or below zero isn't going to end well */
    log_debug("s->M2 %.3e  tmp %.3e  Pn %.3e", s->M2, pow_sq, Pn);
    return;
  }

  if (s->Pn < FLOAT_EQUALITY_EPS) {
    s->Pn = Pn;
  } else {
    s->Pn += (Pn - s->Pn) * CN0_MM_PN_ALPHA;
  }
  float snr = Pd / s->Pn;
  if (!isnormal(snr)) {
    /* CN0 out of limits, no updates. */
    log_debug("Pd %.3e  Pn %.3e  snr %.3f", Pd, Pn, snr);
    return;
  }

  /* Compute CN0 */
  float snr_db = 10.0f * log10f(snr);
  float cn0_dbhz = p->log_bw + snr_db;
  if (cn0_dbhz < CN0_THRES_MIN) {
    cn0_dbhz = CN0_THRES_MIN;
  } else if (cn0_dbhz > CN0_THRES_MAX) {
    cn0_dbhz = CN0_THRES_MAX;
  }
  s->cn0_dbhz = cn0_dbhz;
}

/** \} */
