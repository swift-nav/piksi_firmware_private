/*
 * Copyright (C) 2016 Swift Navigation Inc.
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

#include "filter_common.h"

/** \defgroup track Tracking
 * Functions used in tracking.
 * \{ */

/**
 * Computes parameters for the first order LP IIR filter.
 *
 * Initializes low-pass IIR filter with transfer function:
 *
 * \f[
 *    F(s) = \frac{\omega_0}{s + \omega_0}
 * \f]
 *
 */
void lp1_filter_compute_params(lp1_filter_params_t *p,
                               float cutoff_freq,
                               float loop_freq) {
  float Tw0 = (float)(2 * M_PI) * cutoff_freq / loop_freq;
  p->b = Tw0 / (Tw0 + 2.f);
  p->a = (Tw0 - 2.f) / (Tw0 + 2.f);
}

/**
 * Initializes second order Butterworh lowpass IIR filter.
 *
 * Initializes low-pass IIR filter with transfer function:
 *
 * \f[
 *    F(s) = \frac{\omega_0}{s + \omega_0}
 * \f]
 *
 * The bilinear transform is applied to obtain a digital equivalent
 *
 * \f[
 *    F(z) = \frac{b + bz^{-1}}{1 + az^{-1}}
 * \f]
 *
 * where \f$ \omega_c = \frac{2}{T}\tan (2\pi f_{cut}\frac{T}{2})  \f$
 * where \f$ b = \frac{T\omega_0}{T\omega_0 + 2} \f$
 * and \f$ a = \frac{T\omega_0 - 2}{T\omega_0 + 2} \f$.
 *
 * \param f           Filter object
 * \param initial     Initial value for \f$x_{n} and y_{n} \f$
 * \param p           Filter parameters
 *
 * \return None
 */
void lp1_filter_init(lp1_filter_t *f,
                     const lp1_filter_params_t *p,
                     float initial) {
  memset(f, 0, sizeof(*f));

  /* Set up initial filter state so that stable input produces stable output */
  f->yn = initial;
  f->xn = (1 + p->a - p->b) / p->b * initial;
}

/**
 * Feeds new value into filter.
 *
 * Computes a value according to the transfer function:
 *
 * \f[
 *    F(s) = \frac{\omega_0}{s + \omega_0}
 * \f]
 *
 * The bilinear transform is applied to obtain a digital equivalent
 *
 * \f[
 *    F(z) = \frac{b + bz^{-1}}{1 + az^{-1}}
 * \f]
 *
 * \param f     Filter object
 * \param p     Filter parameters
 * \param value Value to filter (\f$ x_{n} \f$)
 *
 * \return Filtered value (\f$ y_{n} \f$)
 */
float lp1_filter_update(lp1_filter_t *f,
                        const lp1_filter_params_t *p,
                        float value) {
  if (!isnan(value)) {
    f->yn = p->b * (value + f->xn) - p->a * f->yn;
    f->xn = value;
  }
  return f->yn;
}

/** \} */
