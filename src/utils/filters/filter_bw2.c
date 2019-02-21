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
 * Computes parameters for Butterworth LP IIR filter.
 *
 * \param p           Parameters object
 * \param cutoff_freq Filter cut-off frequency in Hz
 * \param loop_freq   Loop frequency in Hz
 */
void bw2_filter_compute_params(bw2_filter_params_t *p,
                               float cutoff_freq,
                               float loop_freq) {
  memset(p, 0, sizeof(*p));

  float Tw0 = tanf((float)M_PI * cutoff_freq / loop_freq);
  float tmp = 1.0f + (float)M_SQRT2 * Tw0 + Tw0 * Tw0;

  p->b = Tw0 * Tw0 / tmp;
  p->a2 = (-2.0f + 2.0f * Tw0 * Tw0) / tmp;
  p->a3 = (1.0f - (float)M_SQRT2 * Tw0 + Tw0 * Tw0) / tmp;
}

/**
 * Initializes second order Butterworh lowpass IIR filter.
 *
 * Initializes low-pass IIR filter with transfer function:
 *
 * \f[
 *    F(s) = \frac{\omega_0^2}{s^2 + s\sqrt{2}\omega_0 + \omega_0^2}
 * \f]
 * The bilinear transform is applied to obtain a digital equivalent
 * \f[
 *    F(z) = \frac{b + 2bz^{-1} + bz^{-2}}{1 + a_2z^{-1} + a_3z^{-2}}
 * \f]
 * where \f$ \omega_c = \frac{2}{T}\tan (2\pi f_{cut}\frac{T}{2})  \f$
 * and \f$ b = \frac{\omega_c^2}{1 + \sqrt{2}\omega_c + \omega_c^2} \f$
 * and \f$ a_2 = \frac{-2 + 2\omega_c^2}{1 + \sqrt{2}\omega_c + \omega_c^2} \f$
 * and \f$ a_3 = \frac{1 - \sqrt{2}\omega_c + \omega_c^2}{1 + \sqrt{2}\omega_c +
 * \omega_c^2} \f$.
 *
 * \param f           Filter object
 * \param initial     Initial value for \f$ x_{n}, x_{n-1}, y_{n} and y_{n-1}
 * \f$
 * \param p           Filter parameters
 *
 * \return None
 */
void bw2_filter_init(bw2_filter_t *f,
                     const bw2_filter_params_t *p,
                     float initial) {
  memset(f, 0, sizeof(*f));
  (void)p;

  f->xn = f->xn_prev = initial;
  f->yn = f->yn_prev = initial;
}

/**
 * Feeds new value into filter.
 *
 * Computes a value according to the transfer function:
 *
 * \f[
 *    F(s) = \frac{\omega_0^2}{s^2 + s\sqrt{2}\omega_0 + \omega_0^2}
 * \f]
 * The bilinear transform is applied to obtain a digital equivalent
 * \f[
 *    F(z) = \frac{b + 2bz^{-1} + bz^{-2}}{1 + a_2z^{-1} + a_3z^{-2}}
 * \f]
 * where \f$ \omega_c = \frac{2}{T}\tan (2\pi f_{cut}\frac{T}{2})  \f$
 * and \f$ b = \frac{\omega_c^2}{1 + \sqrt{2}\omega_c + \omega_c^2} \f$
 * and \f$ a_2 = \frac{-2 + 2\omega_c^2}{1 + \sqrt{2}\omega_c + \omega_c^2} \f$
 * and \f$ a_3 = \frac{1 - \sqrt{2}\omega_c + \omega_c^2}{1 + \sqrt{2}\omega_c +
 * \omega_c^2} \f$.
 *
 * \param f     Filter object
 * \param p     Filter parameters
 * \param value Value to filter (\f$ x_{n} \f$)
 *
 * \return Filtered value (\f$ y_{n} \f$)
 */
float bw2_filter_update(bw2_filter_t *f,
                        const bw2_filter_params_t *p,
                        float value) {
  if (!isnan(value)) {
    float yn_prev = f->yn;
    f->yn = p->b * (value + f->xn * 2.f + f->xn_prev) - p->a2 * f->yn -
            p->a3 * f->yn_prev;

    /* Initial state is not too stable, but should be close to that */
    f->yn_prev = yn_prev;
    f->xn_prev = f->xn;
    f->xn = value;
  }

  return f->yn;
}

/** \} */
