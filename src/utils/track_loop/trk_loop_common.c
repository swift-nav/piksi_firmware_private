/*
 * Copyright (C) 2012, 2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <libswiftnav/constants.h>
#include <libswiftnav/coord_system.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/signal.h>

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "trk_loop_common.h"

/** \defgroup track Tracking
 * Functions used in tracking.
 * \{ */

/** \defgroup track_loop Tracking Loops
 * Functions used by the tracking loops.
 * \{ */

/** Calculate coefficients for a 2nd order digital PLL / DLL loop filter.
 *
 * A second order PLL consists of a phase detector, first-order filter and a
 * Numerically Controlled Oscillator (NCO). A linearised model of a
 * second order PLL is shown below:
 *
 * \image html 2nd_order_dpll.png Linearised second order PLL model.
 *
 * Where \f$K_d\f$ is the discriminator gain and \f$F(s)\f$ and \f$N(s)\f$ are
 * the loop filter and VCO transfer functions. The VCO is essentially an
 * integrator and hence has a transfer function:
 *
 * \f[
 *   N(s) = \frac{K_0}{s}
 * \f]
 *
 * The first-order loop filter is of the form:
 * \f[
 *   F(s) = \frac{s\tau_2 + 1}{s\tau_1}
 * \f]
 * Where \f$\tau_1\f$ and \f$\tau_2\f$ are time constants.
 *
 * The closed loop transfer function for the above PLL is:
 * \f[
 *   H(s) = \frac{k_0\tau_2 s + k_0}
 *               {s^2 + \frac{k_0\tau_2}{\tau_1} s + \frac{k_0}{\tau_1}}
 * \f]
 *
 * Comparing the denominator above to the standard form for a second order
 * system \f$s^2 + 2\omega_n\zeta s + \omega_n^2\f$, we find find the
 * critical frequency and damping ration:
 * \f[
 *   \omega_n = \sqrt{\frac{k_0}{\tau_1}} \\
 *   \zeta = \frac{\omega_n\tau_2}{2}
 * \f]
 *
 * To design our digital implementation of the loop filter, we apply the
 * bilinear transfrom to the loop filter transfer function above
 * \f[
 *   H(z) = \left[\frac{k_0\tau_2 s + k_0}
 *               {s^2 + \frac{k_0\tau_2}{\tau_1} s + \frac{k_0}{\tau_1}}
 *          \right]_{s \leftarrow \frac{2(z-1)}{T(z + 1)}} \\
 *        = \frac{\frac{2\tau_2+T}{2\tau_1} + \frac{T-2\tau_2}{2\tau_1}z^{-1}}
 *               {1 - z^{-1}}
 * \f]
 *
 * The denominator coefficients \f$a_n\f$ are undependent of the loop
 * characteristics.  The numerator coefficients \f$b_n\f$ given by
 * \f[
 *   b_0 = \frac{2\tau_2 + T}{2\tau_1} \\
 *   b_1 = \frac{T - 2\tau_2}{2\tau_1}
 * \f]
 * where
 * \f[
 *   \tau_1 = \frac{k_0}{\omega_n^2} \\
 *   \tau_2 = \frac{2\zeta}{\omega_n}
 * \f]
 *
 * \param bw        The loop noise bandwidth, \f$B_L\f$.
 * \param zeta      The damping ratio, \f$\zeta\f$.
 * \param k         The loop gain, \f$k\f$.
 * \param loop_period_s The loop update period [s], \f$T\f$.
 * \param b0        First filter coefficient, \f$b_0\f$.
 * \param b1        Second filter coefficient, \f$b_1\f$.
 */
void calc_loop_gains(
    float bw, float zeta, float k, float loop_period_s, float *b0, float *b1) {
  float T = loop_period_s;
  /* Find the natural frequency. */
  float omega_n = 8.f * bw * zeta / (4.f * zeta * zeta + 1.f);

  /* Some intermediate values. */
  float tau_1 = k / (omega_n * omega_n);
  float tau_2 = 2.f * zeta / omega_n;

  *b0 = (2.f * tau_2 + T) / (2.f * tau_1);
  *b1 = (T - 2.f * tau_2) / (2.f * tau_1);
}

/** Calculate coefficients for a 2nd order digital PLL / DLL loop filter.
 *
 * References:
 *  -# Understanding GPS: Principles and Applications.
 *     Elliott D. Kaplan. Artech House, 1996.
 *
 * \param[in]  bw   The loop noise bandwidth, \f$B_L\f$.
 * \param[in]  zeta The damping ratio, \f$\zeta\f$.
 * \param[in]  k    The loop gain, \f$k\f$.
 * \param[out] c1   First filter coefficient, \f$c_1\f$.
 * \param[out] c2   Second filter coefficient, \f$c_2\f$.
 */
void calc_loop_gains2(float bw, float zeta, float k, float *c1, float *c2) {
  /* Find the natural frequency. */
  float omega_n = 8.f * zeta * bw / (4.f * zeta * zeta + 1.f);

  *c1 = 2.f * zeta * omega_n / k;
  *c2 = omega_n * omega_n / k;
}

/** Phase discriminator for a Costas loop.
 *
 * \image html costas_loop.png Costas loop block diagram.
 *
 * Implements the \f$\tan^{-1}\f$ Costas loop discriminator.
 *
 * \f[
 *   \varepsilon_k = \tan^{-1} \left(\frac{I_k}{Q_k}\right)
 * \f]
 *
 * References:
 *  -# Understanding GPS: Principles and Applications.
 *     Elliott D. Kaplan. Artech House, 1996.
 *
 * \param I The prompt in-phase correlation, \f$I_k\f$.
 * \param Q The prompt quadrature correlation, \f$Q_k\f$.
 * \return The discriminator value, \f$\varepsilon_k\f$.
 */
float costas_discriminator(float I, float Q) {
  if (I == 0) {
    // Technically, it should be +/- 0.25, but then we'd have to keep track
    //  of the previous sign do it right, so it's simple enough to just return
    //  the average of 0.25 and -0.25 in the face of that ambiguity, so zero.
    return 0;
  }
  return atanf(Q / I) * (float)(1 / (2 * M_PI));
}

/** Normalised non-coherent early-minus-late envelope discriminator.
 *
 * Implements the normalised non-coherent early-minus-late envelope DLL
 * discriminator.
 *
 * \f[
 *   \varepsilon_k = \frac{1}{2} \frac{E - L}{E + L}
 * \f]
 *
 * where:
 *
 * \f[
 *   E = \sqrt{I^2_E + Q^2_E}
 * \f]
 * \f[
 *   L = \sqrt{I^2_L + Q^2_L}
 * \f]
 *
 * References:
 *  -# Understanding GPS: Principles and Applications.
 *     Elliott D. Kaplan. Artech House, 1996.
 *
 * \param cs An array [E, P, L] of correlation_t structs for the Early, Prompt
 *           and Late correlations.
 * \return The discriminator value, \f$\varepsilon_k\f$.
 */
float dll_discriminator(const correlation_t cs[3]) {
  float early_mag = sqrtf(cs[0].I * cs[0].I + cs[0].Q * cs[0].Q);
  float late_mag = sqrtf(cs[2].I * cs[2].I + cs[2].Q * cs[2].Q);
  float early_late_mag_sum = early_mag + late_mag;

  if (0. == early_late_mag_sum) {
    return 0;
  }

  float ret = 0.5f * (early_mag - late_mag) / early_late_mag_sum;
  if (isnormal(ret)) {
    return ret;
  }

  return 0;
}

/** \} */
/** \} */
