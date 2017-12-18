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
#include <libswiftnav/prns.h>
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
 * \param loop_freq The loop update frequency, \f$1/T\f$.
 * \param b0        First filter coefficient, \f$b_0\f$.
 * \param b1        Second filter coefficient, \f$b_1\f$.
 */
void calc_loop_gains(
    float bw, float zeta, float k, float loop_freq, float *b0, float *b1) {
  float T = 1 / loop_freq;
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

/** Frequency discriminator for a FLL, used to aid the PLL.
 *
 * Implements the \f$atan2\f$ frequency discriminator.
 *
 * Strictly speaking this should have a 1 / dt factor, but then we
 * would later multiply the gain by dt again, and it would cancel out.
 * So why do those unnecessary operations at all?
 *
 * \f[
 *    dot_k = abs(I_k * I_{k-1}) + abs(Q_k * Q_{k-1})\\
 *    cross_k = I_{k-1} * Q_k - I_k * Q_{k-1}\\
 *    \varepsilon_k = atan2 \left(cross_k, dot_k\right) / \pi
 * \f]
 *
 * References:
 *  -# Understanding GPS: Principles and Applications.
 *     Elliott D. Kaplan. Artech House, 1996.
 *
 * \param I The prompt in-phase correlation, \f$I_k\f$.
 * \param Q The prompt quadrature correlation, \f$Q_k\f$.
 * \param prev_I The prompt in-phase correlation, \f$I_{k-1}\f$.
 * \param prev_Q The prompt quadrature correlation, \f$Q_{k-1}\f$.
 * \param halfq  Half quadrant discriminator (no bitsync)
 * \return The discriminator value, \f$\varepsilon_k\f$.
 */
float frequency_discriminator(
    float I, float Q, float prev_I, float prev_Q, bool halfq) {
  float angle_semicirc;
  float dot = fabsf(I * prev_I) + fabsf(Q * prev_Q);
  float cross = prev_I * Q - I * prev_Q;
  angle_semicirc = atan2f(cross, dot) / ((float)M_PI);
  if (halfq && (ABS(angle_semicirc) > 0.5f)) {
    angle_semicirc = SIGN(angle_semicirc) * (ABS(angle_semicirc) - 1.0f);
  }
  return angle_semicirc;
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

/**
 * Computes power discriminator for FLL.
 *
 * \param[in] power_epl  Power for minus step, zero, and plus step frequencies.
 * \param[in] fstep      Frequency step between power estimates.
 *
 * \return Computed error in Hz.
 *
 * References:
 * -# A New Proposal for a FLL Discriminator Based on Energy
 *    Xinhua Tang, Emanuela Falletti, Letizia Lo Presti. ION GNSS 2013.
 */
float fll_power_discriminator(const float power_epl[3], float fstep) {
  float fll_err = (power_epl[0] - power_epl[2]) /
                  (power_epl[0] + power_epl[1] + power_epl[2]) * fstep;

  return fll_err;
}

/** Initialize an integral aided loop filter.
 *
 * This initializes a feedback loop with a PI component, plus an extra
 * independent I term.
 *
 * \param s The loop filter state struct to initialize.
 * \param y0 The initial value of the output variable, \f$y_0\f$.
 * \param b0 The proportional gain of the PI error term, \f$b_0\f$.
 * \param b1 The integral gain of the PI error term, \f$b_1\f$.
 * \param aiding_igain The integral gain of the aiding error term, \f$k_{ia}\f$.
 */
void aided_lf_init(
    aided_lf_state_t *s, float y0, float b0, float b1, float aiding_igain) {
  s->y = y0;
  s->prev_error = 0.f;
  s->b0 = b0;
  s->b1 = b1;
  s->aiding_igain = aiding_igain;
}

/** Update step for the integral aided loop filter.
 *
 * This updates a feedback loop with a PI component plus an extra
 * independent I term.
 *
 * \param s The loop filter state struct.
 * \param p_i_error The error output from the discriminator used in both P and I
 * terms.
 * \param aiding_error The error output from the discriminator use just in an I
 * term.
 * \return The updated output variable.
 */
float aided_lf_update(aided_lf_state_t *s,
                      float p_i_error,
                      float aiding_error) {
  s->y += (s->b0 * p_i_error) + (s->b1 * s->prev_error) +
          s->aiding_igain * aiding_error;
  s->prev_error = p_i_error;

  return s->y;
}

/** Initialise a simple first-order loop filter.
 * The gains can be calculated using calc_loop_gains().
 *
 * \param s The loop filter state struct to initialise.
 * \param y0 The initial value of the output variable, \f$y_0\f$.
 * \param b0 First filter coefficient, \f$b_0\f$.
 * \param b1 Second filter coefficient, \f$b_1\f$.
 */
void simple_lf_init(simple_lf_state_t *s, float y0, float b0, float b1) {
  s->y = y0;
  s->prev_error = 0.f;
  s->b0 = b0;
  s->b1 = b1;
}

/** Update step for the simple first-order loop filter.
 *
 * Implements the first-order loop filter as shown below:
 *
 * \image html 1st_order_loop_filter.png Digital loop filter block diagram.
 *
 * with transfer function:
 *
 * \f[
 *   F[z] = \frac{b_0 + b_1 z^{-1}}{1 - z^{-1}}
 * \f]
 *
 * \param s The loop filter state struct.
 * \param error The error output from the discriminator, \f$x_n\f$.
 * \return The updated output variable, \f$y_n\f$.
 */
float simple_lf_update(simple_lf_state_t *s, float error) {
  s->y += (s->b0 * error) + (s->b1 * s->prev_error);
  s->prev_error = error;

  return s->y;
}

/** Initialise an aided tracking loop.
 *
 * For a full description of the loop filter parameters, see calc_loop_gains().
 *
 * \param s The tracking loop state struct to initialise.
 * \param loop_freq The loop update rate.
 * \param code_freq The initial code phase rate (i.e. frequency).
 * \param code_bw The code tracking loop noise bandwidth.
 * \param code_zeta The code tracking loop damping ratio.
 * \param code_k The code tracking loop gain.
 * \param carr_to_code Ratio of carrier to code frequencies,
 *                     or 0 to disable carrier aiding.
 * \param carr_freq The initial carrier frequency.
 * \param carr_bw The carrier tracking loop noise bandwidth.
 * \param carr_zeta The carrier tracking loop damping ratio.
 * \param carr_k The carrier tracking loop gain.
 * \param carr_freq_b1 The integral gain of the aiding error term, \f$k_{ia}\f$.
 */
void aided_tl_init(aided_tl_state_t *s,
                   float loop_freq,
                   float code_freq,
                   float code_bw,
                   float code_zeta,
                   float code_k,
                   float carr_to_code,
                   float carr_freq,
                   float carr_bw,
                   float carr_zeta,
                   float carr_k,
                   float carr_freq_b1) {
  float b0, b1;

  s->carr_freq = carr_freq;
  s->prev_I = 1.0f;  // This works, but is it a really good way to do it?
  s->prev_Q = 0.0f;
  calc_loop_gains(carr_bw, carr_zeta, carr_k, loop_freq, &b0, &b1);
  aided_lf_init(&(s->carr_filt), carr_freq, b0, b1, carr_freq_b1);

  calc_loop_gains(code_bw, code_zeta, code_k, loop_freq, &b0, &b1);
  s->code_freq = code_freq;
  s->carr_to_code = carr_to_code;
  /* If using carrier aiding, initialize code_freq in code loop filter
     to zero to avoid double-counting. */
  simple_lf_init(&(s->code_filt), carr_to_code ? 0 : code_freq, b0, b1);
}

void aided_tl_retune(aided_tl_state_t *s,
                     float loop_freq,
                     float code_bw,
                     float code_zeta,
                     float code_k,
                     float carr_to_code,
                     float carr_bw,
                     float carr_zeta,
                     float carr_k,
                     float carr_freq_b1) {
  calc_loop_gains(carr_bw,
                  carr_zeta,
                  carr_k,
                  loop_freq,
                  &s->carr_filt.b0,
                  &s->carr_filt.b1);
  s->carr_filt.aiding_igain = carr_freq_b1;

  calc_loop_gains(code_bw,
                  code_zeta,
                  code_k,
                  loop_freq,
                  &s->code_filt.b0,
                  &s->code_filt.b1);
  s->carr_to_code = carr_to_code;
}

/** Update step for the aided tracking loop.
 *
 * Implements a basic second-order tracking loop. The code tracking loop is a
 * second-order DLL using dll_discriminator() as its discriminator function.
 * The carrier phase tracking loop is a second-order Costas loop using
 * costas_discriminator(), aided by a frequency discriminator using
 * frequency_discriminator().
 *
 * The tracking loop output variables, i.e. code and carrier frequencies can be
 * read out directly from the state struct.
 *
 * \param s The tracking loop state struct.
 * \param cs An array [E, P, L] of correlation_t structs for the Early, Prompt
 *           and Late correlations.
 * \param halfq         Half quadrant discriminator (no bitsync)
 */
void aided_tl_update(aided_tl_state_t *s, correlation_t cs[3], bool halfq) {
  /* Carrier loop */
  float carr_error = costas_discriminator(cs[1].I, cs[1].Q);
  float freq_error = 0;
  if (s->carr_filt.aiding_igain !=
      0) { /* Don't waste cycles if not using FLL */
    freq_error =
        frequency_discriminator(cs[1].I, cs[1].Q, s->prev_I, s->prev_Q, halfq);
    s->prev_I = cs[1].I;
    s->prev_Q = cs[1].Q;
  }
  s->carr_freq = aided_lf_update(&(s->carr_filt), carr_error, freq_error);

  /* Code loop */
  float code_error = dll_discriminator(cs);
  s->code_freq = simple_lf_update(&(s->code_filt), -code_error);
  if (s->carr_to_code) /* Optional carrier aiding of code loop */
    s->code_freq += s->carr_freq / s->carr_to_code;
}

/**
 * Adjusts FLL/PLL frequency by error.
 *
 * \param[in,out] s   Loop controller
 * \param[in]     err Frequency error in Hz.
 *
 * \return None
 */
void aided_tl_adjust(aided_tl_state_t *s, float err) {
  s->carr_freq += err;
  s->carr_filt.y = s->carr_freq;
}

/**
 * Returns frequency error between DLL and PLL/FLL
 *
 * \param[in] s Loop controller
 *
 * \return Error between DLL and PLL/FLL in chip rate.
 */
float aided_tl_get_dll_error(aided_tl_state_t *s) { return s->code_filt.y; }

/** Initialise a simple tracking loop.
 *
 * For a full description of the loop filter parameters, see calc_loop_gains().
 *
 * \param s The tracking loop state struct to initialise.
 * \param loop_freq The loop update frequency, \f$1/T\f$.
 * \param code_freq The initial code phase rate (i.e. frequency).
 * \param code_bw The code tracking loop noise bandwidth.
 * \param code_zeta The code tracking loop damping ratio.
 * \param code_k The code tracking loop gain.
 * \param carr_freq The initial carrier frequency.
 * \param carr_bw The carrier tracking loop noise bandwidth.
 * \param carr_zeta The carrier tracking loop damping ratio.
 * \param carr_k The carrier tracking loop gain.
 */
void simple_tl_init(simple_tl_state_t *s,
                    float loop_freq,
                    float code_freq,
                    float code_bw,
                    float code_zeta,
                    float code_k,
                    float carr_freq,
                    float carr_bw,
                    float carr_zeta,
                    float carr_k) {
  float b0, b1;

  calc_loop_gains(code_bw, code_zeta, code_k, loop_freq, &b0, &b1);
  s->code_freq = code_freq;
  simple_lf_init(&(s->code_filt), code_freq, b0, b1);

  calc_loop_gains(carr_bw, carr_zeta, carr_k, loop_freq, &b0, &b1);
  s->carr_freq = carr_freq;
  simple_lf_init(&(s->carr_filt), carr_freq, b0, b1);
}

/** Update step for the simple tracking loop.
 *
 * Implements a basic second-order tracking loop. The code tracking loop is a
 * second-order DLL using dll_discriminator() as its discriminator function.
 * The carrier phase tracking loop is a second-order Costas loop using
 * costas_discriminator().
 *
 * The tracking loop output variables, i.e. code and carrier frequencies can be
 * read out directly from the state struct.
 *
 * \param s The tracking loop state struct.
 * \param cs An array [E, P, L] of correlation_t structs for the Early, Prompt
 *           and Late correlations.
 */
void simple_tl_update(simple_tl_state_t *s, correlation_t cs[3]) {
  float code_error = dll_discriminator(cs);
  s->code_freq = simple_lf_update(&(s->code_filt), -code_error);
  float carr_error = costas_discriminator(cs[1].I, cs[1].Q);
  s->carr_freq = simple_lf_update(&(s->carr_filt), carr_error);
}

/** Initialise a code/carrier phase complimentary filter tracking loop.
 *
 * For a full description of the loop filter parameters, see calc_loop_gains().
 *
 * This filter implements gain scheduling. Before `sched` iterations of the
 * loop filter the behaviour will be identical to the simple loop filter. After
 * `sched` iterations, carrier phase information will be used in the code
 * tracking loop.
 *
 * Note, this filter requires that the code and carrier frequencies are
 * expressed as a difference from the nominal frequncy (e.g. 1.023MHz nominal
 * GPS C/A code phase rate, 4.092MHz IF for the carrier).
 *
 * \param s The tracking loop state struct to initialise.
 * \param loop_freq The loop update frequency, \f$1/T\f$.
 * \param code_freq The initial code phase rate (i.e. frequency) difference
 *                  from nominal.
 * \param code_bw The code tracking loop noise bandwidth.
 * \param code_zeta The code tracking loop damping ratio.
 * \param code_k The code tracking loop gain.
 * \param carr_freq The initial carrier frequency difference from nominal, i.e.
 *                  Doppler shift.
 * \param carr_bw The carrier tracking loop noise bandwidth.
 * \param carr_zeta The carrier tracking loop damping ratio.
 * \param carr_k The carrier tracking loop gain.
 * \param tau The complimentary filter cross-over frequency.
 * \param cpc The number of carrier cycles per complete code, or equivalently
 *            the ratio of the carrier frequency to the nominal code frequency.
 * \param sched The gain scheduling count.
 */
void comp_tl_init(comp_tl_state_t *s,
                  float loop_freq,
                  float code_freq,
                  float code_bw,
                  float code_zeta,
                  float code_k,
                  float carr_freq,
                  float carr_bw,
                  float carr_zeta,
                  float carr_k,
                  float tau,
                  float cpc,
                  u32 sched) {
  float b0, b1;

  calc_loop_gains(code_bw, code_zeta, code_k, loop_freq, &b0, &b1);
  s->code_freq = code_freq;
  simple_lf_init(&(s->code_filt), code_freq, b0, b1);

  calc_loop_gains(carr_bw, carr_zeta, carr_k, loop_freq, &b0, &b1);
  s->carr_freq = carr_freq;
  simple_lf_init(&(s->carr_filt), carr_freq, b0, b1);

  s->n = 0;
  s->sched = sched;
  s->carr_to_code = 1.f / cpc;

  s->A = 1.f - (1.f / (loop_freq * tau));
}

/** Update step for a code/carrier phase complimentary filter tracking loop.
 *
 * The tracking loop output variables, i.e. code and carrier frequencies can be
 * read out directly from the state struct.
 *
 * \todo Write proper documentation with math and diagram.
 *
 * \param s The tracking loop state struct.
 * \param cs An array [E, P, L] of correlation_t structs for the Early, Prompt
 *           and Late correlations.
 */
void comp_tl_update(comp_tl_state_t *s, correlation_t cs[3]) {
  float carr_error = costas_discriminator(cs[1].I, cs[1].Q);
  s->carr_freq = simple_lf_update(&(s->carr_filt), carr_error);

  float code_error = dll_discriminator(cs);
  s->code_filt.y = 0.f;
  float code_update = simple_lf_update(&(s->code_filt), -code_error);

  if (s->n > s->sched) {
    s->code_freq = s->A * s->code_freq + s->A * code_update +
                   (1.f - s->A) * s->carr_to_code * s->carr_freq;
  } else {
    s->code_freq += code_update;
  }

  s->n++;
}

/** \} */
/** \} */
