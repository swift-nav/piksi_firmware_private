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

#include "trk_loop_common.h"

#include <math.h>
#include <string.h>

/**
 * Computes coefficients for third order filter.
 *
 * Third order filter with digital boxcar integrators.
 *
 * \f[
 * H(z)=\frac{
 *            T^{2} \omega^{3} + T a \omega^{2} + b \omega +
 *            \left(b \omega\right) z^{-2} +
 *            \left(- T a \omega^{2} - 2 b \omega\right) z^{-1}
 *           }{(1 - z^{-1})^{2}}
 * \f]
 */
void calc_loop_gains_3_bc(float bw,
                          float zeta,
                          float k,
                          float loop_freq,
                          float *b0,
                          float *b1,
                          float *b2) {
  (void)zeta;

  float T = 1.f / loop_freq;

  float omega_0 = bw / 0.7845f;
  float omega_0_2 = omega_0 * omega_0;
  float omega_0_3 = omega_0_2 * omega_0;
  float a3 = 1.1f;
  float b3 = 2.4f;

  /* DLL constants */
  *b0 = (b3 * omega_0 + T * a3 * omega_0_2 + T * T * omega_0_3) / k;
  *b1 = (-2 * b3 * omega_0 - T * a3 * omega_0_2) / k;
  *b2 = (b3 * omega_0) / k;
}

/**
 * Computes coefficients for third order filter.
 *
 * Third order filter with bilinear transform integrators.
 *
 * \f[
 * H(z)=\frac{
 *   \frac{T^{2} \omega^{3}}{4} + \frac{T a}{2} \omega^{2} + b \omega +
 *   z^{-1} \left(\frac{T^{2} \omega^{3}}{2} - 2 b \omega\right) +
 *   z^{-2} \left(\frac{T^{2} \omega^{3}}{4} - \frac{T a}{2} \omega^{2} +
 *   b \omega\right)
 * }{(1 - z^{-1})^{2}}
 * \f]
 */
void calc_loop_gains_3_bl(float bw,
                          float zeta,
                          float k,
                          float loop_freq,
                          float *b0,
                          float *b1,
                          float *b2) {
  (void)zeta;

  float T_2 = .5f / loop_freq;
  float T_2_2 = T_2 * T_2;

  float omega_0 = bw / 0.7845f;
  float omega_0_2 = omega_0 * omega_0;
  float omega_0_3 = omega_0_2 * omega_0;
  float a3 = 1.1f;
  float b3 = 2.4f;

  /* DLL constants */
  *b0 = (b3 * omega_0 + T_2 * a3 * omega_0_2 + T_2_2 * omega_0_3) / k;
  *b1 = 2.f * (-b3 * omega_0 + T_2_2 * a3 * omega_0_3) / k;
  *b2 = (b3 * omega_0 + T_2 * a3 * omega_0_2 + T_2_2 * omega_0_3) / k;
}

/**
 * Helper method to compute filter coefficients.
 *
 * \param[in,out] s      Filter configuration object
 * \param[in]     config Tracking loop configuration parameters
 *
 * \return None
 */
static void update_params(aided_tl_state3b_t *s, const tl_config_t *config) {
  /* Common parameters */
  s->T = 1.f / config->dll_loop_freq;

  /* FLL constants */
  if (config->fll_bw > 0) {
    calc_loop_gains(config->fll_bw,
                    config->carr_zeta,
                    config->carr_k,
                    config->fll_loop_freq,
                    &s->freq_b0,
                    &s->freq_b1);
  } else {
    s->freq_b0 = s->freq_b1 = 0.f;
  }

  /* PLL constants */
  calc_loop_gains_3_bc(config->carr_bw,
                       config->carr_zeta,
                       config->carr_k,
                       config->dll_loop_freq,
                       &s->phase_b0,
                       &s->phase_b1,
                       &s->phase_b2);

  /* DLL constants */
  calc_loop_gains(config->code_bw,
                  config->code_zeta,
                  config->code_k,
                  config->dll_loop_freq,
                  &s->code_b0,
                  &s->code_b1);

  s->carr_to_code = config->carr_to_code > 0 ? 1.f / config->carr_to_code : 0.f;
}

/**
 * Initialize loop controller.
 *
 * \param[out] s      Filter configuration object
 * \param[in]  rates  Tracking loop rates
 * \param[in]  config Tracking loop configuration parameters
 *
 * \return None
 */
void aided_tl_init3b(aided_tl_state3b_t *s,
                     const tl_rates_t *rates,
                     const tl_config_t *config) {
  memset(s, 0, sizeof(*s));

  /* Initial state */
  s->carr_freq = rates->carr_freq;
  s->code_freq = rates->code_freq;
  s->code_sum = rates->code_freq;
  s->phase_sum_b = rates->carr_freq;
  s->phase_sum_a = rates->acceleration / config->dll_loop_freq;

  s->code_prev = 0;
  s->freq_prev = 0;
  s->phase_prev0 = 0;
  s->phase_prev1 = 0;

  s->prev_I = 1;
  s->prev_Q = 0;

  update_params(s, config);
}

/**
 * Update parameters of loop controller.
 *
 * \param[in,out] s      Filter configuration object
 * \param[in]     config Tracking loop configuration parameters
 *
 * \return None
 */
void aided_tl_retune3b(aided_tl_state3b_t *s, const tl_config_t *config) {
  /* Common parameters */
  float T = 1.f / config->dll_loop_freq;

  if (s->T != T) {
    s->freq_prev = 0;
    s->code_prev = 0;
    s->phase_prev0 = 0;
    s->phase_prev1 = 0;
    s->phase_sum_a *= T / s->T;
  }

  update_params(s, config);

  s->prev_I = 1;
  s->prev_Q = 0;
}

/**
 * Updates fll/pll loop filter state
 *
 * \param[in,out] s   FLL/PLL filter configuration object
 *
 * \return None
 */
void aided_tl_update_fll3b(aided_tl_state3b_t *s) { (void)s; }

/**
 * Updates tracking loop filters state
 *
 * \param[in,out] s    PLL & DLL filter configuration object
 * \param[in]  cs      EPL correlations
 * \param[in]  halfq   Half quadrant discriminator (no bitsync)
 *
 * \return None
 */
void aided_tl_update_dll3b(aided_tl_state3b_t *s,
                           const correlation_t cs[3],
                           bool halfq) {
  /* Carrier loop */
  float phase_error = costas_discriminator(cs[1].I, cs[1].Q);
  float freq_error = 0;
  if (s->freq_b0 != 0) {
    freq_error =
        frequency_discriminator(cs[1].I, cs[1].Q, s->prev_I, s->prev_Q, halfq);
  }

  s->prev_I = cs[1].I;
  s->prev_Q = cs[1].Q;

  float phase_tmp = phase_error * s->phase_b0 + s->phase_prev0 * s->phase_b1 +
                    s->phase_prev1 * s->phase_b2;
  s->phase_sum_a += phase_tmp;
  s->phase_sum_b += s->phase_sum_a;

  s->phase_prev1 = s->phase_prev0;
  s->phase_prev0 = phase_error;

  if (freq_error != 0) {
    float freq_tmp = freq_error * s->freq_b0 + s->freq_prev * s->freq_b1;
    s->freq_prev = freq_error;
    s->phase_sum_b += freq_tmp;
  }

  s->carr_freq = s->phase_sum_b;

  /* Code loop */
  float code_error = -dll_discriminator(cs);

  float code_tmp = code_error * s->code_b0 + s->code_prev * s->code_b1;
  s->code_prev = code_error;
  s->code_sum += code_tmp;

  s->code_freq = s->code_sum;

  if (s->carr_to_code > 0) {
    s->code_freq += s->carr_freq * s->carr_to_code;
  }
}

/**
 * Adjusts FLL/PLL frequency by error.
 *
 * \param[in,out] s   Loop controller
 * \param[in]     err Frequency error [Hz]
 *
 * \return None
 */
void aided_tl_adjust3b(aided_tl_state3b_t *s, float err) {
  s->carr_freq += err;
  s->phase_sum_b += err;
  s->code_sum += err / s->carr_to_code;
  s->code_prev = 0;
}

/**
 * Returns frequency error between DLL and PLL/FLL
 *
 * \param[in] s Loop controller
 *
 * \return Error between DLL and PLL/FLL in chip rate.
 */
float aided_tl_get_dll_error3b(const aided_tl_state3b_t *s) {
  return s->code_sum;
}

/**
 * Updates fll discriminator
 *
 * \param[in,out] s                 FLL filter configuration object
 * \param[in]     I                 Prompt in-phase correlation
 * \param[in]     Q                 Prompt quadrature-phase correlation
 * \param[in]     update_fll_discr  Flag to perform discriminator update
 *
 * \return None
 */
void aided_tl_discr_update3b(aided_tl_state3b_t *s,
                             float I,
                             float Q,
                             bool update_fll_discr) {
  (void)s;
  (void)I;
  (void)Q;
  (void)update_fll_discr;
}

/**
 * Get tracking loop rates
 *
 * \param[in]     s                 Loop controller
 * \param[out]    rates             Tracking loop rates
 *
 * \return None
 */
void aided_tl_get_rates3b(const aided_tl_state3b_t *s, tl_rates_t *rates) {
  memset(rates, 0, sizeof(*rates));

  rates->carr_freq = s->carr_freq;
  rates->code_freq = s->code_freq;
  rates->acceleration = s->phase_sum_a / s->T;
}
