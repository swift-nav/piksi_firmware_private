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
 * Helper method to compute filter coefficients.
 *
 * \param[in,out] s      Filter configuration object
 * \param[in]     config Tracking loop configuration parameters
 *
 * \return None
 */
static void update_params(aided_tl_state3_t *s, const tl_config_t *config) {
  /* Common parameters */
  s->T = 1.f / config->dll_loop_freq;

  /* FLL constants */
  if (config->fll_bw > 0.01f) {
    float freq_omega_0 = config->fll_bw / 0.53f;
    float freq_a2 = 2.0f * config->carr_zeta;
    s->freq_c1 = freq_a2 * freq_omega_0 / config->carr_k;
    s->freq_c2 = freq_omega_0 * freq_omega_0 / config->carr_k;
  } else {
    s->freq_c1 = 0.0f;
    s->freq_c2 = 0.0f;
  }

  /* PLL constants */
  float phase_omega_0 = config->carr_bw / 0.7845f;
  float phase_a3 = 1.1f;
  float phase_b3 = 2.4f;

  s->phase_c1 = phase_b3 * phase_omega_0;
  s->phase_c2 = phase_a3 * phase_omega_0 * phase_omega_0;
  s->phase_c3 = phase_omega_0 * phase_omega_0 * phase_omega_0;

  /* DLL constants */
  float code_omega_0 = config->code_bw / 0.53f;
  float code_a2 = 2.0f * config->code_zeta;

  s->code_c1 = code_a2 * code_omega_0 / config->code_k;
  s->code_c2 = code_omega_0 * code_omega_0 / config->code_k;

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
void aided_tl_init3(aided_tl_state3_t *s,
                    const tl_rates_t *rates,
                    const tl_config_t *config) {
  memset(s, 0, sizeof(*s));
  float code_freq = rates->code_freq;
  if (config->carr_to_code != 0) {
    code_freq = 0.0f;
  }

  /* Initial state */
  s->carr_freq = rates->carr_freq;
  s->code_freq = code_freq;

  s->code_vel = code_freq;
  s->phase_acc = rates->acceleration;
  s->phase_vel = rates->carr_freq;

  update_params(s, config);

  s->prev_I = 1;
  s->prev_Q = 0;
}

/**
 * Update parameters of loop controller.
 *
 * \param[in,out] s      Filter configuration object
 * \param[in]     config Tracking loop configuration parameters
 *
 * \return None
 */
void aided_tl_retune3(aided_tl_state3_t *s, const tl_config_t *config) {
  update_params(s, config);
}

/**
 * Updates fll/pll loop filter state
 *
 * \param[in,out] s   FLL/PLL filter configuration object
 *
 * \return None
 */
void aided_tl_update_fll3(aided_tl_state3_t *s) { (void)s; }

/**
 * Updates tracking loop filters state
 *
 * \param[in,out] s      PLL & DLL filter configuration object
 * \param[in]     cs     EPL correlations
 * \param[in]     costas Flag to indicate use of costas discriminator
 *
 * \return None
 */
void aided_tl_update_dll3(aided_tl_state3_t *s,
                          const correlation_t cs[3],
                          bool costas) {
  float phase_error = 0.0f;
  /* Carrier loop */
  if (costas) {
    phase_error = costas_discriminator(cs[1].I, cs[1].Q); /* [cycles] */
  } else if (cs[1].I != 0.0f) {
    /* Otherwise use coherent discriminator */
    phase_error =
        atan2f(cs[1].Q, cs[1].I) * (float)(1 / (2 * M_PI)); /* [cycles] */
  }
  float fll1 = 0.0f;
  float fll2 = 0.0f;
  if (s->freq_c1 > 0.01f) {
    float freq_error = 0.0f;
    freq_error = frequency_discriminator(
                     cs[1].I, cs[1].Q, s->prev_I, s->prev_Q, costas) /
                 (2 * s->T); /* [hz/s] */
    fll1 = s->freq_c1 * freq_error;
    fll2 = s->T * s->freq_c2 * freq_error;
  }
  s->prev_I = cs[1].I;
  s->prev_Q = cs[1].Q;

  s->carr_freq =
      s->phase_c1 * phase_error +
      0.5f * (2.0f * s->phase_vel +
              s->T * (s->phase_c2 * phase_error + fll1 +
                      0.5f * (2.0f * s->phase_acc +
                              s->phase_c3 * s->T * phase_error + fll2)));
  s->phase_vel +=
      s->T *
      (s->phase_c2 * phase_error + fll1 +
       0.5f * (2.0f * s->phase_acc + s->phase_c3 * s->T * phase_error + fll2));
  s->phase_acc += s->phase_c3 * s->T * phase_error + fll2;

  /* Code loop */
  float code_error = -dll_discriminator(cs);
  s->code_freq = s->code_c1 * code_error +
                 0.5f * (2.0f * s->code_vel + s->code_c2 * s->T * code_error) +
                 s->carr_freq * s->carr_to_code;
  s->code_vel += s->code_c2 * s->T * code_error;
}

/**
 * Adjusts FLL/PLL frequency by error.
 *
 * \param[in,out] s   Loop controller
 * \param[in]     err Frequency error [Hz]
 *
 * \return None
 */
void aided_tl_adjust3(aided_tl_state3_t *s, float err) {
  s->carr_freq += err;
  s->phase_vel += err;
  s->code_freq += err * s->carr_to_code;
}

/**
 * Returns frequency error between DLL and PLL/FLL
 *
 * \param[in] s Loop controller
 *
 * \return Error between DLL and PLL/FLL in chip rate.
 */
float aided_tl_get_dll_error3(const aided_tl_state3_t *s) {
  return s->code_freq - s->carr_to_code * s->carr_freq;
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
void aided_tl_discr_update3(aided_tl_state3_t *s,
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
void aided_tl_get_rates3(const aided_tl_state3_t *s, tl_rates_t *rates) {
  memset(rates, 0, sizeof(*rates));

  rates->carr_freq = s->carr_freq;
  rates->code_freq = s->code_freq;
  rates->acceleration = s->phase_acc;
}
