/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

/* PLL2 + DLL1 tracking loop implementation.
   Based on
   Eliot D. Kaplan "Understanding GPS principles and applications" 1996 */

#include <math.h>
#include <string.h>

#include "trk_loop_common.h"

/**
 * Helper method to compute filter coefficients.
 *
 * \param[in,out] s      Filter configuration object
 * \param[in]     config Tracking loop configuration parameters
 */
static void update_params(tl_pll2_state_t *s, const tl_config_t *config) {
  s->T_DLL = config->dll_loop_period_s;

  /* PLL constants */
  float omega_0 = config->carr_bw / 0.53f;
  float omega_0_2 = omega_0 * omega_0;
  float a2 = 1.414f;

  s->carr_c1 = a2 * omega_0 / config->carr_k;
  s->carr_c2 = omega_0_2 / config->carr_k;

  omega_0 = config->code_bw / 0.25;
  s->code_c1 = omega_0 / config->code_k;

  s->carr_to_code = config->carr_to_code > 0 ? 1.f / config->carr_to_code : 0.f;
}

/**
 * Initialize loop controller.
 *
 * \param[out] s      Filter configuration object
 * \param[in]  rates  Tracking loop rates
 * \param[in]  config Tracking loop configuration parameters
 */
void tl_pll2_init(tl_pll2_state_t *s,
                  const tl_rates_t *rates,
                  const tl_config_t *config) {
  memset(s, 0, sizeof(*s));
  float code_freq_hz = rates->code_freq;
  if (config->carr_to_code != 0) {
    code_freq_hz = 0.0f;
  }

  /* Initial state */
  s->carr_freq_hz = rates->carr_freq;
  s->code_freq_hz = code_freq_hz;

  s->carr_vel = rates->carr_freq;

  update_params(s, config);
}

/**
 * Update parameters of loop controller.
 *
 * \param[in,out] s      Filter configuration object
 * \param[in]     config Tracking loop configuration parameters
 */
void tl_pll2_retune(tl_pll2_state_t *s, const tl_config_t *config) {
  update_params(s, config);
}

/**
 * Updates pll/dll loop filter state
 *
 * \param[in,out] s      The filter configuration object
 * \param[in]     cs     Complex valued epl correlations
 * \param[in]     costas Flag to indicate use of costas discriminator
 */
void tl_pll2_update_dll(tl_pll2_state_t *s,
                        const correlation_t cs[3],
                        bool costas) {
  /* Carrier loop */
  float carr_error_cyc = 0.0f;
  if (costas) {
    carr_error_cyc = costas_discriminator(cs[1].I, cs[1].Q);
  } else if (cs[1].I != 0.0f) {
    /* Otherwise use coherent discriminator */
    carr_error_cyc = atan2f(cs[1].Q, cs[1].I) * (float)(1 / (2 * M_PI));
  }

  float carr_vel_change_hz_per_s = carr_error_cyc * s->carr_c2 * s->T_DLL;
  s->carr_freq_hz = carr_error_cyc * s->carr_c1;
  s->carr_freq_hz += 0.5f * (2.0f * s->carr_vel + carr_vel_change_hz_per_s);
  s->carr_vel += carr_vel_change_hz_per_s;

  /* Code loop */
  float code_error = dll_discriminator(cs);
  s->code_freq_hz = s->code_c1 * code_error;

  /* Carrier aiding */
  s->code_freq_hz += s->carr_freq_hz * s->carr_to_code;
}

/**
 * Adjusts filter frequency error.
 *
 * \param[in,out] s   Loop controller
 * \param[in]     err_hz Frequency error [Hz]
 */
void tl_pll2_adjust(tl_pll2_state_t *s, float err_hz) {
  s->carr_freq_hz += err_hz;
  s->carr_vel += err_hz;
  s->code_freq_hz += err_hz * s->carr_to_code;
}

/**
 * Returns frequency error between DLL and PLL
 *
 * \param[in] s Loop controller
 *
 * \return Error between DLL and PLL in chip rate.
 */
float tl_pll2_get_dll_error(const tl_pll2_state_t *s) {
  return s->code_freq_hz - s->carr_to_code * s->carr_freq_hz;
}

/**
 * Get tracking loop rates
 *
 * \param[in]  s     Loop controller
 * \param[out] rates Tracking loop rates
 */
void tl_pll2_get_rates(const tl_pll2_state_t *s, tl_rates_t *rates) {
  memset(rates, 0, sizeof(*rates));

  rates->carr_freq = s->carr_freq_hz;
  rates->code_freq = s->code_freq_hz;
  rates->acceleration = 0;
}
