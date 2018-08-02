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

#include <assert.h>
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
static void update_params(tl_pll3_state_t *s, const tl_config_t *config) {
  s->T_FLL = 1.f / config->fll_loop_freq;
  s->T_DLL = 1.f / config->dll_loop_freq;
  s->fll_bw_hz = config->fll_bw;

  /** PLL & FLL constants
   *  References: Kaplan
   */
  if (config->fll_bw > 0) {
    float freq_omega_0 = config->fll_bw / 0.53f;
    float freq_a2 = 2.0f * config->carr_zeta;
    s->freq_c1 = freq_a2 * freq_omega_0 / config->carr_k;
    s->freq_c2 = freq_omega_0 * freq_omega_0 / config->carr_k;

    s->discr_period_s = 1.0f / (config->fll_discr_freq);
  } else {
    s->freq_c1 = 0;
    s->freq_c2 = 0;
  }
  s->discr_cnt = 0;

  /* PLL constants */
  float omega_0 = config->carr_bw / 0.7845f;
  float omega_0_2 = omega_0 * omega_0;
  float omega_0_3 = omega_0_2 * omega_0;
  float a3 = 1.1f;
  float b3 = 2.4f;

  s->carr_c1 = b3 * omega_0 / config->carr_k;
  s->carr_c2 = a3 * omega_0_2 / config->carr_k;
  s->carr_c3 = omega_0_3 / config->carr_k;

  /* DLL constants */
  calc_loop_gains2(config->code_bw,
                   config->code_zeta,
                   config->code_k,
                   &s->code_c1,
                   &s->code_c2);

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
void tl_pll3_init(tl_pll3_state_t *s,
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
  s->carr_acc = rates->acceleration;
  s->carr_vel = rates->carr_freq;

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
void tl_pll3_retune(tl_pll3_state_t *s, const tl_config_t *config) {
  update_params(s, config);
}

/**
 * Updates fll/pll loop filter state
 *
 * \param[in,out] s FLL/PLL filter configuration object
 *
 * \return None
 */
void tl_pll3_update_fll(tl_pll3_state_t *s) {
  /* This functionality should happen within tl_pll3_update_dll */
  (void)s;
}

/**
 * Updates pll/dll loop filter state
 *
 * \param[in,out] s      FLL/PLL filter configuration object
 * \param[in]     cs     Complex valued epl correlations
 * \param[in]     costas Flag to indicate use of costas discriminator
 *
 * \return None
 */
void tl_pll3_update_dll(tl_pll3_state_t *s,
                        const correlation_t cs[3],
                        bool costas) {
  /* Perform FLL loop update now within this function. */
  float freq_error = 0;
  if ((s->fll_bw_hz > 0) && (0 != s->discr_cnt)) {
    freq_error = (s->discr_sum_hz) / (s->discr_cnt);
    s->discr_sum_hz = 0.f;
    s->discr_cnt = 0;
  }

  /* Carrier loop */
  float carr_error = 0.0f;
  /* Carrier loop */
  if (costas) {
    carr_error = costas_discriminator(cs[1].I, cs[1].Q); /* [cycles] */
  } else if (cs[1].I != 0.0f) {
    /* Otherwise use coherent discriminator */
    carr_error =
        atan2f(cs[1].Q, cs[1].I) * (float)(1 / (2 * M_PI)); /* [cycles] */
  }

  float carr_acc_change =
      s->carr_c3 * s->T_DLL * carr_error + s->freq_c2 * s->T_FLL * freq_error;
  float carr_vel_change =
      s->T_DLL * (s->carr_c2 * carr_error + s->freq_c1 * freq_error +
                  0.5f * (2.0f * s->carr_acc + carr_acc_change));
  s->carr_freq =
      s->carr_c1 * carr_error + 0.5f * (2.0f * s->carr_vel + carr_vel_change);
  s->carr_vel += carr_vel_change;
  s->carr_acc += carr_acc_change;

  /* Code loop */
  float code_error = -dll_discriminator(cs);
  s->code_freq =
      s->code_c1 * code_error +
      0.5f * (2.0f * s->code_vel + s->code_c2 * s->T_DLL * code_error);
  s->code_vel += s->code_c2 * s->T_DLL * code_error;

  /* Carrier aiding */
  s->code_freq += s->carr_freq * s->carr_to_code;
}

/**
 * Adjusts FLL/PLL frequency by error.
 *
 * \param[in,out] s   Loop controller
 * \param[in]     err Frequency error [Hz]
 *
 * \return None
 */
void tl_pll3_adjust(tl_pll3_state_t *s, float err) {
  s->carr_freq += err;
  s->carr_vel += err;
  s->code_freq += err * s->carr_to_code;
}

/**
 * Returns frequency error between DLL and PLL/FLL
 *
 * \param[in] s Loop controller
 *
 * \return Error between DLL and PLL/FLL in chip rate.
 */
float tl_pll3_get_dll_error(const tl_pll3_state_t *s) {
  return s->code_freq - s->carr_to_code * s->carr_freq;
}

/**
 * Updates fll discriminator
 *
 * \param[in,out] s                 FLL filter configuration object
 * \param[in]     I                 Prompt in-phase correlation
 * \param[in]     Q                 Prompt quadrature-phase correlation
 * \param[in]     update_fll_discr  Flag to perform discriminator update
 * \param[in]     halfq             Half quadrant discriminator (no bitsync)
 *
 * \return None
 */
void tl_pll3_discr_update(
    tl_pll3_state_t *s, float I, float Q, bool update_fll_discr, bool halfq) {
  if (s->fll_bw_hz <= 0) {
    /* FLL disabled, skip function all together */
    return;
  }
  if (update_fll_discr && (s->prev_period_s > 0)) {
    /* Skip update if the previous integration period was 0 */
    float dot = I * s->prev_I + Q * s->prev_Q;
    float cross = s->prev_I * Q - I * s->prev_Q;
    float angle_circ = atan2f(cross, dot) / (2.0f * M_PI);
    if (halfq && (ABS(angle_circ) > 0.25f)) {
      angle_circ = SIGN(angle_circ) * (ABS(angle_circ) - 0.5f);
    }
    float mean_period_s = ((s->prev_period_s) + (s->discr_period_s)) / 2.0f;
    s->discr_sum_hz += (angle_circ / mean_period_s);
    s->discr_cnt++;
    assert(0 != s->discr_cnt);
  }
  s->prev_I = I;
  s->prev_Q = Q;
  s->prev_period_s = s->discr_period_s;
}

/**
 * Get tracking loop rates
 *
 * \param[in]  s     Loop controller
 * \param[out] rates Tracking loop rates
 *
 * \return None
 */
void tl_pll3_get_rates(const tl_pll3_state_t *s, tl_rates_t *rates) {
  memset(rates, 0, sizeof(*rates));

  rates->carr_freq = s->carr_freq;
  rates->code_freq = s->code_freq;
  rates->acceleration = s->carr_acc;
}
