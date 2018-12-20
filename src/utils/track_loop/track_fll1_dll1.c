/*
 * Copyright (C) 2019 Swift Navigation Inc.
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
 */
static void update_params(tl_fll1_state_t *s, const tl_config_t *config) {
  s->T_CARR = config->carr_loop_period_s;

  /** FLL constants
   *  References: Kaplan
   */
  assert(config->fll_bw > 0.f);
  float freq_omega_0 = config->fll_bw / 0.53f;
  s->freq_c1 = freq_omega_0 / config->carr_k;

  s->fll_discr_period_s = config->fll_discr_period_s;

  s->fll_discr_sum_hz = 0.f;
  s->fll_discr_cnt = 0;

  s->dll_discr_sum_hz = 0.f;
  s->dll_discr_cnt = 0;

  /* DLL constants */
  assert(config->code_bw > 0);
  s->code_c1 = config->code_bw / 0.25f;

  s->carr_to_code = 0.f;
  if (config->carr_to_code > 0.f) {
    s->carr_to_code = 1.f / config->carr_to_code;
  }
}

/**
 * Initialize loop controller.
 *
 * \param[out] s      Filter configuration object
 * \param[in]  rates  Tracking loop rates
 * \param[in]  config Tracking loop configuration parameters
 */
void tl_fll1_init(tl_fll1_state_t *s,
                  const tl_rates_t *rates,
                  const tl_config_t *config) {
  memset(s, 0, sizeof(*s));
  float code_freq_hz = rates->code_freq;
  if (config->carr_to_code > 0.f) {
    code_freq_hz = 0.f;
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
void tl_fll1_retune(tl_fll1_state_t *s, const tl_config_t *config) {
  update_params(s, config);
}

/**
 * Updates dll loop discriminator state.
 *
 * \param[in,out] s      The filter state
 * \param[in]     cs     Complex valued epl correlations
 */
void tl_fll1_update_dll_discr(tl_fll1_state_t *s, const correlation_t cs[3]) {
  s->dll_discr_sum_hz += dll_discriminator(cs);
  s->dll_discr_cnt++;
  assert(0 != s->dll_discr_cnt);
}

/**
 * Updates dll filter state.
 * \param s Loop state
 */
void tl_fll1_update_dll(tl_fll1_state_t *s) {
  /* Code loop */
  float code_error = 0.f;
  if (s->dll_discr_cnt > 0) {
    code_error = s->dll_discr_sum_hz / s->dll_discr_cnt;
  }
  s->dll_discr_cnt = 0;
  s->dll_discr_sum_hz = 0.f;
  s->code_freq_hz = s->code_c1 * code_error;
}

void tl_fll1_update_fpll(tl_fll1_state_t *s) {
  float freq_error_hz = 0.f;
  if (0 != s->fll_discr_cnt) {
    freq_error_hz = s->fll_discr_sum_hz / s->fll_discr_cnt;
    s->freq_error_hz = freq_error_hz;
    s->fll_discr_sum_hz = 0.f;
    s->fll_discr_cnt = 0;
  }

  float carr_vel_change = s->T_CARR * s->freq_c1 * freq_error_hz;
  s->carr_freq_hz = 0.5f * (2.f * s->carr_vel + carr_vel_change);
  s->carr_vel += carr_vel_change;
}

float tl_fll1_get_freq_error(const tl_fll1_state_t *s) {
  return s->freq_error_hz;
}

/**
 * Adjusts FLL frequency by error.
 *
 * \param[in,out] s   Loop controller
 * \param[in]     err Frequency error [Hz]
 */
void tl_fll1_adjust(tl_fll1_state_t *s, float err) {
  s->carr_freq_hz += err;
  s->carr_vel += err;
}

/**
 * Updates fll discriminator.
 *
 * \param[in,out] s                 FLL filter configuration object
 * \param[in]     I                 Prompt in-phase correlation
 * \param[in]     Q                 Prompt quadrature-phase correlation
 * \param[in]     halfq             Half quadrant discriminator (no bitsync)
 */
void tl_fll1_update_fll_discr(tl_fll1_state_t *s,
                              float I,
                              float Q,
                              bool halfq) {
  if (s->prev_period_s > 0.f) {
    /* Skip update if the previous integration period was 0 */
    float dot = I * s->prev_I + Q * s->prev_Q;
    float cross = s->prev_I * Q - I * s->prev_Q;
    float angle_circ = atan2f(cross, dot) / (2.f * M_PI);
    if (halfq && (ABS(angle_circ) > 0.25f)) {
      angle_circ = SIGN(angle_circ) * (ABS(angle_circ) - 0.5f);
    }
    float mean_period_s = ((s->prev_period_s) + (s->fll_discr_period_s)) / 2.f;
    s->fll_discr_sum_hz += angle_circ / mean_period_s;
    s->fll_discr_cnt++;
    assert(0 != s->fll_discr_cnt);
  }
  s->prev_I = I;
  s->prev_Q = Q;
  s->prev_period_s = s->fll_discr_period_s;
}

/**
 * Get tracking loop rates.
 *
 * \param[in]  s     Loop controller
 * \param[out] rates Tracking loop rates
 */
void tl_fll1_get_rates(const tl_fll1_state_t *s, tl_rates_t *rates) {
  memset(rates, 0, sizeof(*rates));

  rates->carr_freq = s->carr_freq_hz;
  rates->code_freq = s->code_freq_hz + s->carr_freq_hz * s->carr_to_code;
  rates->acceleration = 0.f;
}
