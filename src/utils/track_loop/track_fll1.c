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
static void update_params(aided_tl_state_fll1_t *s, const tl_config_t *config) {
  float T_FLL = 1.f / config->fll_loop_freq;

  s->T = 1.f / config->dll_loop_freq;

  /** FLL constants
   *  References:
   *  -# Improving the Design of Frequency Lock Loops for GNSS Receivers.
   *     James Curran, Gérard Lachapelle, Colin Murphy.
   *     IEEE TRANSACTIONS ON AEROSPACE AND ELECTRONIC SYSTEMS
   *     VOL. 48, NO. 1 JANUARY 2012. Eq. 21
   */
  float BWT = config->fll_bw * T_FLL;
  s->freq_a0 = 2.f * BWT / (1.f + BWT) / config->carr_k;
  s->discr_mul = config->fll_discr_freq * (0.5f * (float)M_1_PI);

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
void aided_tl_fll1_init(aided_tl_state_fll1_t *s,
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
  s->freq_vel = rates->carr_freq;

  update_params(s, config);

  s->prev_I = 1.f;
  s->prev_Q = 0.f;
}

/**
 * Update parameters of loop controller.
 *
 * \param[in,out] s      Filter configuration object
 * \param[in]     config Tracking loop configuration parameters
 *
 * \return None
 */
void aided_tl_fll1_retune(aided_tl_state_fll1_t *s, const tl_config_t *config) {
  update_params(s, config);
}

/**
 * Updates fll loop filter state
 *
 * \param[in,out] s FLL/DLL filter configuration object
 *
 * \return None
 */
void aided_tl_fll1_update_fll(aided_tl_state_fll1_t *s) {
  /* Frequency loop */
  float freq_error = s->discr_sum * s->discr_mul;
  s->discr_sum = 0.f;

  s->freq_vel += freq_error * s->freq_a0;
  s->carr_freq = s->freq_vel;
}

/**
 * Updates dll loop filter state
 *
 * \param[in,out] s  FLL/DLL filter configuration object
 * \param[in]     cs EPL correlations for DLL
 *
 * \return None
 */
void aided_tl_fll1_update_dll(aided_tl_state_fll1_t *s,
                              const correlation_t cs[3]) {
  /* Code loop */
  float code_error = -dll_discriminator(cs);
  s->code_vel += s->code_c2 * s->T * code_error;
  s->code_freq = s->code_vel + s->code_c1 * code_error;

  /* Carrier aiding */
  s->code_freq += s->carr_freq * s->carr_to_code;
}

/**
 * Updates loop
 *
 * The method uses traditional EPL DLL discriminator and power discriminator
 * for FLL.
 *
 * \param[in,out] s      Filter configuration object
 * \param[in]     cs     EPL correlations for DLL
 * \param[in]     fpower Frequency step powers (-fstep, 0, +fstep)
 * \param[in]     fstep  Frequency step
 *
 * \return None
 */
void aided_tl_fll1_update2(aided_tl_state_fll1_t *s,
                           const correlation_t cs[3],
                           float fpower[3],
                           float fstep) {
  /* Carrier loop */
  if (fpower) {
    float freq_error = fll_power_discriminator(fpower, fstep);

    s->freq_vel += freq_error * s->freq_a0;
    s->carr_freq = s->freq_vel;
  }

  /* Code loop */
  if (cs) {
    float code_error = -dll_discriminator(cs);
    float code_tmp = code_error * s->code_c1 + s->code_prev * s->code_c2;
    s->code_prev = code_error;
    s->code_vel += code_tmp;
    s->code_freq = s->code_vel;
  }

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
void aided_tl_fll1_adjust(aided_tl_state_fll1_t *s, float err) {
  s->carr_freq += err;
  s->freq_vel += err;
  s->code_freq += err * s->carr_to_code;
}

/**
 * Returns frequency error between DLL and PLL/FLL
 *
 * \param[in] s Loop controller
 *
 * \return Error between DLL and PLL/FLL in chip rate.
 */
float aided_tl_fll1_get_dll_error(const aided_tl_state_fll1_t *s) {
  return s->code_freq - s->carr_to_code * s->carr_freq;
}

/**
 * Updates fll state
 *
 * \param[in,out] s                FLL filter configuration object
 * \param[in]     I                Prompt in-phase correlation
 * \param[in]     Q                Prompt quadrature-phase correlation
 * \param[in]     update_fll_discr Flag to perform discriminator update
 * \param[in]     halfq            Half quadrant discriminator (no bitsync)
 *
 * \return None
 */
void aided_tl_fll1_discr_update(aided_tl_state_fll1_t *s,
                                float I,
                                float Q,
                                bool update_fll_discr,
                                bool halfq) {
  float angle_rad;
  /* Skip first full cycle */
  if (update_fll_discr) {
    float dot = I * s->prev_I + Q * s->prev_Q;
    float cross = s->prev_I * Q - I * s->prev_Q;
    angle_rad = atan2f(cross, dot);
    if (halfq && (ABS(angle_rad) > (M_PI * 0.5))) {
      angle_rad = SIGN(angle_rad) * (ABS(angle_rad) - M_PI);
    }
    s->discr_sum += angle_rad;
  }
  s->prev_I = I;
  s->prev_Q = Q;
}

/**
 * Get tracking loop rates
 *
 * \param[in]  s     FLL filter configuration object
 * \param[out] rates Tracking loop rates
 *
 * \return None
 */
void aided_tl_fll1_get_rates(const aided_tl_state_fll1_t *s,
                             tl_rates_t *rates) {
  memset(rates, 0, sizeof(*rates));

  rates->carr_freq = s->carr_freq;
  rates->code_freq = s->code_freq;
  rates->acceleration = 0;
}
