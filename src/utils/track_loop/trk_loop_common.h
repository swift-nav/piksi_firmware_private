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

#ifndef TRACK_LOOP_H
#define TRACK_LOOP_H

#include <libswiftnav/common.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/linear_algebra.h>
#include <libswiftnav/signal.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** \addtogroup track
 * \{ */

/** \addtogroup track_loop
 * \{ */

/**
 * Tracking loop rates
 */
typedef struct {
  float carr_freq;    /**< Carrier frequency [Hz] */
  float code_freq;    /**< Code frequency [Hz] */
  float acceleration; /**< Acceleration [Hz/s] */
} tl_rates_t;

/**
 * Tracking loop configuration parameters
 */
typedef struct {
  float code_loop_period_s; /**< code loop period [s] */
  float dll_discr_period_s; /**< DLL discriminator period [s] */
  float carr_loop_period_s; /**< carrier loop period [s] */
  float fll_discr_period_s; /**< FLL discriminator period [s] */
  float code_bw;            /**< DLL bandwidth [Hz] */
  float code_zeta;          /**< DLL damping factor (unitless) */
  float code_k;             /**< DLL loop gain (unitless) */
  float carr_to_code;       /**< PLL/DLL output ratio */
  float pll_bw;             /**< PLL bandwidth [Hz] */
  float carr_zeta;          /**< PLL damping factor (unitless) */
  float carr_k;             /**< PLL loop gain (unitless) */
  float fll_bw;             /**< FLL bandwidth [Hz] */
} tl_config_t;

/**
 * FLL-assisted PLL controller.
 *
 * The controller implements third order PLL assisted with second order FLL.
 */
typedef struct {
  float carr_freq; /**< Frequency doppler */
  float code_freq; /**< Code doppler */

  float T_CODE; /**< code integration interval [s] */
  float T_CARR; /**< carrier integration interval [s] */

  float prev_I;        /**< FLL: I[n-1] */
  float prev_Q;        /**< FLL: Q[n-1] */
  float prev_period_s; /**< FLL: Discriminator period [n-1] */
  float fll_discr_sum_hz; /**< FLL: Discriminator sum over coh. int. period [Hz]
                           */
  float fll_discr_period_s; /**< FLL: Discriminator period [s] */
  u8 fll_discr_cnt; /**< FLL: discr_sum is averaged across this many updates */

  float dll_discr_sum_hz; /* DLL discriminator sum over coh. int. period [Hz] */
  float dll_discr_period_s; /**< DLL: Discriminator period [s] */
  u8 dll_discr_cnt; /**< DLL: discr_sum is averaged across this many updates */

  float freq_c1; /**< FLL: c1 coefficient */
  float freq_c2; /**< FLL: c2 coefficient */

  float carr_c1;  /**< PLL: c1 coefficient */
  float carr_c2;  /**< PLL: c2 coefficient */
  float carr_c3;  /**< PLL: c3 coefficient */
  float carr_acc; /**< PLL: y[n-2] */
  float carr_vel; /**< PLL: y[n-1] */

  float code_c1;  /**< DLL: c1 coefficient */
  float code_c2;  /**< DLL: c2 coefficient */
  float code_vel; /**< DLL: y[n-1] */

  float carr_to_code; /**< FLL to DLL assist coefficient */
  float fll_bw_hz;    /**< FLL BW [Hz] */

  float pll_bw_hz; /**< PLL BW [Hz] */

  float freq_error_hz;
} tl_pll3_state_t;

/**
 * PLL controller.
 *
 * The controller implements 2nd order PLL and 1st order DLL
 */
typedef struct {
  float carr_freq_hz; /**< Carrier doppler */
  float code_freq_hz; /**< Code doppler */

  float T_CODE; /**< DLL/PLL integration interval [s] */

  float carr_c1;  /**< PLL: c1 coefficient */
  float carr_c2;  /**< PLL: c2 coefficient */
  float carr_vel; /**< PLL: y[n-1] */

  float dll_discr_sum_hz; /* DLL discriminator sum over coh. int. period [Hz] */
  float dll_discr_period_s; /**< DLL: Discriminator period [s] */
  u8 dll_discr_cnt; /**< DLL: discr_sum is averaged across this many updates */

  float code_c1; /**< DLL: c1 coefficient */

  float carr_to_code; /**< PLL to DLL assist coefficient [unitless] */
} tl_pll2_state_t;

/** \} */

/** Structure representing a complex valued correlation. */
typedef struct {
  float I; /**< In-phase correlation. */
  float Q; /**< Quadrature correlation. */
} correlation_t;

/** \} */

/** Mark unknown satellite elevation with 100 deg to ensure it is be above
 * the elevation mask */
#define SATELLITE_ELEVATION_UNKNOWN 100

/** \} */

void calc_loop_gains(
    float bw, float zeta, float k, float loop_freq, float *b0, float *b1);
void calc_loop_gains2(float bw, float zeta, float k, float *c1, float *c2);
float costas_discriminator(float I, float Q);
float dll_discriminator(const correlation_t cs[3]);

/* FLL2, PLL3, DLL2 */
void tl_pll3_init(tl_pll3_state_t *s,
                  const tl_rates_t *rates,
                  const tl_config_t *config);
void tl_pll3_retune(tl_pll3_state_t *s, const tl_config_t *config);

void tl_pll3_update_dll_discr(tl_pll3_state_t *s, const correlation_t cs[3]);
void tl_pll3_update_dll(tl_pll3_state_t *s);

void tl_pll3_update_fll_discr(tl_pll3_state_t *s, float I, float Q, bool halfq);
void tl_pll3_update_fpll(tl_pll3_state_t *s,
                         const correlation_t cs[3],
                         bool costas);

void tl_pll3_adjust(tl_pll3_state_t *s, float carr_err);
void tl_pll3_get_rates(const tl_pll3_state_t *s, tl_rates_t *rates);
float tl_pll3_get_freq_error(const tl_pll3_state_t *s);

/* PLL2, DLL1 */
void tl_pll2_init(tl_pll2_state_t *s,
                  const tl_rates_t *rates,
                  const tl_config_t *config);
void tl_pll2_retune(tl_pll2_state_t *s, const tl_config_t *config);
void tl_pll2_update_dll(tl_pll2_state_t *s);
void tl_pll2_update_dll_discr(tl_pll2_state_t *s, const correlation_t cs[3]);
void tl_pll2_update_pll(tl_pll2_state_t *s,
                        const correlation_t cs[3],
                        bool costas);
void tl_pll2_adjust(tl_pll2_state_t *s, float err_hz);
void tl_pll2_get_rates(const tl_pll2_state_t *s, tl_rates_t *rates);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* TRACK_LOOP_H */
