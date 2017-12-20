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

#include <libswiftnav/cnav_msg.h>
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

/** State structure for the I-aided loop filter.
 * Should be initialised with aided_lf_init().
 */
typedef struct {
  float b0;           /**< Filter coefficient. */
  float b1;           /**< Filter coefficient. */
  float aiding_igain; /**< Aiding integral gain. */
  float prev_error;   /**< Previous error. */
  float y;            /**< Output variable. */
} aided_lf_state_t;

/** State structure for the simple loop filter.
 * Should be initialised with simple_lf_init().
 */
typedef struct {
  float b0;         /**< Filter coefficient. */
  float b1;         /**< Filter coefficient. */
  float prev_error; /**< Previous error. */
  float y;          /**< Output variable. */
} simple_lf_state_t;

typedef struct {
  float carr_freq;             /**< Code frequency. */
  aided_lf_state_t carr_filt;  /**< Carrier loop filter state. */
  float code_freq;             /**< Carrier frequenct. */
  simple_lf_state_t code_filt; /**< Code loop filter state. */
  float prev_I, prev_Q;        /**< Previous timestep's in-phase and
                                    quadrature integration, for FLL. */
  float carr_to_code;          /**< Ratio of code to carrier freqs, or
                                    zero to disable carrier aiding */
} aided_tl_state_t;

/**
 * Third order PLL with second order FLL and second order DLL.
 *
 * PLL transfer function:
 * \f[
 * F(z) = z^{-2} \left(\frac{T^{2} \omega^{3}}{4} - \frac{T a}{2} \omega^{2} +
 *                     b \omega\right) +
 *        z^{-1} \left(\frac{T^{2} \omega^{3}}{2} - 2 b \omega\right) +
 *        \frac{T^{2} \omega^{3}}{4} + \frac{T a}{2} \omega^{2} + b \omega
 * \f]
 */
typedef struct {
  float carr_freq; /**< Frequency doppler */
  float code_freq; /**< Code doppler */

  float T; /**< Integration interval */

  float prev_I; /**< FLL: I[n-1] */
  float prev_Q; /**< FLL: Q[n-1] */

  float freq_c1; /**< FLL: c1 coefficient */
  float freq_c2; /**< FLL: c2 coefficient */

  float phase_c1; /**< PLL: c1 coefficient */
  float phase_c2; /**< PLL: c2 coefficient */
  float phase_c3; /**< PLL: c3 coefficient */

  float code_c1; /**< DLL: c1 coefficient */
  float code_c2; /**< DLL: c2 coefficient */

  float phase_acc; /**< PLL: acceleration accumulator */
  float phase_vel; /**< PLL: velocity accumulator */

  float code_vel; /**< DLL: velocity accumulator */

  float carr_to_code; /**< PLL to DLL assist coefficient */
} aided_tl_state3_t;

/**
 * Third order PLL with second order FLL and second order DLL.
 *
 * PLL has transfer function:
 * \f[
 * F(z) = z^{-2} b \omega + z^{-1} \left(- T a \omega^{2} - 2 b \omega\right) +
 *        T^{2} \omega^{3} + T a \omega^{2} + b \omega
 * \f]
 */
typedef struct {
  float carr_freq; /**< Frequency doppler */
  float code_freq; /**< Code doppler */

  float T; /**< Integration interval */

  float prev_I; /**< FLL: I[n-1] */
  float prev_Q; /**< FLL: Q[n-1] */

  float freq_prev; /**< FLL: x[n-1] */
  float freq_b0;   /**< FLL: b0 coefficient */
  float freq_b1;   /**< FLL: b1 coefficient */

  float phase_prev0; /**< PLL: x[n-1] */
  float phase_prev1; /**< PLL: x[n-2] */
  float phase_b0;    /**< PLL: b0 coefficient */
  float phase_b1;    /**< PLL: b1 coefficient */
  float phase_b2;    /**< PLL: b2 coefficient */
  float phase_sum_a; /**< PLL: y[n-2] */
  float phase_sum_b; /**< PLL: y[n-1] */

  float code_prev; /**< DLL: x[n-1] */
  float code_b0;   /**< DLL: b0 coefficient */
  float code_b1;   /**< DLL: b1 coefficient */
  float code_sum;  /**< DLL: y[n-1] */

  float carr_to_code; /**< PLL to DLL assist coefficient */
} aided_tl_state3b_t;

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
  float dll_loop_freq;  /**< DLL loop frequency [Hz] */
  float fll_loop_freq;  /**< FLL loop frequency [Hz] */
  float fll_discr_freq; /**< FLL discriminator frequency [Hz] */
  float code_bw;        /**< DLL bandwidth [Hz] */
  float code_zeta;      /**< DLL damping factor (unitless) */
  float code_k;         /**< DLL loop gain (unitless) */
  float carr_to_code;   /**< PLL/DLL output ratio */
  float carr_bw;        /**< PLL bandwidth [Hz] */
  float carr_zeta;      /**< PLL damping factor (unitless) */
  float carr_k;         /**< PLL loop gain (unitless) */
  float fll_bw;         /**< FLL bandwidth [Hz] */
} tl_config_t;

/**
 * FLL-assisted DLL controller.
 *
 * The controller implements second order DLL assisted with second order FLL.
 */
typedef struct {
  float carr_freq; /**< Frequency doppler */
  float code_freq; /**< Code doppler */

  float T_DLL; /**< DLL Integration interval */
  float T_FLL; /**< FLL Integration interval */

  float prev_I;    /**< FLL: I[n-1] */
  float prev_Q;    /**< FLL: Q[n-1] */
  float discr_sum; /**< FLL: Discriminator sum over coh. int. period */
  float discr_mul; /**< FLL: Discriminator multiplier */

  float freq_prev; /**< FLL: x[n-1] */
  float freq_a0;   /**< FLL: a0 coefficient */
  float freq_a1;   /**< FLL: a1 coefficient */
  float freq_acc;  /**< FLL: y[n-2] */
  float freq_vel;  /**< FLL: y[n-1] */

  float code_prev; /**< DLL: x[n-1] */
  float code_c1;   /**< DLL: c1 coefficient */
  float code_c2;   /**< DLL: c2 coefficient */
  float code_vel;  /**< DLL: y[n-1] */

  float carr_to_code; /**< FLL to DLL assist coefficient */
} aided_tl_state_fll2_t;

/**
 * FLL-assisted DLL controller.
 *
 * The controller implements second order DLL assisted with first order FLL.
 */
typedef struct {
  float carr_freq; /**< Frequency doppler */
  float code_freq; /**< Code doppler */

  float T; /**< Integration interval */

  float prev_I;    /**< FLL: I[n-1] */
  float prev_Q;    /**< FLL: Q[n-1] */
  float discr_sum; /**< FLL: Discriminator sum over coh. int. period */
  float discr_mul; /**< FLL: Discriminator multiplier */

  float freq_a0;  /**< FLL: a0 coefficient */
  float freq_vel; /**< FLL: y[n-1] */

  float code_prev; /**< DLL: x[n-1] */
  float code_c1;   /**< DLL: c1 coefficient */
  float code_c2;   /**< DLL: c2 coefficient */
  float code_vel;  /**< DLL: y[n-1] */

  float carr_to_code; /**< FLL to DLL assist coefficient */
} aided_tl_state_fll1_t;

/**
 * FLL-assisted PLL controller.
 *
 * The controller implements second order PLL assisted with first order FLL.
 */
typedef struct {
  float carr_freq; /**< Frequency doppler */
  float code_freq; /**< Code doppler */

  float T; /**< Integration interval */

  float prev_I;    /**< FLL: I[n-1] */
  float prev_Q;    /**< FLL: Q[n-1] */
  float discr_sum; /**< FLL: Discriminator sum over coh. int. period */
  float discr_mul; /**< FLL: Discriminator multiplier */

  float freq_a0; /**< FLL: a0 coefficient */

  float carr_c1;  /**< PLL: c1 coefficient */
  float carr_c2;  /**< PLL: c2 coefficient */
  float carr_vel; /**< PLL: y[n-1] */

  float code_c1;  /**< DLL: c1 coefficient */
  float code_c2;  /**< DLL: c2 coefficient */
  float code_vel; /**< DLL: y[n-1] */

  float carr_to_code; /**< FLL to DLL assist coefficient */
} aided_tl_state_fll1_pll2_t;

/**
 * FLL-assisted PLL controller.
 *
 * The controller implements third order PLL assisted with second order FLL.
 */
typedef struct {
  float carr_freq; /**< Frequency doppler */
  float code_freq; /**< Code doppler */

  float T_DLL; /**< DLL/PLL Integration interval */
  float T_FLL; /**< FLL Integration interval */

  float prev_I;        /**< FLL: I[n-1] */
  float prev_Q;        /**< FLL: Q[n-1] */
  float prev_period_s; /**< FLL: Discriminator period [n-1] */
  float discr_sum_hz;  /**< FLL: Discriminator sum over coh. int. period [Hz] */
  float discr_period_s; /**< FLL: Discriminator period */
  u8 discr_cnt; /**< FLL: discr_sum is averaged across this many updates */

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
} aided_tl_state_fll2_pll3_t;

/** State structure for a simple tracking loop.
 * Should be initialised with simple_tl_init().
 */
typedef struct {
  float code_freq;             /**< Code phase rate (i.e. frequency). */
  float carr_freq;             /**< Carrier frequency. */
  simple_lf_state_t code_filt; /**< Code loop filter state. */
  simple_lf_state_t carr_filt; /**< Carrier loop filter state. */
} simple_tl_state_t;

/** State structure for a code/carrier phase complimentary filter tracking
 * loop.
 * Should be initialised with comp_tl_init().
 */
typedef struct {
  float code_freq;             /**< Code phase rate (i.e. frequency). */
  float carr_freq;             /**< Carrier frequency. */
  simple_lf_state_t code_filt; /**< Code loop filter state. */
  simple_lf_state_t carr_filt; /**< Carrier loop filter state. */
  u32 sched;                   /**< Gain scheduling count. */
  u32 n;                       /**< Iteration counter. */
  float A;                     /**< Complementary filter crossover gain. */
  float carr_to_code;          /**< Scale factor from carrier to code. */
} comp_tl_state_t;

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
void calc_loop_gains_3_bc(float bw,
                          float zeta,
                          float k,
                          float loop_freq,
                          float *b0,
                          float *b1,
                          float *b2);
void calc_loop_gains_3_bl(float bw,
                          float zeta,
                          float k,
                          float loop_freq,
                          float *b0,
                          float *b1,
                          float *b2);
float costas_discriminator(float I, float Q);
float frequency_discriminator(
    float I, float Q, float prev_I, float prev_Q, bool halfq);
float dll_discriminator(const correlation_t cs[3]);
float fll_power_discriminator(const float power_epl[3], float fstep);

void aided_lf_init(aided_lf_state_t *s,
                   float y0,
                   float pgain,
                   float igain,
                   float aiding_igain);
float aided_lf_update(aided_lf_state_t *s, float p_i_error, float aiding_error);

void simple_lf_init(simple_lf_state_t *s, float y0, float pgain, float igain);
float simple_lf_update(simple_lf_state_t *s, float error);

void simple_tl_init(simple_tl_state_t *s,
                    float loop_freq,
                    float code_freq,
                    float code_bw,
                    float code_zeta,
                    float code_k,
                    float carr_freq,
                    float carr_bw,
                    float carr_zeta,
                    float carr_k);
void simple_tl_update(simple_tl_state_t *s, correlation_t cs[3]);

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
                   float carr_freq_b1);
void aided_tl_retune(aided_tl_state_t *s,
                     float loop_freq,
                     float code_bw,
                     float code_zeta,
                     float code_k,
                     float carr_to_code,
                     float carr_bw,
                     float carr_zeta,
                     float carr_k,
                     float carr_freq_b1);
void aided_tl_update(aided_tl_state_t *s, correlation_t cs[3], bool halfq);
void aided_tl_adjust(aided_tl_state_t *s, float carr_err);
float aided_tl_get_dll_error(aided_tl_state_t *s);

void aided_tl_init3(aided_tl_state3_t *s,
                    const tl_rates_t *rates,
                    const tl_config_t *config);
void aided_tl_retune3(aided_tl_state3_t *s, const tl_config_t *config);
void aided_tl_update_fll3(aided_tl_state3_t *s);
void aided_tl_update_dll3(aided_tl_state3_t *s,
                          const correlation_t cs[3],
                          bool costas);
void aided_tl_adjust3(aided_tl_state3_t *s, float carr_err);
float aided_tl_get_dll_error3(const aided_tl_state3_t *s);
void aided_tl_discr_update3(aided_tl_state3_t *s,
                            float I,
                            float Q,
                            bool update_fll_discr);
void aided_tl_get_rates3(const aided_tl_state3_t *s, tl_rates_t *rates);

void aided_tl_init3b(aided_tl_state3b_t *s,
                     const tl_rates_t *rates,
                     const tl_config_t *config);
void aided_tl_retune3b(aided_tl_state3b_t *s, const tl_config_t *config);
void aided_tl_update_fll3b(aided_tl_state3b_t *s);
void aided_tl_update_dll3b(aided_tl_state3b_t *s,
                           const correlation_t cs[3],
                           bool halfq);
void aided_tl_adjust3b(aided_tl_state3b_t *s, float carr_err);
float aided_tl_get_dll_error3b(const aided_tl_state3b_t *s);
void aided_tl_discr_update3b(aided_tl_state3b_t *s,
                             float I,
                             float Q,
                             bool update_fll_discr);
void aided_tl_get_rates3b(const aided_tl_state3b_t *s, tl_rates_t *rates);

void aided_tl_fll2_init(aided_tl_state_fll2_t *s,
                        const tl_rates_t *rates,
                        const tl_config_t *config);
void aided_tl_fll2_retune(aided_tl_state_fll2_t *s, const tl_config_t *config);
void aided_tl_fll2_update_fll(aided_tl_state_fll2_t *s);
void aided_tl_fll2_update_dll(aided_tl_state_fll2_t *s,
                              const correlation_t cs[3]);
void aided_tl_fll2_update2(aided_tl_state_fll2_t *s,
                           const correlation_t cs[3],
                           float fpower[3],
                           float fstep);
void aided_tl_fll2_adjust(aided_tl_state_fll2_t *s, float carr_err);
float aided_tl_fll2_get_dll_error(const aided_tl_state_fll2_t *s);
void aided_tl_fll2_discr_update(aided_tl_state_fll2_t *s,
                                float I,
                                float Q,
                                bool update_fll_discr,
                                bool halfq);
void aided_tl_fll2_get_rates(const aided_tl_state_fll2_t *s, tl_rates_t *rates);

void aided_tl_fll1_init(aided_tl_state_fll1_t *s,
                        const tl_rates_t *rates,
                        const tl_config_t *config);
void aided_tl_fll1_retune(aided_tl_state_fll1_t *s, const tl_config_t *config);
void aided_tl_fll1_update_fll(aided_tl_state_fll1_t *s);
void aided_tl_fll1_update_dll(aided_tl_state_fll1_t *s,
                              const correlation_t cs[3]);
void aided_tl_fll1_update2(aided_tl_state_fll1_t *s,
                           const correlation_t cs[3],
                           float fpower[3],
                           float fstep);
void aided_tl_fll1_adjust(aided_tl_state_fll1_t *s, float carr_err);
float aided_tl_fll1_get_dll_error(const aided_tl_state_fll1_t *s);
void aided_tl_fll1_discr_update(aided_tl_state_fll1_t *s,
                                float I,
                                float Q,
                                bool update_fll_discr,
                                bool halfq);
void aided_tl_fll1_get_rates(const aided_tl_state_fll1_t *s, tl_rates_t *rates);

void aided_tl_fll1_pll2_init(aided_tl_state_fll1_pll2_t *s,
                             const tl_rates_t *rates,
                             const tl_config_t *config);
void aided_tl_fll1_pll2_retune(aided_tl_state_fll1_pll2_t *s,
                               const tl_config_t *config);
void aided_tl_fll1_pll2_update_fll(aided_tl_state_fll1_pll2_t *s);
void aided_tl_fll1_pll2_update_dll(aided_tl_state_fll1_pll2_t *s,
                                   const correlation_t cs[3]);
void aided_tl_fll1_pll2_adjust(aided_tl_state_fll1_pll2_t *s, float carr_err);
float aided_tl_fll1_pll2_get_dll_error(const aided_tl_state_fll1_pll2_t *s);
void aided_tl_fll1_pll2_discr_update(aided_tl_state_fll1_pll2_t *s,
                                     float I,
                                     float Q,
                                     bool update_fll_discr,
                                     bool halfq);
void aided_tl_fll1_pll2_get_rates(const aided_tl_state_fll1_pll2_t *s,
                                  tl_rates_t *rates);

void aided_tl_fll2_pll3_init(aided_tl_state_fll2_pll3_t *s,
                             const tl_rates_t *rates,
                             const tl_config_t *config);
void aided_tl_fll2_pll3_retune(aided_tl_state_fll2_pll3_t *s,
                               const tl_config_t *config);
void aided_tl_fll2_pll3_update_fll(aided_tl_state_fll2_pll3_t *s);
void aided_tl_fll2_pll3_update_dll(aided_tl_state_fll2_pll3_t *s,
                                   const correlation_t cs[3],
                                   bool costas);
void aided_tl_fll2_pll3_adjust(aided_tl_state_fll2_pll3_t *s, float carr_err);
float aided_tl_fll2_pll3_get_dll_error(const aided_tl_state_fll2_pll3_t *s);
void aided_tl_fll2_pll3_discr_update(aided_tl_state_fll2_pll3_t *s,
                                     float I,
                                     float Q,
                                     bool update_fll_discr,
                                     bool halfq);
void aided_tl_fll2_pll3_get_rates(const aided_tl_state_fll2_pll3_t *s,
                                  tl_rates_t *rates);

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
                  u32 sched);
void comp_tl_update(comp_tl_state_t *s, correlation_t cs[3]);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* TRACK_LOOP_H */
