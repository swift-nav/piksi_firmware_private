/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Valeri Atamaniouk <valeri@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_TRACK_PROFILE_UTILS_H_
#define SWIFTNAV_TRACK_PROFILE_UTILS_H_

#include "track_profiles.h"

#include <nap/nap_common.h>
#include <libswiftnav/track.h>

#define TP_CFLAG_ALIAS_FIRST     ((u32)1 << 0)
#define TP_CFLAG_ALIAS_SECOND    ((u32)1 << 1)
#define TP_CFLAG_USE_CONTROLLER  ((u32)1 << 2)
#define TP_CFLAG_SHORT_CYCLE     ((u32)1 << 3)
#define TP_CFLAG_LONG_CYCLE      ((u32)1 << 4)

#define TP_CFLAG_CN0_SET         ((u32)1 << 5)
#define TP_CFLAG_CN0_ADD         ((u32)1 << 6)
#define TP_CFLAG_CN0_USE         ((u32)1 << 7)

#define TP_CFLAG_FLL_SET         ((u32)1 << 8)
#define TP_CFLAG_FLL_ADD         ((u32)1 << 9)
#define TP_CFLAG_FLL_USE         ((u32)1 << 10)
#define TP_CFLAG_FLL_FIRST       ((u32)1 << 11)
#define TP_CFLAG_FLL_SECOND      ((u32)1 << 12)

#define TP_CFLAG_EPL_SET         ((u32)1 << 13)
#define TP_CFLAG_EPL_ADD         ((u32)1 << 14)
#define TP_CFLAG_EPL_ADD_INV     ((u32)1 << 15)
#define TP_CFLAG_EPL_INV_ADD     ((u32)1 << 16)
#define TP_CFLAG_EPL_USE         ((u32)1 << 17)

#define TP_CFLAG_ALIAS_SET       ((u32)1 << 18)
#define TP_CFLAG_ALIAS_ADD       ((u32)1 << 19)

#define TP_CFLAG_BIT_SYNC_UPDATE ((u32)1 << 20)

#define TP_CFLAG_LD_SET          ((u32)1 << 21)
#define TP_CFLAG_LD_ADD          ((u32)1 << 22)
#define TP_CFLAG_LD_USE          ((u32)1 << 23)

#define TP_DLL_PLL_MEAS_DIM 3

/*
 * Main tracking: PLL loop selection
 */

/*
 * Second order PLL:
 * - Optional first order FLL assist to PLL
 * - Optional PLL assist to DLL
 */
#define tl_pll2_state_t        aided_tl_state_t
#define tl_pll2_init           aided_tl_init
#define tl_pll2_retune         aided_tl_retune
#define tl_pll2_update         aided_tl_update
#define tl_pll2_adjust         aided_tl_adjust
#define tl_pll2_get_dll_error  aided_tl_get_dll_error

#if 0
/* PLL-assisted DLL. FLL and DLL are second order, PLL is third order */
#define tl_pll3_state_t        aided_tl_state3_t
#define tl_pll3_init           aided_tl_init3
#define tl_pll3_retune         aided_tl_retune3
#define tl_pll3_update         aided_tl_update3
#define tl_pll3_adjust         aided_tl_adjust3
#define tl_pll3_get_dll_error  aided_tl_get_dll_error3
#elif 1
/* PLL-assisted DLL. FLL and DLL are second order, PLL is third order */
#define tl_pll3_state_t        aided_tl_state3b_t
#define tl_pll3_init           aided_tl_init3b
#define tl_pll3_retune         aided_tl_retune3b
#define tl_pll3_update         aided_tl_update3b
#define tl_pll3_adjust         aided_tl_adjust3b
#define tl_pll3_get_dll_error  aided_tl_get_dll_error3b
#endif

/*
 * Main tracking: FLL loop selection
 */

/* FLL-assisted DLL. FLL is first order and DLL is second order */
#define tl_fll1_state_t        aided_tl_state_fll1_t
#define tl_fll1_init           aided_tl_fll1_init
#define tl_fll1_retune         aided_tl_fll1_retune
#define tl_fll1_update_fll     aided_tl_fll1_update_fll
#define tl_fll1_update_dll     aided_tl_fll1_update_dll
#define tl_fll1_adjust         aided_tl_fll1_adjust
#define tl_fll1_get_dll_error  aided_tl_fll1_get_dll_error
#define tl_fll1_discr_update   aided_tl_fll1_discr_update

/* FLL-assisted DLL. FLL and DLL are both second order */
#define tl_fll2_state_t        aided_tl_state_fll2_t
#define tl_fll2_init           aided_tl_fll2_init
#define tl_fll2_retune         aided_tl_fll2_retune
#define tl_fll2_update_fll     aided_tl_fll2_update_fll
#define tl_fll2_update_dll     aided_tl_fll2_update_dll
#define tl_fll2_adjust         aided_tl_fll2_adjust
#define tl_fll2_get_dll_error  aided_tl_fll2_get_dll_error
#define tl_fll2_discr_update   aided_tl_fll2_discr_update

/**
 * EPL accumulator structure.
 */
typedef struct
{
  union {
    corr_t epl[TP_DLL_PLL_MEAS_DIM]; /**< EPL accumulators as a vector */
    struct {
      corr_t early;                  /**< Early accumulator */
      corr_t prompt;                 /**< Prompt accumulator */
      corr_t late;                   /**< Late accumulator */
    };
  };
} tp_epl_corr_t;

/**
 * Tracker accumulators.
 *
 * Tracker uses different frequencies for the following algorithms:
 * - DLL and PLL
 * - C/N0 estimator
 * - FLL tracker
 * - Alias (false lock) detector
 * - Lock detector
 */
typedef struct
{
  tp_epl_corr_t    corr_epl; /**< EPL correlation results for DLL and PLL
                              *   in correlation period. */
  corr_t           corr_cn0; /**< C/N0 accumulator */
  corr_t           corr_fll; /**< FLL accumulator */
  corr_t           corr_ad;  /**< False lock (alias) detector accumulator */
  corr_t           corr_ld;  /**< Lock detector accumulator */
} tp_corr_state_t;

/**
 * Tracker loop state.
 */
typedef struct
{
  union {
    tl_pll2_state_t   pll2;          /**< Tracking loop filter state. */
    tl_pll3_state_t   pll3;          /**< Tracking loop filter state. */
    tl_fll1_state_t   fll1;          /**< Tracking loop filter state. */
    tl_fll2_state_t   fll2;          /**< Tracking loop filter state. */
  };
  tp_ctrl_e ctrl;
} tp_tl_state_t;

u8 tp_next_cycle_counter(tp_tm_e tracking_mode,
                         u8 int_ms,
                         u8 cycle_no);

u32 tp_get_cycle_flags(tp_tm_e tracking_mode,
                       u8 int_ms,
                       u8 cycle_no);


u32 tp_compute_cycle_parameters(tp_tm_e tracking_mode,
                                u8 int_ms,
                                u8 cycle_no);

u8 tp_get_cycle_count(tp_tm_e tracking_mode, u8 int_ms);
u8 tp_get_current_cycle_duration(tp_tm_e tracking_mode,
                                 u8 int_ms,
                                 u8 cycle_no);
u32 tp_get_rollover_cycle_duration(tp_tm_e tracking_mode,
                                   u8 int_ms,
                                   u8 cycle_no);
u8 tp_get_cn0_ms(tp_tm_e tracking_mode, u8 int_ms);
u8 tp_get_ld_ms(tp_tm_e tracking_mode, u8 int_ms);
u8 tp_get_alias_ms(tp_tm_e tracking_mode, u8 int_ms);
u8 tp_get_fll_ms(tp_tm_e tracking_mode, u8 int_ms);
u8 tp_get_bit_ms(tp_tm_e tracking_mode, u8 int_ms);


void tp_update_correlators(u32 cycle_flags,
                           const tp_epl_corr_t * restrict cs_now,
                           tp_corr_state_t * restrict corr_state);

void tp_tl_init(tp_tl_state_t *s,
                tp_ctrl_e ctrl,
                float loop_freq,
                float code_freq,
                float code_bw, float code_zeta, float code_k,
                float carr_to_code,
                float carr_freq,
                float carr_bw, float carr_zeta, float carr_k,
                float freq_bw, float fll_loop_freq);

void tp_tl_retune(tp_tl_state_t *s,
                  tp_ctrl_e ctrl,
                  float loop_freq,
                  float code_bw, float code_zeta, float code_k,
                  float carr_to_code,
                  float carr_bw, float carr_zeta, float carr_k,
                  float freq_bw, float fll_loop_freq);

void tp_tl_adjust(tp_tl_state_t *s, float err);
void tp_tl_get_rates(tp_tl_state_t *s, float *carr_freq, float *code_freq);
void tp_tl_update(tp_tl_state_t *s, const tp_epl_corr_t *cs);
float tp_tl_get_dll_error(tp_tl_state_t *s);
bool tp_tl_is_pll(const tp_tl_state_t *s);
bool tp_tl_is_fll(const tp_tl_state_t *s);
void tp_tl_fll_update_first(tp_tl_state_t *s, corr_t cs);
void tp_tl_fll_update_second(tp_tl_state_t *s, corr_t cs);
void tp_tl_fll_update(tp_tl_state_t *s);

#endif /* SWIFTNAV_TRACK_PROFILE_UTILS_H_ */
