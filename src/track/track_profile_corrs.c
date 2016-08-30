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

#include "track_profile_utils.h"

/**
 * Sums up two correlations and return a result.
 *
 * \param[in] a First correlation
 * \param[in] b Second correlation
 *
 * \return Correlation, where I and Q are sums of I and Q of arguments.
 */
static inline corr_t corr_add(const corr_t a, const corr_t b)
{
  corr_t res = {.I = a.I + b.I, .Q = a.Q + b.Q};

  return res;
}

/**
 * Inverts correlation
 *
 * \param[in] a Correlation
 *
 * \return Correlation, where I and Q are inverted.
 */
static inline corr_t corr_inv(const corr_t a)
{
  corr_t res = {.I = -a.I, .Q = -a.Q};

  return res;
}
/**
 * Sums up two correlations and return a result.
 *
 * \param[in] a First correlation
 * \param[in] b Second correlation
 * \param[out] res Resulting accumulator
 *
 * \return Resulting accumulator
 */
static inline tp_epl_corr_t *corr_epl_add(const tp_epl_corr_t * restrict a,
                                          const tp_epl_corr_t * restrict b,
                                          tp_epl_corr_t * restrict res)
{
  for (unsigned i = 0; i < TP_DLL_PLL_MEAS_DIM; ++i)
    res->epl[i] = corr_add(a->epl[i], b->epl[i]);

  return res;
}
/**
 * Inverts EPL correlations if the prompt in-phase is negative.
 *
 * \param[in]  a   Input accumulator
 * \param[out] res Resulting accumulator
 *
 * \return Resulting accumulator
 */
static inline tp_epl_corr_t *corr_epl_inv(const tp_epl_corr_t * restrict a,
                                          tp_epl_corr_t * restrict res)
{
  if (a->prompt.I > 0)
    *res = *a;
  else
    for (unsigned i = 0; i < TP_DLL_PLL_MEAS_DIM; ++i)
      res->epl[i] = corr_inv(a->epl[i]);

  return res;
}

/**
 * Processes correlation readings and updates tracking accumulators.
 *
 * The method updates values of accumulators according to supplied operation
 * flags.
 *
 * \param[in]     cycle_flags Operation flags
 * \param[in]     cs_now      Correlation readings
 * \param[in,out] corr_state  Accumulator structure to update.
 *
 * \return None
 */
void tp_update_correlators(u32 cycle_flags,
                           const tp_epl_corr_t *restrict cs_now,
                           tp_corr_state_t *restrict corr_state)
{
  /* C/N0 estimator accumulator updates */
  if (0 != (cycle_flags & TP_CFLAG_CN0_SET))
    corr_state->corr_cn0 = cs_now->prompt;
  else if (0 != (cycle_flags & TP_CFLAG_CN0_ADD))
    corr_state->corr_cn0 = corr_add(corr_state->corr_cn0, cs_now->prompt);

  /* PLL/DLL accumulator updates */
  tp_epl_corr_t tmp_epl;
  if (0 != (cycle_flags & TP_CFLAG_EPL_SET))
    corr_state->corr_epl = *cs_now;
  else if (0 != (cycle_flags & TP_CFLAG_EPL_ADD))
    corr_state->corr_epl = *corr_epl_add(&corr_state->corr_epl, cs_now,
                                         &tmp_epl);
  else if (0 != (cycle_flags & TP_CFLAG_EPL_ADD_INV))
    /* Sum-up and normalize by bit value (for 20+ ms integrations) */
    corr_state->corr_epl = *corr_epl_inv(corr_epl_add(&corr_state->corr_epl,
                                                      cs_now, &tmp_epl),
                                         &tmp_epl);
  else if (0 != (cycle_flags & TP_CFLAG_EPL_INV_ADD))
    /* Normalize by bit value and sum-up (for 20+ ms integrations) */
    corr_state->corr_epl = *corr_epl_add(&corr_state->corr_epl,
                                         corr_epl_inv(cs_now, &tmp_epl),
                                         &tmp_epl);

  /* False lock (alias) detector accumulator updates */
  if (0 != (cycle_flags & TP_CFLAG_ALIAS_SET))
    corr_state->corr_ad = cs_now->prompt;
  else if (0 != (cycle_flags & TP_CFLAG_ALIAS_ADD))
    corr_state->corr_ad = corr_add(corr_state->corr_ad, cs_now->prompt);

  /* Lock detector accumulator updates */
  if (0 != (cycle_flags & TP_CFLAG_LD_SET))
    corr_state->corr_ld = cs_now->prompt;
  else if (0 != (cycle_flags & TP_CFLAG_LD_ADD))
    corr_state->corr_ld = corr_add(corr_state->corr_ld, cs_now->prompt);

  /* FLL accumulator updates */
  if (0 != (cycle_flags & TP_CFLAG_FLL_SET))
    corr_state->corr_fll = cs_now->prompt;
  else if (0 != (cycle_flags & TP_CFLAG_FLL_ADD))
    corr_state->corr_fll = corr_add(corr_state->corr_fll, cs_now->prompt);

}
