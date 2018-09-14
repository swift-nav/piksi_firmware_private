/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Michele Bavaro <michele@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "track_common.h"

/**
 * Sums up two correlations and return a result.
 *
 * \param[in] a First correlation
 * \param[in] b Second correlation
 *
 * \return Correlation, where I and Q are sums of I and Q of arguments.
 */
static inline corr_t corr_add(const corr_t a, const corr_t b) {
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
static inline corr_t corr_inv(const corr_t a) {
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
static inline tp_epl_corr_t *corr_epl_add(const tp_epl_corr_t *restrict a,
                                          const tp_epl_corr_t *restrict b,
                                          tp_epl_corr_t *restrict res) {
  for (unsigned i = 0; i < TP_DLL_PLL_MEAS_DIM; ++i)
    res->five[i] = corr_add(a->five[i], b->five[i]);

  return res;
}
/**
 * Flips EPL accumulator sign.
 *
 * \param[in]  a   Input accumulator
 * \param[out] res Resulting accumulator
 *
 * \return Pointer to resulting accumulator
 */
static inline tp_epl_corr_t *corr_epl_inv(const tp_epl_corr_t *restrict a,
                                          tp_epl_corr_t *restrict res) {
  for (u8 i = 0; i < TP_DLL_PLL_MEAS_DIM; ++i)
    res->five[i] = corr_inv(a->five[i]);

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
                           tp_corr_state_t *restrict corr_state) {
  tp_epl_corr_t tmp_epl;
  tp_epl_corr_t straight;
  tp_epl_corr_t *cs_straight = (tp_epl_corr_t *)&straight;

  /* Correlator accumulators update */
  if (0 != (cycle_flags & TPF_EPL_INV)) {
    corr_epl_inv(cs_now, cs_straight);
  } else {
    *cs_straight = *cs_now;
  }

  if (0 != (cycle_flags & TPF_EPL_SET))
    corr_state->corr_all = *cs_straight;
  else if (0 != (cycle_flags & TPF_EPL_ADD))
    corr_state->corr_all =
        *corr_epl_add(&corr_state->corr_all, cs_straight, &tmp_epl);

  /* C/N0 estimator accumulators updates */
  if (0 != (cycle_flags & TPF_CN0_SET))
    corr_state->corr_cn0 = *cs_straight;
  else if (0 != (cycle_flags & TPF_CN0_ADD))
    corr_state->corr_cn0 =
        *corr_epl_add(&corr_state->corr_cn0, cs_straight, &tmp_epl);

  /* False lock (alias) detector accumulator updates */
  if (0 != (cycle_flags & TPF_ALIAS_SET))
    corr_state->corr_ad = cs_straight->prompt;
  else if (0 != (cycle_flags & TPF_ALIAS_ADD))
    corr_state->corr_ad = corr_add(corr_state->corr_ad, cs_straight->prompt);

  /* Lock detector accumulator updates */
  if (0 != (cycle_flags & TPF_PLD_SET))
    corr_state->corr_ld = cs_straight->prompt;
  else if (0 != (cycle_flags & TPF_PLD_ADD))
    corr_state->corr_ld = corr_add(corr_state->corr_ld, cs_straight->prompt);

  /* FLL accumulator updates */
  if (0 != (cycle_flags & TPF_FLL_SET))
    corr_state->corr_fll = cs_straight->prompt;
  else if (0 != (cycle_flags & TPF_FLL_ADD))
    corr_state->corr_fll = corr_add(corr_state->corr_fll, cs_straight->prompt);

  if (0 != (cycle_flags & TPF_BSYNC_INV)) {
    /* GLO decoder is driven by 10ms meander affected data */
    assert(0 != (cycle_flags & TPF_EPL_INV));
    *cs_straight = *cs_now;
  }

  /* Message payload / bit sync accumulator updates */
  corr_t bit = (0 != (cycle_flags & TPF_BIT_PILOT)) ? cs_straight->very_late
                                                    : cs_straight->prompt;
  if (0 != (cycle_flags & TPF_BSYNC_SET)) {
    corr_state->corr_bit = bit;
  } else if (0 != (cycle_flags & TPF_BSYNC_ADD)) {
    corr_state->corr_bit = corr_add(corr_state->corr_bit, bit);
  }
}
