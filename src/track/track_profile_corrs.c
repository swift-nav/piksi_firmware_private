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
 * In-place sum of two correlations: A = A + B
 * Note, no check is done for overflow.
 *
 * \param a     First correlation (source and destination)
 * \param[in] b Second correlation (source)
 *
 */
static void corr_add(corr_t *a, const corr_t *b) {
  a->I += b->I;
  a->Q += b->Q;
}

/**
 * In-place inverts correlation value
 *
 * \param a Correlation (source and destination)
 *
 */
static void corr_inv(corr_t *a) {
  a->I = -a->I;
  a->Q = -a->Q;
}

/**
 * In place sums up of correlation array.
 *
 * \param a First correlation (source and destination)
 * \param[in] b Second correlation (source)
 *
 */
static void corr_epl_add(tp_epl_corr_t *a, const tp_epl_corr_t *b) {
  for (u8 i = 0; i < TP_CORR_DATAPILOT_DIM; i++) {
    corr_add(a->all + i, b->all + i);
  }
}

/**
 * In-place inverts correlation array
 *
 * \param  a  Correlation array (source and destination)
 *
 */
static void corr_epl_inv(tp_epl_corr_t *a) {
  for (u8 i = 0; i < TP_CORR_DATAPILOT_DIM; i++) {
    corr_inv(a->all + i);
  }
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
                           const tp_epl_corr_t *cs_now,
                           tp_corr_state_t *corr_state) {
  tp_epl_corr_t straight = (*cs_now);

  /* Correlator accumulators update */
  if (0 != (cycle_flags & TPF_EPL_INV)) {
    corr_epl_inv(&straight);
  }

  if (0 != (cycle_flags & TPF_EPL_SET)) {
    corr_state->corr_main = straight;
  } else if (0 != (cycle_flags & TPF_EPL_ADD)) {
    corr_epl_add(&corr_state->corr_main, &straight);
  }

  /* C/N0 estimator accumulators updates */
  if (0 != (cycle_flags & TPF_CN0_SET)) {
    corr_state->corr_cn0 = straight.prompt;
  } else if (0 != (cycle_flags & TPF_CN0_ADD)) {
    corr_add(&corr_state->corr_cn0, &straight.prompt);
  }

  /* False lock (alias) detector accumulator updates */
  if (0 != (cycle_flags & TPF_ALIAS_SET)) {
    corr_state->corr_ad = straight.prompt;
  } else if (0 != (cycle_flags & TPF_ALIAS_ADD)) {
    corr_add(&corr_state->corr_ad, &straight.prompt);
  }

  /* Lock detector accumulator updates */
  if (0 != (cycle_flags & TPF_PLD_SET)) {
    corr_state->corr_ld = straight.prompt;
  } else if (0 != (cycle_flags & TPF_PLD_ADD)) {
    corr_add(&corr_state->corr_ld, &straight.prompt);
  }

  /* FLL accumulator updates */
  if (0 != (cycle_flags & TPF_FLL_SET)) {
    corr_state->corr_fll = straight.prompt;
  } else if (0 != (cycle_flags & TPF_FLL_ADD)) {
    corr_add(&corr_state->corr_fll, &straight.prompt);
  }

  if (0 != (cycle_flags & TPF_BSYNC_INV)) {
    /* GLO decoder is driven by 10ms meander affected data */
    assert(0 != (cycle_flags & TPF_EPL_INV));
    straight = (*cs_now);
  }

  /* Message payload / bit sync accumulator updates */
  corr_t bit = (0 != (cycle_flags & TPF_BIT_PILOT)) ? straight.dp_prompt
                                                    : straight.prompt;
  if (0 != (cycle_flags & TPF_BSYNC_SET)) {
    corr_state->corr_bit = bit;
  } else if (0 != (cycle_flags & TPF_BSYNC_ADD)) {
    corr_add(&corr_state->corr_bit, &bit);
  }
}
