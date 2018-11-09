/*
 * Copyright (C) 2016 - 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <assert.h>
#include <string.h>

#include "track_common.h"

/**
 * Initializes tracking filter.
 *
 * The method attempts initializes filter state with given parameters.
 *
 * \param[in,out] s      Tracker state to reconfigure.
 * \param[in]     ctrl   Type of new controller.
 * \param[in]     rates  Tracking loop rates
 * \param[in]     config Tracking loop configuration parameters
 *
 * \return None.
 */
void tp_tl_init(tp_tl_state_t *s,
                tp_ctrl_e ctrl,
                const tl_rates_t *rates,
                const tl_config_t *config) {
  /*
   * TODO add logic to initialize internal filter states: velocity and
   *      acceleration.
   */

  memset(s, 0, sizeof(*s));
  s->ctrl = ctrl;

  switch (ctrl) {
    case TP_CTRL_PLL2:
      tl_pll2_init(&s->pll2, rates, config);
      break;

    case TP_CTRL_PLL3:
      tl_pll3_init(&s->pll3, rates, config);
      break;

    default:
      assert(false);
  }
}

/**
 * Reconfigures tracker.
 *
 * The method attempts to reconfigure filter state with new parameters while
 * keeping accumulated states.
 *
 * \param[in,out] s      Tracker state to reconfigure.
 * \param[in]     ctrl   Type of new controller.
 * \param[in]     config Tracking loop configuration parameters
 * \param[in]     aid    Carrier aiding scale factor
 *
 * \return None.
 */
void tp_tl_retune(tp_tl_state_t *s,
                  tp_ctrl_e ctrl,
                  const tl_config_t *config,
                  float aid) {
  if (ctrl == s->ctrl) {
    switch (ctrl) {
      case TP_CTRL_PLL2:
        tl_pll2_retune(&s->pll2, config, aid);
        break;

      case TP_CTRL_PLL3:
        tl_pll3_retune(&s->pll3, config, aid);
        break;

      default:
        assert(false);
    }
  } else {
    /*
     * When the controller type changes, the filter outputs must be preserved.
     */

    tl_rates_t rates = {0};
    tp_tl_get_rates(s, &rates);
    tp_tl_init(s, ctrl, &rates, config);
  }

  s->ctrl = ctrl;
}

/**
 * Adjusts tracker state.
 *
 * The method adjusts PLL/FLL output by the given frequency error.
 *
 * \param[in,out] s   Tracker state.
 * \param[in]     err Correction in Hz.
 *
 * \return None
 */
void tp_tl_adjust(tp_tl_state_t *s, float err) {
  switch (s->ctrl) {
    case TP_CTRL_PLL2:
      tl_pll2_adjust(&s->pll2, err);
      break;

    case TP_CTRL_PLL3:
      tl_pll3_adjust(&s->pll3, err);
      break;

    default:
      assert(false);
  }
}

/**
 * Returns filter output frequencies.
 *
 * \param[in]  s         Tracker state
 * \param[out] rates     Tracking loop rates
 *
 * \return None
 */
void tp_tl_get_rates(const tp_tl_state_t *s, tl_rates_t *rates) {
  switch (s->ctrl) {
    case TP_CTRL_PLL2:
      tl_pll2_get_rates(&s->pll2, rates);
      break;

    case TP_CTRL_PLL3:
      tl_pll3_get_rates(&s->pll3, rates);
      break;

    default:
      assert(false);
  }
}

/**
 * Returns tracking loop configuration parameters.
 *
 * \param[in]  l         Tracking loop parameters
 * \param[out] config    Tracking loop configuration parameters
 *
 * \return None
 */
void tp_tl_get_config(const tp_loop_params_t *l, tl_config_t *config) {
  memset(config, 0, sizeof(*config));

  config->code_bw = l->code_bw;
  config->code_zeta = l->code_zeta;
  config->code_k = l->code_k;
  config->carr_to_code = l->carr_to_code;
  config->pll_bw = l->pll_bw;
  config->carr_zeta = l->carr_zeta;
  config->carr_k = l->carr_k;
  config->fll_bw = l->fll_bw;
}

/**
 * Updates FLL & PLL filters.
 * \param[in, out] s  FLL & PLL filters state.
 * \param[in]      cs EPL correlator outputs.
 * \param[in]      costas Costas (true) or 4 quadrant discriminator (false).
 */
void tp_tl_update_fpll(tp_tl_state_t *s, const tp_epl_corr_t *cs, bool costas) {
  correlation_t cs2[3];
  for (u8 i = 0; i < 3; i++) {
    cs2[i].I = cs->all[i].I;
    cs2[i].Q = cs->all[i].Q;
  }

  switch (s->ctrl) {
    case TP_CTRL_PLL2:
      tl_pll2_update_pll(&s->pll2, cs2, costas);
      break;

    case TP_CTRL_PLL3:
      tl_pll3_update_fpll(&s->pll3, cs2, costas);
      break;

    default:
      assert(false);
  }
}

/**
 * DLL update.
 * \param s Tracker state.
 */
void tp_tl_update_dll(tp_tl_state_t *s) {
  switch (s->ctrl) {
    case TP_CTRL_PLL2:
      tl_pll2_update_dll(&s->pll2);
      break;
    case TP_CTRL_PLL3:
      tl_pll3_update_dll(&s->pll3);
      break;

    default:
      assert(false);
  }
}

/**
 * Get unfiltered FLL discriminator frequency error
 *
 * \param[in] s  Tracker state.
 *
 * \return Raw frequency error [Hz]
 *
 */
float tp_tl_get_fll_error(const tp_tl_state_t *s) {
  float freq_error_hz = 0.0f;

  switch (s->ctrl) {
    case TP_CTRL_PLL2:
      break;

    case TP_CTRL_PLL3:
      freq_error_hz = tl_pll3_get_freq_error(&s->pll3);
      break;

    default:
      assert(false);
  }

  return freq_error_hz;
}

/**
 * DLL discriminator update.
 *
 * \param s  Tracker state.
 * \param cs EPL correlator data.
 */
void tp_tl_update_dll_discr(tp_tl_state_t *s, const tp_epl_corr_t *cs) {
  correlation_t cs2[3];
  for (u32 i = 0; i < 3; i++) {
    cs2[i].I = cs->all[i].I;
    cs2[i].Q = cs->all[i].Q;
  }

  switch (s->ctrl) {
    case TP_CTRL_PLL2:
      tl_pll2_update_dll_discr(&s->pll2, cs2);
      break;

    case TP_CTRL_PLL3:
      tl_pll3_update_dll_discr(&s->pll3, cs2);
      break;

    default:
      assert(false);
  }
}

/**
 * FLL discriminator update.
 *
 * Update discriminator, I_prev & Q_prev.
 *
 * \param[in,out] s  Tracker state.
 * \param[in]     cs EPL correlator outputs.
 * \param[in]     halfq Half quadrant discriminator (no bitsync)
 */
void tp_tl_update_fll_discr(tp_tl_state_t *s, corr_t cs, bool halfq) {
  switch (s->ctrl) {
    case TP_CTRL_PLL2:
      break;

    case TP_CTRL_PLL3:
      tl_pll3_update_fll_discr(&s->pll3, cs.I, cs.Q, halfq);
      break;

    default:
      assert(false);
  }
}
