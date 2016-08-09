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

#include <assert.h>
#include <string.h>

void tp_tl_init(tp_tl_state_t *s,
                tp_ctrl_e ctrl,
                float loop_freq,
                float code_freq,
                float code_bw, float code_zeta, float code_k,
                float carr_to_code,
                float carr_freq,
                float carr_bw, float carr_zeta, float carr_k,
                float carr_fll_aid_gain)
{
  memset(s, 0, sizeof(*s));
  s->ctrl = ctrl;

  switch (ctrl) {
  case TP_CTRL_PLL2:
    tl_pll2_init(&s->pll2, loop_freq,
                 code_freq,
                 code_bw, code_zeta, code_k,
                 carr_to_code,
                 carr_freq,
                 carr_bw, carr_zeta, carr_k,
                 carr_fll_aid_gain);
    break;
  case TP_CTRL_PLL3:
    tl_pll3_init(&s->pll3, loop_freq,
                 code_freq,
                 code_bw, code_zeta, code_k,
                 carr_to_code,
                 carr_freq,
                 carr_bw, carr_zeta, carr_k,
                 carr_fll_aid_gain);
    break;
  case TP_CTRL_FLL1:
    tl_fll1_init(&s->fll1, loop_freq,
                 code_freq,
                 code_bw, code_zeta, code_k,
                 carr_to_code,
                 carr_freq,
                 carr_bw, carr_zeta, carr_k,
                 carr_fll_aid_gain);
    break;
  case TP_CTRL_FLL2:
    tl_fll2_init(&s->fll2, loop_freq,
                 code_freq,
                 code_bw, code_zeta, code_k,
                 carr_to_code,
                 carr_freq,
                 carr_bw, carr_zeta, carr_k,
                 carr_fll_aid_gain);
    break;
  default:
    assert(false);
  }
}

void tp_tl_retune(tp_tl_state_t *s,
                  tp_ctrl_e ctrl,
                  float loop_freq,
                  float code_bw, float code_zeta, float code_k,
                  float carr_to_code,
                  float carr_bw, float carr_zeta, float carr_k,
                  float carr_fll_aid_gain)
{
  if (ctrl == s->ctrl) {
    switch (ctrl) {
    case TP_CTRL_PLL2:
      tl_pll2_retune(&s->pll2, loop_freq,
                     code_bw, code_zeta, code_k,
                     carr_to_code,
                     carr_bw, carr_zeta, carr_k,
                     carr_fll_aid_gain);
      break;
    case TP_CTRL_PLL3:
      tl_pll3_retune(&s->pll3, loop_freq,
                     code_bw, code_zeta, code_k,
                     carr_to_code,
                     carr_bw, carr_zeta, carr_k,
                     carr_fll_aid_gain);
      break;
    case TP_CTRL_FLL1:
      tl_fll1_retune(&s->fll1, loop_freq,
                     code_bw, code_zeta, code_k,
                     carr_to_code,
                     carr_bw, carr_zeta, carr_k,
                     carr_fll_aid_gain);
      break;
    case TP_CTRL_FLL2:
      tl_fll2_retune(&s->fll2, loop_freq,
                     code_bw, code_zeta, code_k,
                     carr_to_code,
                     carr_bw, carr_zeta, carr_k,
                     carr_fll_aid_gain);
      break;
    default:
      assert(false);
    }
  } else {
    float code_freq, carr_freq;
    tp_tl_get_rates(s, &carr_freq, &code_freq);
    tp_tl_init(s, ctrl,
               loop_freq,
               code_freq,
               code_bw, code_zeta, code_k,
               carr_to_code,
               carr_freq,
               carr_bw, carr_zeta, carr_k,
               carr_fll_aid_gain);
  }

  s->ctrl = ctrl;

}

void tp_tl_adjust(tp_tl_state_t *s, float err)
{
  switch (s->ctrl) {
  case TP_CTRL_PLL2:
    tl_pll2_adjust(&s->pll2, err);
    break;

  case TP_CTRL_PLL3:
    tl_pll3_adjust(&s->pll3, err);
    break;

  case TP_CTRL_FLL1:
    tl_fll1_adjust(&s->fll1, err);
    break;

  case TP_CTRL_FLL2:
    tl_fll2_adjust(&s->fll2, err);
    break;

  default:
    assert(false);
  }
}

void tp_tl_get_rates(tp_tl_state_t *s, float *carr_freq, float *code_freq)
{
  switch (s->ctrl) {
  case TP_CTRL_PLL2:
    *carr_freq = s->pll2.carr_freq;
    *code_freq = s->pll2.code_freq;
    break;

  case TP_CTRL_PLL3:
    *carr_freq = s->pll3.carr_freq;
    *code_freq = s->pll3.code_freq;
    break;

  case TP_CTRL_FLL1:
    *carr_freq = s->fll1.carr_freq;
    *code_freq = s->fll1.code_freq;
    break;

  case TP_CTRL_FLL2:
    *carr_freq = s->fll2.carr_freq;
    *code_freq = s->fll2.code_freq;
    break;

  default:
    assert(false);
  }
}

void tp_tl_update(tp_tl_state_t *s, const tp_epl_corr_t *cs)
{
  /* TODO: Make this more elegant. */
  correlation_t cs2[3];
  for (u32 i = 0; i < 3; i++) {
    cs2[i].I = cs->epl[2-i].I;
    cs2[i].Q = cs->epl[2-i].Q;
  }

  switch (s->ctrl) {
  case TP_CTRL_PLL2:
    tl_pll2_update(&s->pll2, cs2);
    break;

  case TP_CTRL_PLL3:
    tl_pll3_update(&s->pll3, cs2);
    break;

  case TP_CTRL_FLL1:
    tl_fll1_update(&s->fll1, cs2);
    break;

  case TP_CTRL_FLL2:
    tl_fll2_update(&s->fll2, cs2);
    break;

  default:
    assert(false);
  }
}

float tp_tl_get_dll_error(tp_tl_state_t *s)
{
  float dll_error = 0.;

  switch (s->ctrl) {
  case TP_CTRL_PLL2:
    dll_error = tl_pll2_get_dll_error(&s->pll2);
    break;

  case TP_CTRL_PLL3:
    dll_error = tl_pll3_get_dll_error(&s->pll3);
    break;

  case TP_CTRL_FLL1:
    dll_error = tl_fll1_get_dll_error(&s->fll1);
    break;

  case TP_CTRL_FLL2:
    dll_error = tl_fll2_get_dll_error(&s->fll2);
    break;

  default:
    assert(false);
  }

  return dll_error;
}

bool tp_tl_is_pll(tp_tl_state_t *s)
{
  switch (s->ctrl) {
  case TP_CTRL_PLL2:
  case TP_CTRL_PLL3:
    return true;
  default:
    return false;
  }
}
bool tp_tl_is_fll(tp_tl_state_t *s)
{
  switch (s->ctrl) {
  case TP_CTRL_FLL1:
  case TP_CTRL_FLL2:
    return true;
  default:
    return false;
  }
}
