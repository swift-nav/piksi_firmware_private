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

/**
 * Initializes tracking filter.
 *
 * The method attempts initializes filter state with given parameters.
 *
 * \param[in,out] s              Tracker state to reconfigure.
 * \param[in]     ctrl           Type of new controller.
 * \param[in]     dll_loop_freq  Loop frequency for DLL/PLL (Hz).
 * \param[in]     fll_loop_freq  Loop frequency for FLL (Hz).
 * \param[in]     fll_discr_freq FLL discriminator update rate (Hz).
 * \param[in]     code_freq      DLL initial output frequency (chips/s).
 * \param[in]     carr_freq      PLL(FLL) initial output frequency (Hz).
 * \param[in]     acceleration   PLL(FLL) initial acceleration (Hz/s).
 * \param[in]     code_bw        DLL filter one-sided bandwidth (Hz).
 * \param[in]     code_zeta      DLL filter damping factor (unitless).
 * \param[in]     code_k         DLL filter gain factor (unitless).
 * \param[in]     carr_to_code   Optional coefficient for using PLL/FLL output
 *                               for DLL assistance.
 * \param[in]     carr_bw        PLL filter one-sided bandwidth (Hz).
 * \param[in]     carr_zeta      PLL filter damping factor (unitless).
 * \param[in]     carr_k         PLL filter gain factor (unitless).
 * \param[in]     fll_bw         FLL noise bandwidth (Hz).

 *
 * \return None.
 */
void tp_tl_init(tp_tl_state_t *s,
                tp_ctrl_e ctrl,
                float dll_loop_freq,
                float fll_loop_freq,
                float fll_discr_freq,
                float code_freq,
                float carr_freq,
                float acceleration,
                float code_bw, float code_zeta, float code_k,
                float carr_to_code,
                float carr_bw, float carr_zeta, float carr_k,
                float fll_bw)
{
  /*
   * TODO add logic to initialize internal filter states: velocity and
   *      acceleration.
   */

  memset(s, 0, sizeof(*s));
  s->ctrl = ctrl;

  switch (ctrl) {
  case TP_CTRL_PLL2:
    tl_pll2_init(&s->pll2,
                 dll_loop_freq,
                 fll_loop_freq,
                 fll_discr_freq,
                 code_freq,
                 carr_freq,
                 acceleration,
                 code_bw, code_zeta, code_k,
                 carr_to_code,
                 carr_bw, carr_zeta, carr_k,
                 fll_bw);
    break;

  case TP_CTRL_PLL3:
    tl_pll3_init(&s->pll3,
                 dll_loop_freq,
                 fll_loop_freq,
                 fll_discr_freq,
                 code_freq,
                 carr_freq,
                 acceleration,
                 code_bw, code_zeta, code_k,
                 carr_to_code,
                 carr_bw, carr_zeta, carr_k,
                 fll_bw);
    break;

  case TP_CTRL_FLL1:
    tl_fll1_init(&s->fll1,
                 dll_loop_freq,
                 fll_loop_freq,
                 fll_discr_freq,
                 code_freq,
                 carr_freq,
                 acceleration,
                 code_bw, code_zeta, code_k,
                 carr_to_code,
                 fll_bw, carr_zeta, carr_k);
    break;

  case TP_CTRL_FLL2:
    tl_fll2_init(&s->fll2,
                 dll_loop_freq,
                 fll_loop_freq,
                 fll_discr_freq,
                 code_freq,
                 carr_freq,
                 acceleration,
                 code_bw, code_zeta, code_k,
                 carr_to_code,
                 fll_bw, carr_zeta, carr_k);
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
 * \param[in,out] s              Tracker state to reconfigure.
 * \param[in]     ctrl           Type of new controller.
 * \param[in]     dll_loop_freq  Loop frequency for DLL/PLL (Hz).
 * \param[in]     fll_loop_freq  Loop frequency for FLL (Hz).
 * \param[in]     fll_discr_freq FLL discriminator update rate (Hz).
 * \param[in]     code_bw        DLL filter one-sided bandwidth (Hz).
 * \param[in]     code_zeta      DLL filter damping factor (unitless).
 * \param[in]     code_k         DLL filter gain factor (unitless).
 * \param[in]     carr_to_code   Optional coefficient for using PLL/FLL output
 *                               for DLL assistance.
 * \param[in]     carr_bw        PLL filter one-sided bandwidth (Hz).
 * \param[in]     carr_zeta      PLL filter damping factor (unitless).
 * \param[in]     carr_k         PLL filter gain factor (unitless).
 * \param[in]     fll_bw         FLL coefficient or noise bandwidth (Hz).

 *
 * \return None.
 */
void tp_tl_retune(tp_tl_state_t *s,
                  tp_ctrl_e ctrl,
                  float dll_loop_freq,
                  float fll_loop_freq,
                  float fll_discr_freq,
                  float code_bw, float code_zeta, float code_k,
                  float carr_to_code,
                  float carr_bw, float carr_zeta, float carr_k,
                  float fll_bw)
{
  if (ctrl == s->ctrl) {
    switch (ctrl) {
    case TP_CTRL_PLL2:
      tl_pll2_retune(&s->pll2,
                     dll_loop_freq,
                     fll_loop_freq,
                     fll_discr_freq,
                     code_bw, code_zeta, code_k,
                     carr_to_code,
                     carr_bw, carr_zeta, carr_k,
                     fll_bw);
      break;

    case TP_CTRL_PLL3:
      tl_pll3_retune(&s->pll3,
                     dll_loop_freq,
                     fll_loop_freq,
                     fll_discr_freq,
                     code_bw, code_zeta, code_k,
                     carr_to_code,
                     carr_bw, carr_zeta, carr_k,
                     fll_bw);
      break;

    case TP_CTRL_FLL1:
      tl_fll1_retune(&s->fll1,
                     dll_loop_freq,
                     fll_loop_freq,
                     fll_discr_freq,
                     code_bw, code_zeta, code_k,
                     carr_to_code,
                     fll_bw, carr_zeta, carr_k);
      break;

    case TP_CTRL_FLL2:
      tl_fll2_retune(&s->fll2,
                     dll_loop_freq,
                     fll_loop_freq,
                     fll_discr_freq,
                     code_bw, code_zeta, code_k,
                     carr_to_code,
                     fll_bw, carr_zeta, carr_k);
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
    tp_tl_init(s, ctrl,
               dll_loop_freq,
               fll_loop_freq,
               fll_discr_freq,
               rates.code_freq,
               rates.carr_freq,
               rates.acceleration,
               code_bw, code_zeta, code_k,
               carr_to_code,
               carr_bw, carr_zeta, carr_k,
               fll_bw);
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

/**
 * Returns filter output frequencies.
 *
 * \param[in]  s         Tracker state
 * \param[out] rates     Tracking loop rates
 *
 * \return None
 */
void tp_tl_get_rates(tp_tl_state_t *s, tl_rates_t *rates)
{
  switch (s->ctrl) {
  case TP_CTRL_PLL2:
    tl_pll2_get_rates(&s->pll2, rates);
    break;

  case TP_CTRL_PLL3:
    tl_pll3_get_rates(&s->pll3, rates);
    break;

  case TP_CTRL_FLL1:
    tl_fll1_get_rates(&s->fll1, rates);
    break;

  case TP_CTRL_FLL2:
    tl_fll2_get_rates(&s->fll2, rates);
    break;

  default:
    assert(false);
  }
}

/**
 * Updates tracker filter.
 *
 * The method performs computation of DLL and PLL(FLL) corrections according
 * to input data.
 *
 * \param[in, out] s  Tracker state.
 * \param[in]      cs EPL correlator outputs.
 *
 * \return None
 */
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
    tl_pll2_update_dll(&s->pll2, cs2);
    break;

  case TP_CTRL_PLL3:
    tl_pll3_update_dll(&s->pll3, cs2);
    break;

  case TP_CTRL_FLL1:
    tl_fll1_update_dll(&s->fll1, cs2);
    break;

  case TP_CTRL_FLL2:
    tl_fll2_update_dll(&s->fll2, cs2);
    break;

  default:
    assert(false);
  }
}

/**
 * Return DLL error if available.
 *
 * \param[in] s Tracker state.
 *
 * \return Error in Hz between DLL and PLL/FLL filters.
 */
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

/**
 * Test if the tracker is PLL.
 *
 * \param[in] s Tracker state.
 *
 * \retval true  PLL or FLL-assisted PLL is used.
 * \retval false FLL-only tracking is used.
 */
bool tp_tl_is_pll(const tp_tl_state_t *s)
{
  switch (s->ctrl) {
  case TP_CTRL_PLL2:
  case TP_CTRL_PLL3:
    return true;

  default:
    return false;
  }
}

/**
 * First FLL discriminator update.
 *
 * Ignore updating discriminator (due to possible data bit change).
 * Update I_prev & Q_prev only.
 *
 * \param[in,out] s  Tracker state.
 * \param[in]     cs EPL correlator outputs.
 *
 * \return None
 *
 * \sa tp_tl_fll_update_second
 * \sa tp_tl_fll_update
 */
void tp_tl_fll_update_first(tp_tl_state_t *s, corr_t cs)
{
  switch (s->ctrl) {
  case TP_CTRL_PLL2:
    tl_pll2_discr_update(&s->pll2, cs.I, cs.Q, false);
    break;

  case TP_CTRL_PLL3:
    tl_pll3_discr_update(&s->pll3, cs.I, cs.Q, false);
    break;

  case TP_CTRL_FLL1:
    tl_fll1_discr_update(&s->fll1, cs.I, cs.Q, false);
    break;

  case TP_CTRL_FLL2:
    tl_fll2_discr_update(&s->fll2, cs.I, cs.Q, false);
    break;

  default:
    assert(false);
  }
}

/**
 * Second FLL discriminator update.
 *
 * Update discriminator, I_prev & Q_prev.
 *
 * \param[in,out] s  Tracker state.
 * \param[in]     cs EPL correlator outputs.
 *
 * \return None
 *
 * \sa tp_tl_fll_update_first
 * \sa tp_tl_fll_update
 */
void tp_tl_fll_update_second(tp_tl_state_t *s, corr_t cs)
{
  switch (s->ctrl) {
  case TP_CTRL_PLL2:
    tl_pll2_discr_update(&s->pll2, cs.I, cs.Q, true);
    break;

  case TP_CTRL_PLL3:
    tl_pll3_discr_update(&s->pll3, cs.I, cs.Q, true);
    break;

  case TP_CTRL_FLL1:
    tl_fll1_discr_update(&s->fll1, cs.I, cs.Q, true);
    break;

  case TP_CTRL_FLL2:
    tl_fll2_discr_update(&s->fll2, cs.I, cs.Q, true);
    break;

  default:
    assert(false);
  }
}

/**
 * Perform FLL adjustment in tracking loop filter.
 *
 * The method performs FLL adjustment according to accumulated discriminator
 * values.
 *
 * \param[in,out] s Tracker state.
 *
 * \return None
 *
 * \sa tp_tl_fll_update_first
 * \sa tp_tl_fll_update_second
 */
void tp_tl_fll_update(tp_tl_state_t *s)
{
  switch (s->ctrl) {
  case TP_CTRL_PLL2:
    tl_pll2_update_fll(&s->pll2);
    break;

  case TP_CTRL_PLL3:
    tl_pll3_update_fll(&s->pll3);
    break;

  case TP_CTRL_FLL1:
    tl_fll1_update_fll(&s->fll1);
    break;

  case TP_CTRL_FLL2:
    tl_fll2_update_fll(&s->fll2);
    break;

  default:
    assert(false);
  }
}

/**
 * Test if the tracker is FLL.
 *
 * \param[in] s Tracker state.
 *
 * \retval true  FLL-only tracking is used.
 * \retval false PLL or FLL-assisted PLL is used.
 */
bool tp_tl_is_fll(const tp_tl_state_t *s)
{
  switch (s->ctrl) {
  case TP_CTRL_FLL1:
  case TP_CTRL_FLL2:
    return true;

  default:
    return false;
  }
}
