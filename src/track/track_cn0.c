/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Valeri Atamaniouk <valeri@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "track_cn0.h"

#include <assert.h>
#include <math.h>

#include <chconf.h>
#include <board.h>
#include <platform_cn0.h>

/** C/N0 estimator IIR averaging coefficient:
 * See http://www.insidegnss.com/auto/IGM_gnss-sol-janfeb10.pdf p. 22
 * See http://dsp.stackexchange.com/questions/378/
 *
 * For N=72 Alpha=0.016(6)
 * For N=200 Alpha=0.0055(5)
 */
#define CN0_EST_LPF_ALPHA     (.0167f)
/** C/N0 LPF cutoff frequency. The lower it is, the more stable CN0 looks like
 * and the slower is the response. */
#define CN0_EST_LPF_CUTOFF_HZ (.1f)
/** C/N0 LPF cutoff frequency computed from C/N0 ms to make C/N0 faster when
 * signals are strong. */
#define CN0_EST_LPF_IT_CUTOFF_HZ(ms) (CN0_EST_LPF_CUTOFF_HZ * expf((ms) / -20.f))
/** C/N0 LPF cutoff frequency computed from C/N0 ms to make C/N0 faster when
 * signals are strong. This is fast alternative. */
#define CN0_EST_LPF_IT_FAST_CUTOFF_HZ(ms) \
  (CN0_EST_LPF_CUTOFF_HZ * expf((ms) / -20.f + 1.9f))

#define INTEG_PERIOD_1_MS  1
#define INTEG_PERIOD_5_MS  5
#define INTEG_PERIOD_10_MS 10
#define INTEG_PERIOD_20_MS 20

/** Mask for faster filter */
#define INTEG_PERIOD_FAST_MASK 0x80u

static const u8 cn0_configs[] = {
  INTEG_PERIOD_1_MS,
  INTEG_PERIOD_5_MS,
  INTEG_PERIOD_10_MS,
  INTEG_PERIOD_20_MS,
  INTEG_PERIOD_20_MS | INTEG_PERIOD_FAST_MASK
};

#define INTEG_PERIODS_NUM (sizeof(cn0_configs) / \
                           sizeof(cn0_configs[0]))

/** C/N0 estimator and filter parameters: one pair per integration time */
static track_cn0_params_t cn0_est_pre_computed[INTEG_PERIODS_NUM] PLATFORM_CN0_DATA;

/** Pre-compute C/N0 estimator and filter parameters. The parameters are
 * computed using equivalent of cn0_est_compute_params() function for
 * integration periods and cut-off frequency defined in this file.
 */
void track_cn0_params_init(void)
{
  for(u32 i = 0; i < INTEG_PERIODS_NUM; i++) {
    bool fast_mask = (0 != (cn0_configs[i] & INTEG_PERIOD_FAST_MASK));
    u8 actual_ms = cn0_configs[i] & ~INTEG_PERIOD_FAST_MASK;

    float cutoff_freq = 1;
    if (fast_mask)
      cutoff_freq = CN0_EST_LPF_IT_FAST_CUTOFF_HZ(actual_ms);
    else
      cutoff_freq = CN0_EST_LPF_IT_CUTOFF_HZ(actual_ms);

    float loop_freq = 1e3f / actual_ms;
    cn0_est_compute_params(&cn0_est_pre_computed[i].est_params,
                           PLATFORM_CN0_EST_BW_HZ,
                           CN0_EST_LPF_ALPHA,
                           loop_freq);
    cn0_est_pre_computed[i].est_params.t_int = cn0_configs[i];
    cn0_filter_compute_params(&cn0_est_pre_computed[i].filter_params,
                              cutoff_freq,
                              loop_freq);
  }
}

/**
 * Helper for estimator initialization
 *
 * \param[out] e     Estimator state.
 * \param[in]  p     Estimator parameters.
 * \param[in]  t     Estimator type.
 * \param[in]  cn0_0 Initial C/N0 value.
 *
 * \return None
 */
static void init_estimator(track_cn0_state_t *e,
                           const cn0_est_params_t *p,
                           track_cn0_est_e t,
                           float cn0_0)
{
  switch (t) {
  case TRACK_CN0_EST_BL:
    cn0_est_bl_init(&e->bl, p, cn0_0);
    break;

  case TRACK_CN0_EST_MM:
    cn0_est_mm_init(&e->mm, p, cn0_0);
    break;

  default:
    assert(false);
  }

}

/**
 * Helper for estimator update
 *
 * \param[in,out] e Estimator state.
 * \param[in]     p Estimator parameters.
 * \param[in]     t Estimator type.
 * \param[in]     I      In-phase component.
 * \param[in]     Q      Quadrature component.
 *
 * \return Estimator update result (dB/Hz).
 */
static float update_estimator(track_cn0_state_t *e,
                              const cn0_est_params_t *p,
                              track_cn0_est_e t,
                              float I, float Q)
{
  float cn0 = 0;
  float cn0_bl = 0;
  float cn0_mm = 0;

  cn0_bl = cn0_est_bl_update(&e->bl, p, I, Q);
  cn0_mm = cn0_est_mm_update(&e->mm, p, I, Q);

  switch (t) {
  case TRACK_CN0_EST_BL:
    cn0 = cn0_bl;
    break;

  case TRACK_CN0_EST_MM:
    cn0 = cn0_mm;
    break;

  default:
    assert(false);
  }

  return cn0;
}

/**
 * Helper for C/N0 estimator parameter lookup.
 *
 * \param[in]     cn0_ms Estimator update period.
 * \param[in,out] p      Parameter buffer to use if precomputed parameters are
 *                       not available.
 *
 * \return Precomputed parameter entry or \a p populated with appropriate
 *         parameters if precomputed entry is not available.
 */
static const track_cn0_params_t *track_cn0_get_params(u8 cn0_ms,
                                                      track_cn0_params_t *p,
                                                      u8 flags)
{
  const track_cn0_params_t *pparams = NULL;
  u8 config_key = cn0_ms;

  if (0 != (flags & TRACK_CN0_FLAG_FAST_TYPE))
    config_key |= INTEG_PERIOD_FAST_MASK;

  for (u32 i = 0; i < INTEG_PERIODS_NUM; i++) {
    if (config_key == cn0_configs[i]) {
      pparams = &cn0_est_pre_computed[i];
      break;
    }
  }

  if (NULL == pparams) {
    float loop_freq = 1e3f / cn0_ms;
    cn0_est_compute_params(&p->est_params, PLATFORM_CN0_EST_BW_HZ,
                           CN0_EST_LPF_ALPHA,
                           loop_freq);
    p->est_params.t_int = cn0_ms;

    float cutoff_freq = 1;
    if (0 != (flags & TRACK_CN0_FLAG_FAST_TYPE))
      cutoff_freq = CN0_EST_LPF_IT_FAST_CUTOFF_HZ(cn0_ms);
    else
      cutoff_freq = CN0_EST_LPF_IT_CUTOFF_HZ(cn0_ms);

    cn0_filter_compute_params(&p->filter_params,
                              cutoff_freq,
                              loop_freq);

    pparams = p;
  }

  return pparams;
}

/**
 * Initializes C/N0 estimator
 *
 * \param[in]  sid    Signal identifier for logging.
 * \param[in]  cn0_ms C/N0 estimator update period in ms.
 * \param[out] e      C/N0 estimator state.
 * \param[in]  cn0_0  Initial C/N0 value in dB/Hz.
 * \param[in]  flags  Tuning flags.
 *
 * \return None
 */
void track_cn0_init(gnss_signal_t sid,
                    u8 cn0_ms,
                    track_cn0_state_t *e,
                    float cn0_0,
                    u8 flags)
{
  track_cn0_params_t p;

  e->type = TRACK_CN0_EST_PRIMARY;
  e->cn0_0 = (u8)cn0_0;
  e->flags = flags;
  e->cn0_ms = cn0_ms;

  const track_cn0_params_t *pp = track_cn0_get_params(cn0_ms, &p, e->flags);

  init_estimator(e, &pp->est_params, TRACK_CN0_EST_PRIMARY, cn0_0);
  init_estimator(e, &pp->est_params, TRACK_CN0_EST_SECONDARY, cn0_0);

  cn0_filter_init(&e->filter, &pp->filter_params, cn0_0);

  log_debug_sid(sid, "Initializing estimator %s (%f dB/Hz @ %u ms)",
                track_cn0_str(e->type),
                e->filter.yn,
                (unsigned)e->cn0_ms);
}

/**
 * Updates C/N0 estimator.
 *
 * \param[in]     sid    Signal identifier for logging.
 * \param[in]     t      Type of estimator value to use/return.
 * \param[in,out] e      Estimator state.
 * \param[in]     I      In-phase component.
 * \param[in]     Q      Quadrature component.
 *
 * \return Filtered estimator value.
 */
float track_cn0_update(gnss_signal_t sid,
                       track_cn0_est_e t,
                       track_cn0_state_t *e,
                       float I, float Q)
{
  track_cn0_params_t p;
  const track_cn0_params_t *pp = track_cn0_get_params(e->cn0_ms, &p, e->flags);
  float cn0 = 0;

  if (e->type != t) {
    log_debug_sid(sid, "Changing estimator from %s to %s at (%f dB/Hz @ %u ms)",
                  track_cn0_str(e->type),
                  track_cn0_str(t),
                  e->filter.yn,
                  (unsigned)e->cn0_ms);
    e->type = t;
  }

  cn0 = update_estimator(e, &pp->est_params, t, I, Q);
  cn0 = cn0_filter_update(&e->filter, &pp->filter_params, cn0);

  return cn0;
}

/**
 * Provides literal constant for C/N0 estimator type
 *
 * \param[in] t C/N0 estimator type
 *
 * \return Abbreviated estimator name.
 */
const char *track_cn0_str(track_cn0_est_e t)
{
  const char *str = "?";
  switch (t) {
  case TRACK_CN0_EST_BL: str = "BL"; break;
  case TRACK_CN0_EST_MM: str = "MM"; break;
  default: assert(false);
  }
  return str;
}
