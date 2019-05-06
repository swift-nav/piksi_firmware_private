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
#include "track_cn0.h"

#include <assert.h>
#include <board.h>
#include <math.h>
#include <stdint.h>

#include "chconf.h"
#include "settings/settings_client.h"
#include "track/tracker.h"

/** C/N0 LPF cutoff frequency. The lower it is, the more stable CN0 looks like
 * and the slower is the response. */
#define CN0_EST_LPF_CUTOFF_HZ (.25f)

#define MAX_CN0_UPDATE_INTERVAL_MS 20

/**
 * C/N0 estimator parameters.
 */
typedef struct {
  cn0_est_params_t estimator;    /**< C/N0 estimator algorithm */
  cn0_filter_params_t lp_filter; /**< Additional C/N0 value LP filter */
} track_cn0_params_t;

/**< Estimator and filter  parameters */
static track_cn0_params_t g_params[1 + MAX_CN0_UPDATE_INTERVAL_MS];

/** Pre-compute C/N0 estimator and filter parameters. The parameters are
 * computed using equivalent of cn0_est_compute_params() function for
 * integration periods and cut-off frequency defined in this file.
 */
void track_cn0_params_init(void) {
  for (u8 ms = 1; ms <= MAX_CN0_UPDATE_INTERVAL_MS; ms++) {
    track_cn0_params_t *params = &g_params[ms];
    cn0_est_compute_params(&(params->estimator), ms);
    const float loop_freq = 1000.0f / ms;
    cn0_filter_compute_params(
        &(params->lp_filter), CN0_EST_LPF_CUTOFF_HZ, loop_freq);
  }
}

/**
 * Helper for estimator initialization
 *
 * \param[out] e     Estimator state.
 * \param[in]  cn0_0 Initial C/N0 value.
 *
 * \return None
 */
static void init_estimator(track_cn0_state_t *state, float cn0_0) {
  cn0_est_mm_init(&state->moment, cn0_0);
}

/**
 * Helper for estimator update
 *
 * \param[in,out] e Estimator state.
 * \param[in]     p Estimator parameters.
 * \param[in]     I      In-phase component.
 * \param[in]     Q      Quadrature component.
 *
 * \return Estimator update result (dB/Hz).
 */
static float cn0_update_estimator(track_cn0_state_t *state,
                                  const cn0_est_params_t *p,
                                  float I,
                                  float Q) {
  cn0_est_mm_update(&state->moment, p, I, Q);
  return state->moment.cn0_dbhz;
}

/**
 * Helper for C/N0 estimator parameter lookup.
 *
 * \param[in]     cn0_ms Estimator update period.
 *
 * \return Precomputed parameter entry.
 */
static const track_cn0_params_t *track_cn0_get_params(u8 cn0_ms) {
  assert((0 < cn0_ms) && (cn0_ms <= MAX_CN0_UPDATE_INTERVAL_MS));
  return &g_params[cn0_ms];
}

/**
 * Initializes C/N0 estimator and filter
 *
 * \param[out] e      C/N0 estimator state.
 * \param[in]  cn0_ms C/N0 estimator update period in ms.
 * \param[in]  cn0    Initial C/N0 value in dB/Hz.
 *
 * \return None
 */
void track_cn0_init(track_cn0_state_t *state, u8 cn0_ms, float cn0) {
  const track_cn0_params_t *params = track_cn0_get_params(cn0_ms);

  init_estimator(state, cn0);
  cn0_filter_init(&state->filter, &params->lp_filter, cn0);
}

/**
 * Updates C/N0 estimator.
 *
 * \param[in,out] e      Estimator state.
 * \param[in]     cn0_ms CN0 integration time [ms]
 * \param[in]     I      In-phase component.
 * \param[in]     Q      Quadrature component.
 *
 * \return Filtered estimator value.
 */
void track_cn0_update(track_cn0_state_t *state, u8 cn0_ms, float I, float Q) {
  const track_cn0_params_t *params = track_cn0_get_params(cn0_ms);

  state->cn0_dbhz_inst = cn0_update_estimator(state, &params->estimator, I, Q);
  state->cn0_dbhz_filt = cn0_filter_update(
      &state->filter, &params->lp_filter, state->cn0_dbhz_inst);

  if (state->cn0_dbhz_inst < THRESH_SENS_DBHZ) {
    if (state->weak_signal_ms < SECS_MS) { /* to avoid wrapping to 0 */
      state->weak_signal_ms += cn0_ms;
    }
  } else {
    state->weak_signal_ms = 0;
  }
}

/**
 * Returns C/N0 offset according to C/N0 integration period.
 *
 * \param[in] cn0_ms Integration period of C/N0 estimator.
 *
 * \return Offset in dB/Hz that corresponds to C/N0 increase for the given input
 */
float track_cn0_get_offset(u8 cn0_ms) {
  return g_params[cn0_ms].estimator.cn0_offset_db;
}
