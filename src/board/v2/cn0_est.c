/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Dmitry Tatarinov <dmitry.tatarinov@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#include "cn0_est.h"

#define INTEG_PERIOD_1_MS  1
#define INTEG_PERIOD_2_MS  2
#define INTEG_PERIOD_4_MS  4
#define INTEG_PERIOD_5_MS  5
#define INTEG_PERIOD_10_MS 10
#define INTEG_PERIOD_20_MS 20

static const u8 integration_periods[] = {
  INTEG_PERIOD_1_MS,
  INTEG_PERIOD_2_MS,
  INTEG_PERIOD_4_MS,
  INTEG_PERIOD_5_MS,
  INTEG_PERIOD_10_MS,
  INTEG_PERIOD_20_MS
};

#define INTEG_PERIODS_NUM (sizeof(integration_periods) / \
                           sizeof(integration_periods[0]))

#define CN0_EST_LPF_CUTOFF 0.1f

static cn0_est_params_t cn0_est_pre_computed[INTEG_PERIODS_NUM];

float cn0_estimate(track_cn0_state_t *data,
                   const corr_t* cs,
                   u8 int_ms)
{
  cn0_est_params_t params;
  const cn0_est_params_t *pparams = NULL;

  /* TODO
   * Store a pointer to the cn0_est_params_t in the gps_l1ca_tracker_data_t
   * structure so we don't have to scan through the whole array each time
   */
  for(u32 i = 0; i < INTEG_PERIODS_NUM; i++) {
    if(int_ms == integration_periods[i]) {
      pparams = &cn0_est_pre_computed[i];
      break;
    }
  }

  if(NULL == pparams) {
    cn0_est_compute_params(&params, 1e3f / int_ms, CN0_EST_LPF_CUTOFF,
                           1e3f / int_ms);
    pparams = &params;
  }

  return cn0_est(&data->state,
                 pparams,
                 (float) cs[1].I/int_ms,
                 (float) cs[1].Q/int_ms);
}

/* Pre-compute C/N0 estimator and filter parameters. The parameters are
 * computed using equivalent of cn0_est_compute_params() function for
 * integration periods of 1, 2, 4, 5, 10 and 20ms and cut-off frequency
 * of 0.1 Hz.
 */
void cn0_est_precompute(void)
{
  for(u32 i = 0; i < INTEG_PERIODS_NUM; i++) {
    cn0_est_compute_params(&cn0_est_pre_computed[i],
                           1e3f / integration_periods[i],
                           CN0_EST_LPF_CUTOFF,
                           1e3f / integration_periods[i]);
  }
}

/** Wrapper for cn0_est_init
 */
void cn0_init(track_cn0_state_t *s, u8 int_ms, float cn0_0)
{
  cn0_est_init(&s->state, 1e3f/int_ms, cn0_0);
}
