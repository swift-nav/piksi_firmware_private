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


#include <board.h>

/* C/N0 estimator IIR averaging coefficient */
/* See http://www.insidegnss.com/auto/IGM_gnss-sol-janfeb10.pdf p. 22 */
/* See http://dsp.stackexchange.com/questions/378/ */
//#define CN0_EST_LPF_ALPHA     (0.016666667f) /* N=72 */
//#define CN0_EST_LPF_ALPHA     (0.005555556f) /* N=200 */
#define CN0_EST_LPF_ALPHA     (.0167f)
/* C/N0 LPF cutoff frequency. The lower it is, the more stable CN0 looks like */
#define CN0_EST_LPF_CUTOFF_HZ (.1f)

/* Noise bandwidth: GPS L1 1.023 * 2. Normalized with sample rate. The
 * approximate formula is:
 *
 * CN0_EST_BW_HZ = 3.1e-7f * NAP_FRONTEND_SAMPLE_RATE_Hz
 *
 * For V2 the ENBW is 5, for V3 it is 32.
 */
#if defined(BOARD_PIKSI_V2)
/* PIKSIv2 */
#define CN0_EST_BW_HZ         (5.f)
#elif defined(BOARD_DIGILENT_UZED)
/* PIKSIv3 */
#define CN0_EST_BW_HZ         (32.f)
#else
#error Unsupported board
#endif

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

/** C/N0 estimator and filter parameters: one pair per integration time */
static track_cn0_params_t cn0_est_pre_computed[INTEG_PERIODS_NUM];

/* Pre-compute C/N0 estimator and filter parameters. The parameters are
 * computed using equivalent of cn0_est_compute_params() function for
 * integration periods and cut-off frequency defined in this file.
 */
void track_cn0_params_init(void)
{
  for(u32 i = 0; i < INTEG_PERIODS_NUM; i++) {
    float loop_freq = 1e3f / integration_periods[i];
    cn0_est_compute_params(&cn0_est_pre_computed[i].est_params,
                           CN0_EST_BW_HZ,
                           CN0_EST_LPF_ALPHA,
                           loop_freq);
    cn0_filter_compute_params(&cn0_est_pre_computed[i].filter_params,
                              CN0_EST_LPF_CUTOFF_HZ,
                              loop_freq);
  }
}

static const track_cn0_params_t *track_cn0_get_params(u8 int_ms,
                                                      track_cn0_params_t *p)
{
  const track_cn0_params_t *pparams = NULL;

  for (u32 i = 0; i < INTEG_PERIODS_NUM; i++) {
    if (int_ms == integration_periods[i]) {
      pparams = &cn0_est_pre_computed[i];
      break;
    }
  }

  if (NULL == pparams) {
    float loop_freq = 1e3f / int_ms;
    cn0_est_compute_params(&p->est_params, CN0_EST_BW_HZ, CN0_EST_LPF_ALPHA,
                           loop_freq);

    cn0_filter_compute_params(&p->filter_params,
                              CN0_EST_LPF_CUTOFF_HZ,
                              loop_freq);
    pparams = p;
  }

  return pparams;
}


void track_cn0_init(u8 int_ms,
                    track_cn0_state_t *e,
                    float cn0_0)
{
  track_cn0_params_t p;
  const track_cn0_params_t *pp = track_cn0_get_params(int_ms, &p);

  cn0_est_bl_init(&e->primary, &pp->est_params, cn0_0);
  cn0_est_snv_init(&e->secondary, &pp->est_params, cn0_0);

  cn0_filter_init(&e->filter, &pp->filter_params, cn0_0);
}

float track_cn0_update(track_cn0_est_e t,
                       u8 int_ms,
                       track_cn0_state_t *e,
                       float I, float Q)
{
  track_cn0_params_t p;
  const track_cn0_params_t *pp = track_cn0_get_params(int_ms, &p);
  float cn0 = 0;
  float cn0_bl, cn0_snv;

  cn0_bl = cn0_est_bl_update(&e->primary, &pp->est_params, I, Q);
  cn0_snv = cn0_est_snv_update(&e->secondary, &pp->est_params, I, Q);

  switch (t) {
  case TRACK_CN0_EST_BL:
    cn0 = cn0_bl;
    break;

  case TRACK_CN0_EST_SNV:
    cn0 = cn0_snv;
    break;

  default:
    assert(false);
  }

  cn0 = cn0_filter_update(&e->filter, &pp->filter_params, cn0);

  return cn0;
}
