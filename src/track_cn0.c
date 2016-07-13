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
 * CN0_EST_BW_HZ = NBW / TRACK_SAMPLE_FREQ
 *
 * For V2 the ENBW is 4.88, for V3 it is 26.4.
 */
#if defined(BOARD_PIKSI_V2)
/* PIKSIv2 */
/* #define CN0_EST_BW_HZ  (float)(2e6 / TRACK_SAMPLE_FREQ * 40) */
#define CN0_EST_BW_HZ     (4.88f)
#elif defined(BOARD_DIGILENT_UZED)
/* PIKSIv3 */
/* #define CN0_EST_BW_HZ  (float)(33e6 / TRACK_SAMPLE_FREQ  * 20) */
#define CN0_EST_BW_HZ     (26.4f)
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
    cn0_est_pre_computed[i].est_params.t_int = integration_periods[i];
    cn0_filter_compute_params(&cn0_est_pre_computed[i].filter_params,
                              CN0_EST_LPF_CUTOFF_HZ,
                              loop_freq);
  }
}

static void init_estimator(cn0_est_state_t *e,
                           const cn0_est_params_t *p,
                           track_cn0_est_e t,
                           float cn0_0)
{
  switch (t) {
  case TRACK_CN0_EST_RSCN:
    cn0_est_rscn_init(e, p, cn0_0);
    break;

  case TRACK_CN0_EST_BL:
    cn0_est_bl_init(e, p, cn0_0);
    break;

  case TRACK_CN0_EST_SNV:
    cn0_est_rscn_init(e, p, cn0_0);
    break;

  case TRACK_CN0_EST_MM:
    cn0_est_mm_init(e, p, cn0_0);
    break;

  case TRACK_CN0_EST_NWPR:
    cn0_est_nwpr_init(e, p, cn0_0);
    break;

  case TRACK_CN0_EST_SVR:
    cn0_est_svr_init(e, p, cn0_0);
    break;

  case TRACK_CN0_EST_CH:
    cn0_est_ch_init(e, p, cn0_0);
    break;

  default:
    assert(false);
  }

}

static float update_estimator(cn0_est_state_t *e,
                              const cn0_est_params_t *p,
                              track_cn0_est_e t,
                              float I, float Q)
{
  float cn0 = 0;
  switch (t) {
  case TRACK_CN0_EST_RSCN:
    cn0 = cn0_est_rscn_update(e, p, I, Q);
    break;

  case TRACK_CN0_EST_BL:
    cn0 = cn0_est_bl_update(e, p, I, Q);
    break;

  case TRACK_CN0_EST_SNV:
    cn0 = cn0_est_snv_update(e, p, I, Q);
    break;

  case TRACK_CN0_EST_MM:
    cn0 = cn0_est_mm_update(e, p, I, Q);
    break;

  case TRACK_CN0_EST_NWPR:
    cn0 = cn0_est_nwpr_update(e, p, I, Q);
    break;

  case TRACK_CN0_EST_SVR:
    cn0 = cn0_est_svr_update(e, p, I, Q);
    break;

  case TRACK_CN0_EST_CH:
    cn0 = cn0_est_ch_update(e, p, I, Q);
    break;

  default:
    assert(false);
  }

  return cn0;
}


static const track_cn0_params_t *track_cn0_get_params(u8 int_ms,
                                                      track_cn0_params_t *p)
{
  const track_cn0_params_t *pparams = NULL;

  for (u32 i = 0; i < INTEG_PERIODS_NUM; i++) {
    if (int_ms == integration_periods[i]) {
      //cn0_est_pre_computed[i].est_params.t_int = int_ms;
      pparams = &cn0_est_pre_computed[i];
      break;
    }
  }

  if (NULL == pparams) {
    float loop_freq = 1e3f / int_ms;
    cn0_est_compute_params(&p->est_params, CN0_EST_BW_HZ, CN0_EST_LPF_ALPHA,
                           loop_freq);
    p->est_params.t_int = int_ms;

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

  init_estimator(&e->primary, &pp->est_params, TRACK_CN0_EST_PRIMARY, cn0_0);
  init_estimator(&e->secondary, &pp->est_params, TRACK_CN0_EST_SECONDARY, cn0_0);

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

  float cn0_pri = update_estimator(&e->primary, &pp->est_params, TRACK_CN0_EST_PRIMARY, I, Q);
  float cn0_sec = update_estimator(&e->secondary, &pp->est_params, TRACK_CN0_EST_SECONDARY, I, Q);

  switch (t) {
  case TRACK_CN0_EST_PRIMARY:
    cn0 = cn0_pri;
    break;
  case TRACK_CN0_EST_SECONDARY:
    cn0 = cn0_sec;
    break;
  default:
    assert(false);
  }

  cn0 = cn0_filter_update(&e->filter, &pp->filter_params, cn0);

  return cn0;
}

const char *track_cn0_str(track_cn0_est_e t)
{
  const char *str = "?";
  switch (t) {
  case TRACK_CN0_EST_RSCN: str = "RSCN"; break;
  case TRACK_CN0_EST_BL: str = "BL"; break;
  case TRACK_CN0_EST_SNV: str = "SNV"; break;
  case TRACK_CN0_EST_MM: str = "MM"; break;
  case TRACK_CN0_EST_NWPR: str = "NWPR"; break;
  case TRACK_CN0_EST_SVR: str = "SVR"; break;
  case TRACK_CN0_EST_CH: str = "CH"; break;
  default: assert(false);
  }
  return str;
}
