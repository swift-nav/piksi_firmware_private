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
#include <settings.h>

/** Configuration section name */
#define TRACK_CN0_EST_SETTING_SECTION "cn0_est"

/** C/N0 estimator IIR averaging coefficient:
 * See http://www.insidegnss.com/auto/IGM_gnss-sol-janfeb10.pdf p. 22
 * See http://dsp.stackexchange.com/questions/378/
 *
 * For N=24 Alpha=0.05(3)
 * For N=72 Alpha=0.016(6)
 * For N=200 Alpha=0.0055(5)
 */
#define CN0_EST_LPF_ALPHA     (.005f)
/** C/N0 LPF cutoff frequency. The lower it is, the more stable CN0 looks like
 * and the slower is the response. */
#define CN0_EST_LPF_CUTOFF_HZ (.25f)
/** Integration interval: 1ms */
#define INTEG_PERIOD_1_MS  1
/** Integration interval: 5ms */
#define INTEG_PERIOD_5_MS  5
/** Integration interval: 5ms */
#define INTEG_PERIOD_10_MS 10
/** Integration interval: 20ms */
#define INTEG_PERIOD_20_MS 20

/** C/N0 offset for 1ms estimator interval [dB/Hz] */
#define TRACK_CN0_OFFSET_1MS_DBHZ  0
/** C/N0 offset for 5ms estimator interval [dB/Hz] */
#define TRACK_CN0_OFFSET_5MS_DBHZ  7
/** C/N0 offset for 10ms estimator interval [dB/Hz] */
#define TRACK_CN0_OFFSET_10MS_DBHZ 10
/** C/N0 offset for 20ms estimator interval [dB/Hz] */
#define TRACK_CN0_OFFSET_20MS_DBHZ 13
/** Total number of precomputed integration intervals */
#define INTEG_PERIODS_NUM (ARRAY_SIZE(cn0_periods_ms))

/** Predefined integration periods for C/N0 estimators */
static const u8 cn0_periods_ms[] = {
  INTEG_PERIOD_1_MS,
  INTEG_PERIOD_5_MS,
  INTEG_PERIOD_10_MS,
  INTEG_PERIOD_20_MS
};

/**
 * Subsystem configuration type.
 */
typedef struct
{
  float alpha;             /**< Estimator alpha coefficient */
  float nbw;               /**< Noise bandwidth for the platform */
  float scale;             /**< Scale factor for C/N0 estimator */
  float cn0_shift;         /**< Shift for C/N0 estimator */
  float cutoff;            /**< C/N0 LP filter cutoff frequency [Hz] */
  u8    update_count;      /**< Configuration update counter */
  track_cn0_params_t params[INTEG_PERIODS_NUM]; /**< Estimator and filter
                                                 *   parameters */
  float pri2sec_threshold; /**< Threshold for switching primary to secondary
                            *   estimator  (20ms) [dB/Hz].
                            *
                            * The value is specified for 20ms C/N0 estimator
                            * period, and automatically adjusted for other
                            * intervals.
                            *
                            * \sa track_cn0_get_pri2sec_threshold
                            */
  float sec2pri_threshold; /**< Threshold for switching secondary to primary
                            *   estimator  (20ms) [dB/Hz].
                            *
                            * The value is specified for 20ms C/N0 estimator
                            * period, and automatically adjusted for other
                            * intervals.
                            *
                            * \sa track_cn0_get_sec2pri_threshold
                            */
} track_cn0_config_t;

/**
 * Subsystem configuration.
 */
static track_cn0_config_t cn0_config PLATFORM_CN0_DATA = {
  .alpha = CN0_EST_LPF_ALPHA,
  .nbw = PLATFORM_CN0_EST_BW_HZ,
  .scale = PLATFORM_CN0_EST_SCALE,
  .cn0_shift = PLATFORM_CN0_EST_SHIFT,
  .cutoff = CN0_EST_LPF_CUTOFF_HZ,
  .update_count = 0,
  .pri2sec_threshold = TRACK_CN0_PRI2SEC_THRESHOLD,
  .sec2pri_threshold = TRACK_CN0_SEC2PRI_THRESHOLD,
};

static float q_avg = 8.f; /* initial value for noise level */

/**
 * Helper to recompute estimator parameters
 */
static void recompute_settings(void)
{
  for(u32 i = 0; i < INTEG_PERIODS_NUM; i++) {
    float loop_freq = 1e3f / cn0_periods_ms[i];

    cn0_est_compute_params(&cn0_config.params[i].est_params,
                           cn0_config.nbw,
                           cn0_config.alpha,
                           loop_freq,
                           cn0_config.scale,
                           cn0_config.cn0_shift);
    cn0_config.params[i].est_params.t_int = cn0_periods_ms[i];
    cn0_filter_compute_params(&cn0_config.params[i].filter_params,
                              cn0_config.cutoff,
                              loop_freq);
  }
}

/**
 * Updates C/N0 setting.
 *
 * The method updates C/N0 setting and recomputes active estimator parameters
 * if required.
 *
 * \param[in] s   Setting definition
 * \param[in] val Value literal
 *
 * \retval true if the setting has been changed.
 */
static bool settings_notify_proxy(struct setting *s, const char *val)
{
  bool res = settings_default_notify(s, val);

  if (res) {
    cn0_config.update_count ++;
    recompute_settings();
  }

  return res;
}

/** Pre-compute C/N0 estimator and filter parameters. The parameters are
 * computed using equivalent of cn0_est_compute_params() function for
 * integration periods and cut-off frequency defined in this file.
 */
void track_cn0_params_init(void)
{
  for (u32 i = 0; i < INTEG_PERIODS_NUM; i++) {
    float loop_freq = 1e3f / cn0_periods_ms[i];
    cn0_est_compute_params(&cn0_config.params[i].est_params,
                           cn0_config.nbw,
                           cn0_config.alpha,
                           loop_freq,
                           cn0_config.scale,
                           cn0_config.cn0_shift);
    cn0_config.params[i].est_params.t_int = cn0_periods_ms[i];
    cn0_filter_compute_params(&cn0_config.params[i].filter_params,
                              cn0_config.cutoff,
                              loop_freq);
  }
  cn0_config.update_count = 1;
  recompute_settings();

  SETTING_NOTIFY(TRACK_CN0_EST_SETTING_SECTION, "alpha",
                 cn0_config.alpha,
                 TYPE_FLOAT, settings_notify_proxy);
  SETTING_NOTIFY(TRACK_CN0_EST_SETTING_SECTION, "nbw",
                 cn0_config.nbw,
                 TYPE_FLOAT, settings_notify_proxy);
  SETTING_NOTIFY(TRACK_CN0_EST_SETTING_SECTION, "scale",
                 cn0_config.scale,
                 TYPE_FLOAT, settings_notify_proxy);
  SETTING_NOTIFY(TRACK_CN0_EST_SETTING_SECTION, "cn0_shift",
                 cn0_config.cn0_shift,
                 TYPE_FLOAT, settings_notify_proxy);
  SETTING_NOTIFY(TRACK_CN0_EST_SETTING_SECTION, "cutoff",
                 cn0_config.cutoff,
                 TYPE_FLOAT, settings_notify_proxy);
  SETTING(TRACK_CN0_EST_SETTING_SECTION, "pri2sec_threshold",
          cn0_config.pri2sec_threshold,
          TYPE_FLOAT);
  SETTING(TRACK_CN0_EST_SETTING_SECTION, "sec2pri_threshold",
          cn0_config.sec2pri_threshold,
          TYPE_FLOAT);
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
  case TRACK_CN0_EST_BASIC:
    cn0_est_basic_init(&e->basic, p, cn0_0, q_avg * sqrtf(p->t_int));
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
 * \param[in]     ve_I   Very early in-phase accumulator
 * \param[in]     ve_Q   Very early quadrature accumulator
 *
 * \return Estimator update result (dB/Hz).
 */
static float update_estimator(track_cn0_state_t *e,
                              const cn0_est_params_t *p,
                              track_cn0_est_e t,
                              float I, float Q,
                              float ve_I, float ve_Q)
{
  float cn0 = 0;
  float cn0_basic = cn0_est_basic_update(&e->basic, p, I, Q, ve_I, ve_Q);

  switch (t) {
  case TRACK_CN0_EST_BASIC:
    q_avg = q_avg * (1 - p->alpha) + p->alpha * e->basic.noise_Q_abs / sqrtf(p->t_int);
    cn0 = cn0_basic;
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
                                                      track_cn0_params_t *p)
{
  const track_cn0_params_t *pparams = NULL;
  u8 config_key = cn0_ms;

  for (u32 i = 0; i < INTEG_PERIODS_NUM; i++) {
    if (config_key == cn0_periods_ms[i]) {
      pparams = &cn0_config.params[i];
      break;
    }
  }

  if (NULL == pparams) {
    float loop_freq = 1e3f / cn0_ms;
    cn0_est_compute_params(&p->est_params,
                           cn0_config.nbw,
                           cn0_config.alpha,
                           loop_freq,
                           cn0_config.scale,
                           cn0_config.cn0_shift);
    p->est_params.t_int = cn0_ms;

    float cutoff_freq = 1;
    cutoff_freq = CN0_EST_LPF_CUTOFF_HZ;

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
 * \param[in]  mesid  ME signal identifier for logging.
 * \param[in]  cn0_ms C/N0 estimator update period in ms.
 * \param[out] e      C/N0 estimator state.
 * \param[in]  cn0_0  Initial C/N0 value in dB/Hz.
 * \param[in]  flags  Tuning flags.
 *
 * \return None
 */
void track_cn0_init(const me_gnss_signal_t mesid,
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

  const track_cn0_params_t *pp = track_cn0_get_params(cn0_ms, &p);

  init_estimator(e, &pp->est_params, TRACK_CN0_EST_PRIMARY, cn0_0);
  init_estimator(e, &pp->est_params, TRACK_CN0_EST_SECONDARY, cn0_0);

  cn0_filter_init(&e->filter, &pp->filter_params, cn0_0);

  e->ver = cn0_config.update_count;

  log_debug_mesid(mesid,
                  "Initializing estimator %s (%f dB/Hz @ %u ms)",
                  track_cn0_str(e->type),
                  e->filter.yn,
                  (unsigned)e->cn0_ms);
}

/**
 * Updates C/N0 estimator.
 *
 * \param[in]     mesid  ME signal identifier for logging.
 * \param[in]     t      Type of estimator value to use/return.
 * \param[in,out] e      Estimator state.
 * \param[in]     I      In-phase component.
 * \param[in]     Q      Quadrature component.
 * \param[in]     ve_I   Very early in-phase accumulator
 * \param[in]     ve_Q   Very early quadrature accumulator
 *
 * \return Filtered estimator value.
 */
float track_cn0_update(const me_gnss_signal_t mesid,
                       track_cn0_est_e t,
                       track_cn0_state_t *e,
                       float I, float Q,
                       float ve_I, float ve_Q)
{
  track_cn0_params_t p;
  const track_cn0_params_t *pp = track_cn0_get_params(e->cn0_ms, &p);
  float cn0 = 0;

  if (e->ver != cn0_config.update_count) {
    u8 cn0_0 = e->cn0_0;
    track_cn0_init(mesid, e->cn0_ms, e, e->filter.yn, e->flags);
    e->cn0_0 = cn0_0;
  }

  if (e->type != t) {
    log_debug_mesid(mesid,
                    "Changing estimator from %s to %s at (%f dB/Hz @ %u ms)",
                    track_cn0_str(e->type),
                    track_cn0_str(t),
                    e->filter.yn,
                    (unsigned)e->cn0_ms);
    e->type = t;
  }

  cn0 = update_estimator(e, &pp->est_params, t, I, Q, ve_I, ve_Q);
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
  case TRACK_CN0_EST_BASIC: str = "BASIC"; break;
  default: assert(!"Unknown estimator type");
  }
  return str;
}

/**
 * Returns C/N0 offset according to C/N0 integration period.
 *
 * \param[in] cn0_ms Integration period of C/N0 estimator.
 *
 * \return Offset in dB/Hz that corresponds to C/N0 increase for the given input
 */
float track_cn0_get_offset(u8 cn0_ms)
{
  float cn0_offset = 0;

  switch (cn0_ms) {
  case INTEG_PERIOD_1_MS:
    cn0_offset = TRACK_CN0_OFFSET_1MS_DBHZ;
    break;

  case INTEG_PERIOD_5_MS:
    cn0_offset = TRACK_CN0_OFFSET_5MS_DBHZ;
    break;

  case INTEG_PERIOD_10_MS:
    cn0_offset = TRACK_CN0_OFFSET_10MS_DBHZ;
    break;

  case INTEG_PERIOD_20_MS:
    cn0_offset = TRACK_CN0_OFFSET_20MS_DBHZ;
    break;

  default:
    cn0_offset = 10.f * log10f(cn0_ms);
    break;
  }
  return cn0_offset;
}

/**
 * Computes C/N0 threshold for enabling secondary estimator.
 *
 * \param[in] cn0_ms C/N0 estimator integration time [ms]
 *
 * \return C/N0 threshold in dB/Hz for enabling secondary estimator
 *
 * \sa track_cn0_get_sec2pri_threshold
 */
float track_cn0_get_pri2sec_threshold(u8 cn0_ms)
{
  float cn0_offset = track_cn0_get_offset(cn0_ms);
  return cn0_config.pri2sec_threshold + TRACK_CN0_OFFSET_20MS_DBHZ - cn0_offset;
}

/**
 * Computes C/N0 threshold for enabling primary estimator.
 *
 * \param[in] cn0_ms C/N0 estimator integration time [ms]
 *
 * \return C/N0 threshold in dB/Hz for enabling primary estimator
 *
 * \sa track_cn0_get_pri2sec_threshold
 */
float track_cn0_get_sec2pri_threshold(u8 cn0_ms)
{
  float cn0_offset = track_cn0_get_offset(cn0_ms);
  return cn0_config.sec2pri_threshold + TRACK_CN0_OFFSET_20MS_DBHZ - cn0_offset;
}
