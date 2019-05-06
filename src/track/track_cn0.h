/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Michele Bavaro <michele@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_TRACK_CN0_H
#define SWIFTNAV_TRACK_CN0_H

#include <swiftnav/signal.h>

#include "cn0_est/cn0_est_common.h"
#include "filters/filter_common.h"
#include "nap/nap_constants.h"

/* Configure C/N0 value filter algorithm */
#define cn0_filter_params_t lp1_filter_params_t
#define cn0_filter_compute_params lp1_filter_compute_params
#define cn0_filter_t lp1_filter_t
#define cn0_filter_init lp1_filter_init
#define cn0_filter_update lp1_filter_update

/**
 * C/N0 estimator state.
 */
typedef struct {
  cn0_est_mm_state_t moment; /**< MM estimator */
  cn0_filter_t filter;       /**< Additional C/N0 filter */

  float cn0_dbhz_inst; /**< Last unfiltered CN0 estimation [dB-Hz] */
  float cn0_dbhz_filt; /**< Last unfiltered CN0 estimation [dB-Hz] */
  u32 weak_signal_ms;  /**< Signal is below #THRESH_SENS_DBHZ this long [ms] */
} track_cn0_state_t;

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void track_cn0_params_init(void);
void track_cn0_init(track_cn0_state_t *state, u8 cn0_ms, float cn0);
void track_cn0_update(track_cn0_state_t *state, u8 cn0_ms, float I, float Q);
float track_cn0_get_offset(u8 cn0_ms);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* SWIFTNAV_TRACK_CN0_H */
