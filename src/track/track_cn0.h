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

#ifndef SWIFTNAV_TRACK_CN0_H
#define SWIFTNAV_TRACK_CN0_H

#include <libswiftnav/track.h>
#include <nap/nap_constants.h>

/** Fixed SNR offset for converting 1ms C/N0 to SNR */
#define TRACK_CN0_SNR_OFFSET   (-174.f + PLATFORM_NOISE_FIGURE)
/** Converts C/N0 into SNR */
#define TRACK_CN0_TO_SNR(x) ((x) + TRACK_CN0_SNR_OFFSET)
/** Adjusts C/N0 according to noise figure */
#define TRACK_CN0_ADJUST(x) ((float)(x) + 2.f - PLATFORM_NOISE_FIGURE)

#define TRACK_CN0_FLAG_FAST_TYPE (0x80u)

/** C/N0 level above which primary estimator shall be used */
#define TRACK_CN0_SEC2PRI_THRESHOLD  TRACK_CN0_ADJUST(32.f)
/** C/N0 level below which secondary estimator shall be used */
#define TRACK_CN0_PRI2SEC_THRESHOLD  (TRACK_CN0_SEC2PRI_THRESHOLD - 4.f)

/* Configure C/N0 value filter algorithm */
#define cn0_filter_params_t       lp1_filter_params_t
#define cn0_filter_compute_params lp1_filter_compute_params
#define cn0_filter_t              lp1_filter_t
#define cn0_filter_init           lp1_filter_init
#define cn0_filter_update         lp1_filter_update

/**
 * C/N0 estimator types
 */
typedef enum {
  TRACK_CN0_EST_BASIC, /**< Basic estimator */
  TRACK_CN0_EST_PRIMARY = TRACK_CN0_EST_BASIC,
  TRACK_CN0_EST_SECONDARY = TRACK_CN0_EST_BASIC
} track_cn0_est_e;

/**
 * C/N0 estimator parameters.
 */
typedef struct
{
  cn0_est_params_t    est_params;    /**< C/N0 estimator algorithm */
  cn0_filter_params_t filter_params; /**< Additional C/N0 value LP filter */
} track_cn0_params_t;

/**
 * C/N0 estimator state.
 */
typedef struct
{
  u32 cn0_0: 8;                /**< Initial C/N0 for bootstrap */
  u32 flags: 8;                /**< Configuration flags */
  u32 cn0_ms: 6;               /**< C/N0 filter interval in ms */
  u32 type: 2;                 /**< Currently used estimator type */
  u32 ver: 8;                  /**< Configuration version */
  cn0_est_basic_state_t basic; /**< Basic estimator for Very Early tap in use */

  /* Other supported estimators for testing:
   * cn0_est_ch_state_t   ch;
   * cn0_est_nwpr_state_t nwpr;
   * cn0_est_rscn_state_t rscn;
   * cn0_est_snv_state_t  snv;
   * cn0_est_svr_state_t  svr;
   */
  cn0_filter_t    filter;      /**< Additional C/N0 filter */
} track_cn0_state_t;

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void track_cn0_params_init(void);
void track_cn0_init(gnss_signal_t sid, u8 cn0_ms,
                    track_cn0_state_t *e, float cn0_0, u8 flags);
float track_cn0_update(gnss_signal_t sid, track_cn0_est_e t,
                       track_cn0_state_t *e,
                       float I, float Q,
                       float ve_I, float ve_Q);
const char *track_cn0_str(track_cn0_est_e t);
float track_cn0_get_offset(u8 cn0_ms);
float track_cn0_get_pri2sec_threshold(u8 cn0_ms);
float track_cn0_get_sec2pri_threshold(u8 cn0_ms);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* SWIFTNAV_TRACK_CN0_H */
