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
/** Converts CN0 into SNR */
#define TRACK_CN0_TO_SNR(x) ((x) + TRACK_CN0_SNR_OFFSET)
/** Adjusts C/N0 according to noise figure */
#define TRACK_CN0_ADJUST(x) ((float)(x) + 2.f - PLATFORM_NOISE_FIGURE)

/** C/N0 level above which primary estimator shall be used */
#define TRACK_CN0_SEC2PRI_THRESHOLD  TRACK_CN0_ADJUST(45.f)
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
  TRACK_CN0_EST_RSCN, /**< Real Signal-Complex Noise method */
  TRACK_CN0_EST_BL,   /**< Beauliu's method */
  TRACK_CN0_EST_SNV,  /**< Signal to noise variance method */
  TRACK_CN0_EST_MM,   /**< Moment method (M2M4) */
  TRACK_CN0_EST_NWPR, /**< Narrowband-Wideband Power Ratio method */
  TRACK_CN0_EST_SVR,  /**< ?? */
  TRACK_CN0_EST_CH,   /**< ?? */
  TRACK_CN0_EST_PRIMARY = TRACK_CN0_EST_BL,
  TRACK_CN0_EST_SECONDARY = TRACK_CN0_EST_MM
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
  cn0_est_state_t primary;   /**< Estimator for high SNR values */
  cn0_est_state_t secondary; /**< Estimator for low SNR values */
  cn0_filter_t    filter;    /**< Additional C/N0 filter */
} track_cn0_state_t;

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void track_cn0_params_init(void);
void track_cn0_init(u8 int_ms, track_cn0_state_t *e, float cn0_0);
float track_cn0_update(track_cn0_est_e t, u8 int_ms, track_cn0_state_t *e,
                       float I, float Q);
const char *track_cn0_str(track_cn0_est_e t);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* SWIFTNAV_TRACK_CN0_H */
