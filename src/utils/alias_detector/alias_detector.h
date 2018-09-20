/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef ALIAS_DETECTOR_H
#define ALIAS_DETECTOR_H

#include <swiftnav/common.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * False phase lock detector.
 *
 * References:
 *  -# Understanding GPS: Principles and Applications, 2nd Edition
 *     Section 5.12, pp 235-236
 *     Elliott D. Kaplan. Artech House, 1996.
 */
typedef struct {
  float k;       /**< Coefficient for converting radians into degrees.
                  * k=1/(2*pi*t), t - time difference between sample points. */
  float dot;     /**< Accumulated dot products. */
  float cross;   /**< Accumulated cross products. */
  float first_I; /**< First in-phase sample. */
  float first_Q; /**< First quadrature-phase sample. */
  u16 acc_len;   /**< Accumulation length. */
  u16 fl_count;  /**< Currently accumulated point count. */
} alias_detect_t;

void alias_detect_init(alias_detect_t *a, u32 acc_len, float time_diff);
void alias_detect_reinit(alias_detect_t *a, u32 acc_len, float time_diff);
void alias_detect_first(alias_detect_t *a, float I, float Q);
float alias_detect_second(alias_detect_t *a, float I, float Q);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* ALIAS_DETECTOR_H */
