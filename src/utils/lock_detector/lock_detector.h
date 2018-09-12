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

#ifndef LOCK_DETECTOR_H
#define LOCK_DETECTOR_H

#include <stdbool.h>
#include <swiftnav/common.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * Basic exponential smoothing filter
 */
struct loop_detect_lpf {
  float y; /**< Output and state variable. */
};

/** State structure for basic lock detector with optimistic and pessimistic
 *  indicators.
 */
typedef struct {
  struct loop_detect_lpf lpfi; /**< I path LPF state. */
  struct loop_detect_lpf lpfq; /**< Q path LPF state. */
  float k1;                    /**< Filter coefficient. */
  float k2;                    /**< I Scale factor. */
  u16 lo, lp;           /**< Optimistic and pessimistic count threshold. */
  u16 pcount1, pcount2; /**< Counter state variables. */
  bool outo, outp;      /**< Optimistic and pessimistic indicator. */
} lock_detect_t;

void lock_detect_init(lock_detect_t *l, float k1, float k2, u16 lp, u16 lo);
void lock_detect_reinit(lock_detect_t *l, float k1, float k2, u16 lp, u16 lo);
void lock_detect_update(lock_detect_t *l, float I, float Q, float DT);
void freq_lock_detect_update(lock_detect_t *l, float err);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* LOCK_DETECTOR_H */
