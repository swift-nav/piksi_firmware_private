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

#ifndef SWIFTNAV_LOCKD_H
#define SWIFTNAV_LOCKD_H

/**
 * Basic exponential smoothing filter
 */
struct lockd_lpf {
  float y; /**< Output and state variable. */
};

/** State structure for basic lock detector with optimistic and pessimistic
 *  indicators.
 */
typedef struct {
  struct lockd_lpf lpfs; /**< Siganl path LPF state. */
  struct lockd_lpf lpfn; /**< Noise path LPF state. */
  float fc;              /**< Filter coefficient. */
  float scale;           /**< Signal scale factor. */
  u16 cnt_threshold;     /**< Lock count threshold. */
  u16 cnt;               /**< Counter state variable. */
  bool locked;           /**< Lock state indicator. */
} lockd_t;

void lockd_init(lockd_t *l, float fc, float scale, u16 cnt_threshold);
void lockd_reinit(lockd_t *l, float fc, float scale, u16 cnt_threshold);
void lockd_update(lockd_t *l, float signal, float noise);

#endif /* SWIFTNAV_LOCKD_H */
