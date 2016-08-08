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

#ifndef SRC_BOARD_V2_CN0_EST_H_
#define SRC_BOARD_V2_CN0_EST_H_

#include <libswiftnav/track.h>

#include "track_api.h"

typedef struct  {
  cn0_est_state_t state;
} track_cn0_state_t;

float cn0_estimate(track_cn0_state_t *data,
                   const corr_t* cs,
                   u8 int_ms);
void cn0_est_precompute(void);
void cn0_init(track_cn0_state_t *s, u8 int_ms, float cn0_0);

#endif /* SRC_BOARD_V2_CN0_EST_H_ */
