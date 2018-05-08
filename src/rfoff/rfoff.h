/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_RFOFF_H
#define SWIFTNAV_RFOFF_H

#include <libswiftnav/signal.h>
#include "board/nap/nap_common.h"
#include "run_stats/run_stats.h"

typedef struct rfoff {
  running_stats_t noise;
  running_stats_t signal;
  double signal_now;
  double noise_now;
  double signal_tau3_ms;
  u8 int_ms;
  bool rfoff;
  u8 rfoff_countdown;
} rfoff_t;

void rfoff_init(rfoff_t *self);
bool rfoff_detected(rfoff_t *self,
                   u8 int_ms,
                   const corr_t *ve,
                   const corr_t *e,
                   const corr_t *p,
                   const corr_t *l);

#endif /* SWIFTNAV_RFOFF_H  */
