/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_CONS_TIME_STORAGE_H
#define SWIFTNAV_CONS_TIME_STORAGE_H

#include <swiftnav/gnss_time.h>
#include <swiftnav/signal.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  gps_time_t t;
  double a0;
  double a1;
} cons_time_params_t;

void store_cons_time_params(gnss_signal_t sid,
                            const cons_time_params_t *params);
bool get_cons_time_params(gnss_signal_t sid, cons_time_params_t *params);

#ifdef __cplusplus
}
#endif

#endif /* SWIFTNAV_CONS_TIME_STORAGE_H */
