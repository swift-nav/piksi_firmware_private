/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_RUN_STATS_H
#define SWIFTNAV_RUN_STATS_H

#include <swiftnav/common.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** Running statistics */
typedef struct {
  u64 n;                 /**< number of samples */
  double sum;            /**< sum of samples */
  double sum_of_squares; /**< sum of samples' squares */
} running_stats_t;

void running_stats_init(running_stats_t *p);
void running_stats_update(running_stats_t *p, double v);
void running_stats_get_products(running_stats_t *p, double *mean, double *std);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* SWIFTNAV_RUN_STATS_H */
