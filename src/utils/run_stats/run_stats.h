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

#include <libswiftnav/common.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** Running_Stats statistics */
typedef struct {
  double alpha;
  double mean;
  double variance;
  bool init_done;
} running_stats_t;

void running_stats_init(running_stats_t *p, double alpha);
void running_stats_update(running_stats_t *p, double v);
double running_stats_get_std(running_stats_t *p);
double running_stats_get_mean(running_stats_t *p);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* SWIFTNAV_RUN_STATS_H */
