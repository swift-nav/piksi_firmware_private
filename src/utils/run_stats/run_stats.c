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

#include "run_stats.h"

#include <math.h>
#include <stdio.h>

/** Running statistics initializaiton
 * \param p Running statistics state
 */
void running_stats_init(running_stats_t *p) {
  p->n = 0;
  p->sum = 0;
  p->sum_of_squares = 0;
}

/** Running statistics update
 * \param p Running statistics state
 * \param v New value
 */
void running_stats_update(running_stats_t *p, double v) {
  p->n++;
  if (0 == p->n) {
    /* Overflow just happened. Reset statistics by intializing it. */
    running_stats_init(p);
    p->n = 1;
  }

  p->sum += v;
  p->sum_of_squares += v * v;
}

/** Running statistics update
 * \param p Running statistics state
 * \param[out] mean Mean value
 * \param[out] std Standard deviation
 */
void running_stats_get_products(running_stats_t *p, double *mean, double *std) {
  if (NULL != mean) {
    if (0 == p->n) {
      *mean = 0;
    } else {
      *mean = p->sum / p->n;
    }
  }

  if (NULL != std) {
    if (1 >= p->n) {
      *std = 0;
    } else {
      double variance = p->sum_of_squares / (p->n - 1) -
                        (p->sum * p->sum) / p->n / (p->n - 1);
      *std = sqrt(variance);
    }
  }
}
