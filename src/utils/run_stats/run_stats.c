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

#include <math.h>
#include <string.h>
#include <assert.h>
#include "run_stats.h"

/* A filter-like adaptation of B. P. Welford running statistics algorithm
   presented in Donald Knuth’s Art of Computer Programming, Vol 2, page 232,
   3rd edition.
   The idea is taken from https://www.embeddedrelated.com/showarticle/785.php */

/**
 * Running statistics initializaiton
 * \param p Running statistics state
 * \param alpha First order IIR filter parameter
 */
void running_stats_init(running_stats_t *p, double alpha) {
  memset(p, 0, sizeof(*p));
  p->alpha = alpha;
}

/**
 * Running statistics update
 * \param p Running statistics state
 * \param v New value
 */
void running_stats_update(running_stats_t *p, double v) {
  if (!p->init_done) {
    p->mean = v;
    p->init_done = true;
    return;
  }
  double mean = p->mean + p->alpha * (v - p->mean);
  p->variance += p->alpha * ((v - p->mean) * (v - mean) - p->variance);
  assert(p->variance >= 0);
  p->mean = mean;
}

/**
 * Get standard deviation value
 * \param p Running statistics state
 * \return the standard deviation value
 */
double running_stats_get_std(running_stats_t *p) {
  return sqrt(p->variance);
}

/**
 * Get mean value
 * \param p Running statistics state
 * \return the mean value
 */
double running_stats_get_mean(running_stats_t *p) {
  return p->mean;
}
