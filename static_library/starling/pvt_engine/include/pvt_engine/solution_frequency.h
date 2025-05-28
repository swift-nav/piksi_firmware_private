/*
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_SOLUTION_FREQUENCY_H
#define STARLING_SOLUTION_FREQUENCY_H

#include <libsbp/common.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  ONE_HERTZ,
  TWO_HERTZ,
  FOUR_HERTZ,
  FIVE_HERTZ,
  TEN_HERTZ,
  TWENTY_HERTZ,
  NUM_OBSERVATION_RATES
} observation_rate_t;

/* Get the rate at which the filter calculates solutions. */
bool get_solution_frequency(double requested_frequency_hz,
                            observation_rate_t *rate);

double get_minimum_solution_frequency(void);

double get_maximum_solution_frequency(void);

/* Get the max sats for the given requested frequency. */
bool get_max_sats_from_requested_freq(double requested_frequency_hz,
                                      s32 *max_sats);

/* Get the max sats for the given observation rate. */
s32 get_max_sats_from_freq_enum(observation_rate_t rate);

#ifdef __cplusplus
}
#endif

#endif  // STARLING_SOLUTION_FREQUENCY_H
