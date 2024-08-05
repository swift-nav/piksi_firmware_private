/**
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

#ifndef LIBPAL_IMPL_CHRONO_MONOTONIC_CLOCK_H
#define LIBPAL_IMPL_CHRONO_MONOTONIC_CLOCK_H

#include <libpal/chrono/monotonic_clock.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Function signature for getting the number of nanoseconds since the
 * system has booted.
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_monotonic_clock_now()
 *
 * @see libpal/chrono/monotonic_clock.h
 */
typedef enum pal_error (*pal_monotonic_clock_now_t)(uint64_t *now);

/**
 * PAL monotonic clock implementation definition
 */
struct pal_impl_monotonic_clock {
  pal_monotonic_clock_now_t monotonic_clock_now;
};

/**
 * Install PAL monotonic clock implementation into API
 *
 * Call this function during pal_impl_init to register the implementation's
 * monotonic clock module with the libpal API
 *
 * @param impl Monotonic clock implementation definition
 */
void pal_set_impl_monotonic_clock(struct pal_impl_monotonic_clock *impl);

#ifdef __cplusplus
}
#endif

#endif  // LIBPAL_IMPL_CHRONO_MONOTONIC_CLOCK_H
