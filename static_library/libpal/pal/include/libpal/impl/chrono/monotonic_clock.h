/*
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBPAL_IMPL_CHRONO_MONOTONIC_CLOCK_H
#define LIBPAL_IMPL_CHRONO_MONOTONIC_CLOCK_H

#include <stdint.h>

#include <libpal/error.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Function signature for getting the number of nanoseconds since the
 * system has booted.
 *
 * This function must set the value pointed to by \p now to the number of
 * nanoseconds since some epoch defined by the system. The timer on which this
 * call is based must be a monotonically increasing clock, consecutive calls to
 * this function must return ever increasing numbers.
 *
 * @param now On success the implementation must set this to the value of the
 * system monotonic clock
 * @return PAL error code
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
 * @return PAL error code
 */
enum pal_error pal_set_impl_monotonic_clock(
    struct pal_impl_monotonic_clock *impl);

#ifdef __cplusplus
}
#endif

#endif  // LIBPAL_IMPL_CHRONO_MONOTONIC_CLOCK_H
