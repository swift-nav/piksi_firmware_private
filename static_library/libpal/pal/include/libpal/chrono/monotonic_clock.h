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

#ifndef LIBPAL_CHRONO_MONOTONIC_CLOCK_H
#define LIBPAL_CHRONO_MONOTONIC_CLOCK_H

#include <stdbool.h>

#include <libpal/impl/chrono/monotonic_clock.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @return true if monotonic clock module is available / has been setup for the
 * specific platform
 */
bool pal_has_impl_monotonic_clock(void);

/**
 * @param now Number of nanoseconds since the system has booted up
 * @return PAL error code
 */
enum pal_error pal_monotonic_clock_now(uint64_t *now);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // LIBPAL_CHRONO_MONOTONIC_CLOCK_H
