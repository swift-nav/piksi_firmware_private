/*
 * Copyright (C) 2020 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBPAL_WATCHDOG_WATCHDOG_H
#define LIBPAL_WATCHDOG_WATCHDOG_H

#include <stdbool.h>
#include <stddef.h>

#include <libpal/error.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Check Global PAL Watchdog Implementation
 * @return True if Global PAL Watchdog Implementation has been set, otherwise
 * False.
 */
bool pal_has_impl_watchdog(void);

/**
 * Notify the watchdog
 *
 * This function will reset the platform watchdog timer, if it exists.
 *
 * @return PAL error code
 */
enum pal_error pal_watchdog_notify(void);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // LIBPAL_WATCHDOG_WATCHDOG_H;
