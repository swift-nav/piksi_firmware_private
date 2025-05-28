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

#ifndef LIBPAL_IMPL_WATCHDOG_WATCHDOG_H
#define LIBPAL_IMPL_WATCHDOG_WATCHDOG_H

#include <libpal/error.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Notify the watchdog timer
 *
 * @return PAL error code
 */
typedef enum pal_error (*pal_watchdog_notify_t)(void);

/**
 * PAL watchdog implmentation definition
 */
struct pal_impl_watchdog {
  // Implementation watchdog notify routine
  pal_watchdog_notify_t notify;
};

/**
 * Install PAL watchdog implementation in to API
 *
 * Call this function during pal_impl_init to register the implementation's
 * watchdog module with the libpal API
 *
 * @param impl Watchdog implementation definition
 * @return PAL error code
 */
enum pal_error pal_set_impl_watchdog(struct pal_impl_watchdog *impl);

#ifdef __cplusplus
}
#endif

#endif  // LIBPAL_IMPL_WATCHDOG_WATCHDOG_H
