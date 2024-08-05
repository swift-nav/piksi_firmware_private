/**
 * Copyright (C) 2020 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBPAL_IMPL_WATCHDOG_WATCHDOG_H
#define LIBPAL_IMPL_WATCHDOG_WATCHDOG_H

#include <libpal/watchdog/watchdog.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * PAL watchdog implementation
 *
 * This file defines the interace a PAL implementation must use to install its
 * own watchdog notify ability into the libpal API. The function pointer names
 * and signatures in this file match those in libpal/watchdog/watchdog.h. The
 * PAL implementation must provide a version of these functions which meet the
 * requirements stated in the documentation contained in that file.
 */

/** Notify the watchdog timer
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_watchdog_notify() (see libpal/watchdog/watchdog.h)
 *
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
 */
void pal_set_impl_watchdog(struct pal_impl_watchdog *impl);

#ifdef __cplusplus
}
#endif

#endif  // LIBPAL_IMPL_WATCHDOG_WATCHDOG_H
