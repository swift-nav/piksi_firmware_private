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

#ifndef LIBPAL_IMPL_SYNCH_MUTEX_H
#define LIBPAL_IMPL_SYNCH_MUTEX_H

#include <libpal/synch/mutex.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * PAL Mutex Implementation
 *
 * This file defines the interface a PAL implementation must use to install its
 * own mutex ability in to the libpal API
 *
 * The function pointer names and signatures in this file match those in
 * libpal/synch/mutex.h. The PAL implementation must provide a version of these
 * functions which meet the requirements stated in the documentation contained
 * in that file.
 */

/**
 * Allocate a mutex
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_mutex_alloc() (see libpal/synch/mutex.h)
 *
 */
typedef enum pal_error (*pal_mutex_alloc_t)(pal_mutex_t *mutex);

/**
 * Free a mutex
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_mutex_free() (see libpal/synch/mutex.h)
 *
 */
typedef enum pal_error (*pal_mutex_free_t)(pal_mutex_t mutex);

/**
 * Lock a mutex
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_mutex_lock() (see libpal/synch/mutex.h)
 *
 */
typedef enum pal_error (*pal_mutex_lock_t)(pal_mutex_t mutex);

/**
 * Unlock a mutex
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_mutex_unlock() (see libpal/synch/mutex.h)
 *
 */
typedef enum pal_error (*pal_mutex_unlock_t)(pal_mutex_t mutex);

/**
 * PAL mutex implementation definition
 */
struct pal_impl_mutex {
  /// Implementation allocate mutex routine
  pal_mutex_alloc_t alloc;
  /// Implementation free mutex routine
  pal_mutex_free_t free;
  /// Implementation lock mutex routine
  pal_mutex_lock_t lock;
  /// Implementation unlock mutex routine
  pal_mutex_unlock_t unlock;
};

/**
 * Install PAL mutex implementation in to API
 *
 * Call this function during pal_impl_init to register the implementation's
 * mutex module with the libpal API
 *
 * @param impl Mutex implementation definition
 */
void pal_set_impl_mutex(struct pal_impl_mutex *impl);

#ifdef __cplusplus
}
#endif

#endif
