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

#ifndef LIBPAL_IMPL_SYNCH_MUTEX_H
#define LIBPAL_IMPL_SYNCH_MUTEX_H

#include <libpal/error.h>

#ifdef __cplusplus
extern "C" {
#endif

// PAL Mutex Context
typedef void *pal_mutex_t;

/**
 * Allocate a mutex
 *
 * This function must create and initialize a new mutex object. On success the
 * \p mutex parameter must be updated with a handle to the newly created mutex.
 * The handle will later be passed to calls to pal_mutex_lock/pal_mutex_unlock
 * and finally pal_mutex_free.
 *
 * @param mutex On success must be updated with a new mutex handle
 * @return PAL error code
 */
typedef enum pal_error (*pal_mutex_alloc_t)(pal_mutex_t *mutex);

/**
 * Free a mutex
 *
 * Starling will call this function to free a previously allocated mutex when it
 * is no longer required. The platform implementation must release all resources
 * associated with the mutex and destroy the handle
 *
 * @param mutex Mutex handle previously returned from pal_mutex_alloc
 * @return PAL error code
 */
typedef enum pal_error (*pal_mutex_free_t)(pal_mutex_t mutex);

/**
 * Lock a mutex
 *
 * Grants the calling thread ownership of the mutex. No more than 1 thread may
 * own the mutex at any point in time. If another thread has already locked the
 * given mutex at the time of calling this function must block the caller until
 * the mutex becomes available for locking.
 *
 * @param mutex Mutex handle previously returned from pal_mutex_alloc
 * @return PAL error code
 */
typedef enum pal_error (*pal_mutex_lock_t)(pal_mutex_t mutex);

/**
 * Unlock a mutex
 *
 * Release owndership of the mutex.
 *
 * @param mutex Mutex handle previously returned from pal_mutex_alloc
 * @return PAL error code
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
 * @return PAL error code
 */
enum pal_error pal_set_impl_mutex(struct pal_impl_mutex *impl);

#ifdef __cplusplus
}
#endif

#endif
