/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBPAL_MUTEX_H
#define LIBPAL_MUTEX_H

#include <stdbool.h>

#include <libpal/impl/synch/mutex.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Check Global PAL Mutex Implementation
 * @return True if Global PAL Mutex Implementation has been set, otherwise
 * False.
 */
bool pal_has_impl_mutex(void);

/**
 * Allocate a PAL mutex
 *
 * Returns a handle representing a mutex which can be used with other functions
 * in this module. Once the mutex is no longer needed it must be released by
 * passing the returned handle to pal_mutex_free. Failure to do so could result
 * in memory leaks.
 *
 * @param mutex On success will be set to a handle representing a newly
 * allocated mutex
 * @return PAL error code
 */
enum pal_error pal_mutex_alloc(pal_mutex_t *mutex);

/**
 * Free a previously allocated mutex
 *
 * Releases all resources used by this mutex. Once this function returns success
 * the caller's handle will be automatically set to NULL and must not be reused.
 *
 * @param mutex Pointer to mutex handle returned from pal_mutex_alloc
 * @return PAL error code
 */
enum pal_error pal_mutex_free(pal_mutex_t *mutex);

/**
 * Lock a mutex
 *
 * When this function returns the specified mutex will be locked. Any other
 * thread which tries to lock the same mutex will be blocked until the caller of
 * this function unlocks the mutex with a call to pal_mutex_unlock. If when this
 * function is called another thread has already locked the given mutex this
 * thread will be blocked until the other thread unlocks it.
 *
 * It is guaranteed that when this function returns the mutex will be locked.
 * Take great care to unlock the mutex in an appropriate manner, failure to do
 * so can result in a deadlocked system.
 *
 * @param mutex PAL mutex handle
 * @return PAL error code
 */
enum pal_error pal_mutex_lock(pal_mutex_t mutex);

/**
 * Unlock a mutex
 *
 * This function must be given a mutex handle which was previously locked by a
 * call to pal_mutex_lock. When the function returns the mutex will be unlocked.
 * Any other thread which was blocked trying to lock this mutex will be able to
 * wake up and take ownership.
 *
 * @param mutex Locked PAL mutex handle
 * @return PAL error code
 */
enum pal_error pal_mutex_unlock(pal_mutex_t mutex);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // LIBPAL_MUTEX_H
