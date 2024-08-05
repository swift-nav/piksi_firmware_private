/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBPAL_SYNCH_CONDITION_VAR_H
#define LIBPAL_SYNCH_CONDITION_VAR_H

#include <stdint.h>

#include <libpal/error.h>
#include <libpal/synch/mutex.h>

#ifdef __cplusplus
extern "C" {
#endif

// PAL Condition Variable Context
typedef void *pal_cv_t;

/**
 * Check Global PAL Condition Variable Implementation
 * @return True if Global PAL Condition Variable Implementation has been set,
 * otherwise False.
 */
bool pal_has_impl_cv(void);

/**
 * Allocate a condition variable
 *
 * Returns a handle to a PAL condition variable which can be used in other
 * functions calls in this module. Once the condition variable is no longer
 * required it must be freed by passing it back to pal_cv_free
 *
 * @param cv On success will be set to a handle representing a newly allocated
 * condition variable
 * @return PAL error code
 */
enum pal_error pal_cv_alloc(pal_cv_t *cv);

/**
 * Free a previously allocated condition variable
 *
 * When a condition variable is no longer required pass it back to this function
 * so the PAL implementation can free any resources associated with it. Failure
 * to do so can result in memory leaks
 *
 * After this function returns SUCCESS the caller's handle will automatically be
 * set to NULL and must be be reused.
 *
 * @param cv Pointer to condition variable handle created by pal_cv_alloc
 * @return PAL error code
 */
enum pal_error pal_cv_free(pal_cv_t *cv);

/**
 * Notify 1 waiting thread
 *
 * If there are any other threads waiting on this condition variable (blocked on
 * a call to pal_cv_wait or pal_cv_wait_for) wake one of them up. The unblocked
 * thread will return from whatever PAL CV function they are in with ownership
 * of their respective mutex
 *
 * @param cv PAL condition variable handle
 * @return PAL error code
 */
enum pal_error pal_cv_notify_one(pal_cv_t cv);

/**
 * Notify all waiting threads
 *
 * Wake up all threads which are waiting on this condition variable via a call
 * to pal_cv_wait or pal_cv_wait_for.
 *
 * @param cv PAL condition variable handle
 * @return PAL error code
 */
enum pal_error pal_cv_notify_all(pal_cv_t cv);

/**
 * Block execution of the current thread until the condition variable is
 * signalled
 *
 * The current thread will be blocked indefinitely until another thread calls
 * pal_cv_notify_one or pal_cv_notify_all for this condition variable.
 *
 * This function must be passed a locked mutex on entry. When the function exit
 * the mutex will still be locked, it is the responsibility of the caller to
 * ensure it is unlocked again in an appropriate fashion. The mutex must have
 * been previously allocated via a call to pal_mutex_alloc()
 *
 * @param cv PAL condition variable handle
 * @param mutex Locked PAL mutex handle
 * @return PAL error code
 */
enum pal_error pal_cv_wait(pal_cv_t cv, pal_mutex_t lock);

/**
 * Block execution of the current thread until the condition variable is
 * signalled or a timeout expires
 *
 * The current thread will be blocked until either another thread calls a notify
 * function for this condition variable, or if the specified timeout period
 * expires.
 *
 * This function must be passed a locked mutex on entry. When the function exits
 * the mutex will still be locked, it is the responsibility of the caller to
 * ensure it is unlocked again in an appropriate fashion. The mutex must have
 * been previously allocated via a call to pal_mutex_alloc()
 *
 * @param cv PAL condition variable handle
 * @param mutex Locked PAL mutex handle
 * @param timeout_us Timeout in microseconds
 * @return PAL error code
 */
enum pal_error pal_cv_wait_for(pal_cv_t cv, pal_mutex_t lock,
                               uint64_t timeout_us);

#ifdef __cplusplus
}
#endif

#endif
