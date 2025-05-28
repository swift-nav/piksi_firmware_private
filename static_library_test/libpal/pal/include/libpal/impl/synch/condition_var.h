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

#ifndef LIBPAL_IMPL_SYNCH_CONDITION_VAR_H
#define LIBPAL_IMPL_SYNCH_CONDITION_VAR_H

#include <stdint.h>

#include <libpal/error.h>
#include <libpal/impl/synch/mutex.h>

#ifdef __cplusplus
extern "C" {
#endif

// PAL Condition Variable Context
typedef void *pal_cv_t;

/**
 * Allocate a condition variable
 *
 * This function must create and initialise a new condition variable. On success
 * it must create a new handle and update the \p cv parameter. This handle will
 * be passed in to future calls to other functions in the module, eventually it
 * will be passed to pal_cv_free.
 *
 * @param cv On success must be updated to a new condition variable handle
 * @return PAL error code
 */
typedef enum pal_error (*pal_cv_alloc_t)(pal_cv_t *cv);

/**
 * Free a previously allocated condition variable
 *
 * Starling will call this function when it no longer needs a previously
 * allocated condition variable. The platform implementation must release all
 * associated resources.
 *
 * @param cv Condition variable handle previously returned from pal_cv_alloc
 * @return PAL error code
 */
typedef enum pal_error (*pal_cv_free_t)(pal_cv_t cv);

/**
 * Notify 1 waiting thread
 *
 * This function must notify (wake up) one and only one thread which is blocked
 * on the condition variable.
 *
 * @param cv Condition variable handle
 * @return PAL error code
 */
typedef enum pal_error (*pal_cv_notify_one_t)(pal_cv_t cv);

/**
 * Notify all waiting threads
 *
 * This function must notify (wake up) all threads currently blocked on the
 * condition variable.
 *
 * @param cv Condition variable handle
 * @return PAL error code
 */
typedef enum pal_error (*pal_cv_notify_all_t)(pal_cv_t cv);

/**
 * Wait on a condition variable
 *
 * This function must block the caller until it is notified (woken up) by
 * another thread calling either pal_cv_notify_one or pal_cv_notify_all.
 *
 * This function will be passed a handle to a mutex which is currently locked by
 * the calling thread. It must unlock the mutex before blocking the current
 * thread on the condition variable and relock the mutex once woken up.
 *
 * @param cv Condition variable handle
 * @param mutex Locked mutex handle
 * @return PAL error code
 */
typedef enum pal_error (*pal_cv_wait_t)(pal_cv_t cv, pal_mutex_t mutex);

/**
 * Wait on a condition variable with timeout
 *
 * This function must block the caller until either it is notified (woken up) by
 * another thread calling either pal_cv_notify_one or pal_cv_notify_all, or the
 * timeout period expires.
 *
 * This function will be passed a handle to a mutex which is currently locked by
 * the calling thread. It must unlock the mutex before blocking the current
 * thread on the condition variable and relock the mutex once woken up.
 *
 * If this function returns because it has been notified it must return
 * PAL_SUCCESS.
 *
 * The \p timeout_us is the microsecond duration this function must block for
 * without being notified before waking up and returning PAL_TIMEOUT.
 *
 * @param cv Condition variable handle
 * @param mutex Locked mutex handle
 * @param timeout_us Maximum blocking duration
 * @return PAL error code
 *
 */
typedef enum pal_error (*pal_cv_wait_for_t)(pal_cv_t cv, pal_mutex_t mutex,
                                            uint64_t timeout_us);

/**
 * PAL condition variable implementation definition
 */
struct pal_impl_cv {
  /// Implementation allocate condition variable routine
  pal_cv_alloc_t alloc;
  /// Implementation free condition variable routine
  pal_cv_free_t free;
  /// Implementation notify 1 waiting thread routine
  pal_cv_notify_one_t notify_one;
  /// Implementation notify all waiting thread routine
  pal_cv_notify_all_t notify_all;
  /// Implementation wait on condition variable routine
  pal_cv_wait_t wait;
  /// Implemention wait on condition variable with timeout routine
  pal_cv_wait_for_t wait_for;
};

/**
 * Install PAL condition variable variable implementation in to API
 *
 * Call this function during pal_impl_init to register the implementation's
 * condition variable module with the libpal API
 *
 * @param impl Condition variable implementation definition
 * @return PAL error code
 */
enum pal_error pal_set_impl_cv(struct pal_impl_cv *impl);

#ifdef __cplusplus
}
#endif

#endif
