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

#ifndef LIBPAL_IMPL_SYNCH_CONDITION_VAR_H
#define LIBPAL_IMPL_SYNCH_CONDITION_VAR_H

#include <libpal/synch/condition_var.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * PAL condition variable implementation
 *
 * This file defines the interface a PAL implementation must use to install its
 * own condition variable ability in to the libpal API
 *
 * The function pointer names and signatures in this file match those in
 * libpal/synch/cv.h. The PAL implementation must provide a version of these
 * functions which meet the requirements stated in the documentation contained
 * in that file.
 */

/**
 * Allocate a condition variable
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_cv_alloc() (see libpal/synch/cv.h)
 *
 */
typedef enum pal_error (*pal_cv_alloc_t)(pal_cv_t *cv);

/**
 * Free a previously allocated condition variable
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_cv_free() (see libpal/synch/cv.h)
 *
 */
typedef enum pal_error (*pal_cv_free_t)(pal_cv_t cv);

/**
 * Notify 1 waiting thread
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_cv_notify_one() (see libpal/synch/cv.h)
 *
 */
typedef enum pal_error (*pal_cv_notify_one_t)(pal_cv_t cv);

/**
 * Notify all waiting threads
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_cv_notify_all() (see libpal/synch/cv.h)
 *
 */
typedef enum pal_error (*pal_cv_notify_all_t)(pal_cv_t cv);

/**
 * Wait on a condition variable
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_cv_wait() (see libpal/synch/cv.h)
 *
 */
typedef enum pal_error (*pal_cv_wait_t)(pal_cv_t cv, pal_mutex_t lock);

/**
 * Wait on a condition variable with timeout
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_cv_wait_for() (see libpal/synch/cv.h)
 *
 */
typedef enum pal_error (*pal_cv_wait_for_t)(pal_cv_t cv, pal_mutex_t lock,
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
 */
void pal_set_impl_cv(struct pal_impl_cv *impl);

#ifdef __cplusplus
}
#endif

#endif
