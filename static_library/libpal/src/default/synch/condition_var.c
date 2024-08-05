/*
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#include <stddef.h>

#include <libpal/error.h>
#include <libpal/impl/synch/condition_var.h>
#include <libpal/synch/condition_var.h>
#include <pal_private.h>

#include "not_implemented.h"

/**
 * Default Condition Variable Allocation Routine
 * @param cv Unused
 * @return PAL_INVALID
 */
static enum pal_error default_cv_alloc(pal_cv_t *cv) {
  (void)cv;
  NOT_IMPLEMENTED();
  return PAL_INVALID;
}

/**
 * Default Condition Variable Free Routine
 * @param cv unused
 */
static enum pal_error default_cv_free(pal_cv_t cv) {
  (void)cv;
  NOT_IMPLEMENTED();
  return PAL_INVALID;
}

/**
 * Default Condition Variable Notify One Routine
 * @param cv unused
 */
static enum pal_error default_cv_notify_one(pal_cv_t cv) {
  (void)cv;
  NOT_IMPLEMENTED();
  return PAL_INVALID;
}

/**
 * Default Condition Variable Notify All Routine
 * @param cv unused
 */
static enum pal_error default_cv_notify_all(pal_cv_t cv) {
  (void)cv;
  NOT_IMPLEMENTED();
  return PAL_INVALID;
}

/**
 * Default Condition Variable Wait Routine
 * @param cv unused
 * @param lock unused
 */
static enum pal_error default_cv_wait(pal_cv_t cv, pal_mutex_t lock) {
  (void)cv;
  (void)lock;
  NOT_IMPLEMENTED();
  return PAL_INVALID;
}

/**
 * Default Condition Variable Wait For Routine
 * @param cv unused
 * @param lock unused
 * @param timeout_us unused
 * @return PAL_INVALID
 */
static enum pal_error default_cv_wait_for(pal_cv_t cv, pal_mutex_t lock,
                                          uint64_t timeout_us) {
  (void)cv;
  (void)lock;
  (void)timeout_us;
  NOT_IMPLEMENTED();
  return PAL_INVALID;
}

static struct pal_impl_cv cv_impl = {
    .alloc = default_cv_alloc,
    .free = default_cv_free,
    .notify_one = default_cv_notify_one,
    .notify_all = default_cv_notify_all,
    .wait = default_cv_wait,
    .wait_for = default_cv_wait_for,
};

void pal_reset_impl_cv(void) {
  cv_impl.alloc = default_cv_alloc;
  cv_impl.free = default_cv_free;
  cv_impl.notify_one = default_cv_notify_one;
  cv_impl.notify_all = default_cv_notify_all;
  cv_impl.wait = default_cv_wait;
  cv_impl.wait_for = default_cv_wait_for;
}

void pal_set_impl_cv(struct pal_impl_cv *impl) {
  assert(NULL != impl);
  assert(NULL != impl->alloc);
  assert(NULL != impl->free);
  assert(NULL != impl->notify_one);
  assert(NULL != impl->notify_all);
  assert(NULL != impl->wait);
  assert(NULL != impl->wait_for);

  cv_impl = *impl;
}

bool pal_has_impl_cv() { return cv_impl.alloc != default_cv_alloc; }

enum pal_error pal_cv_alloc(pal_cv_t *cv) { return cv_impl.alloc(cv); }

enum pal_error pal_cv_free(pal_cv_t *cv) {
  if (!cv) {
    return PAL_INVALID;
  }
  enum pal_error ret = cv_impl.free(*cv);
  if (ret == PAL_SUCCESS) {
    *cv = NULL;
  }
  return ret;
}

enum pal_error pal_cv_notify_one(pal_cv_t cv) { return cv_impl.notify_one(cv); }

enum pal_error pal_cv_notify_all(pal_cv_t cv) { return cv_impl.notify_all(cv); }

enum pal_error pal_cv_wait(pal_cv_t cv, pal_mutex_t lock) {
  return cv_impl.wait(cv, lock);
}

enum pal_error pal_cv_wait_for(pal_cv_t cv, pal_mutex_t lock,
                               uint64_t timeout_us) {
  return cv_impl.wait_for(cv, lock, timeout_us);
}
