/*
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#include <string.h>

#include <libpal/error.h>
#include <libpal/impl/synch/condition_var.h>
#include <libpal/require.h>
#include <libpal/synch/condition_var.h>
#include <pal_private.h>

static struct pal_impl_cv cv_impl = {
    .alloc = NULL,
    .free = NULL,
    .notify_one = NULL,
    .notify_all = NULL,
    .wait = NULL,
    .wait_for = NULL,
};

void pal_reset_impl_cv(void) { memset(&cv_impl, 0, sizeof(cv_impl)); }

enum pal_error pal_set_impl_cv(struct pal_impl_cv *impl) {
  enum pal_error ret =
      pal_require(NULL != impl && NULL != impl->alloc && NULL != impl->free &&
                  NULL != impl->notify_one && NULL != impl->notify_all &&
                  NULL != impl->wait && NULL != impl->wait_for);

  if (ret == PAL_SUCCESS) {
    cv_impl = *impl;
  }
  return ret;
}

bool pal_has_impl_cv() { return cv_impl.alloc != NULL; }

enum pal_error pal_cv_alloc(pal_cv_t *cv) {
  enum pal_error ret = pal_require(pal_has_impl_cv() && cv != NULL);
  if (ret == PAL_SUCCESS) {
    ret = cv_impl.alloc(cv);
  }
  return ret;
}

enum pal_error pal_cv_free(pal_cv_t *cv) {
  enum pal_error ret = pal_require(cv != NULL);
  if (ret != PAL_SUCCESS) {
    return ret;
  }
  if (!*cv) {
    return PAL_SUCCESS;
  }
  ret = pal_require(pal_has_impl_cv());
  if (ret == PAL_SUCCESS) {
    ret = cv_impl.free(*cv);
    *cv = NULL;
  }
  return ret;
}

enum pal_error pal_cv_notify_one(pal_cv_t cv) {
  enum pal_error ret = pal_require(pal_has_impl_cv() && cv != NULL);
  if (ret == PAL_SUCCESS) {
    ret = cv_impl.notify_one(cv);
  }
  return ret;
}

enum pal_error pal_cv_notify_all(pal_cv_t cv) {
  enum pal_error ret = pal_require(pal_has_impl_cv() && cv != NULL);
  if (ret == PAL_SUCCESS) {
    ret = cv_impl.notify_all(cv);
  }
  return ret;
}

enum pal_error pal_cv_wait(pal_cv_t cv, pal_mutex_t mutex) {
  enum pal_error ret =
      pal_require(pal_has_impl_cv() && cv != NULL && mutex != NULL);
  if (ret == PAL_SUCCESS) {
    ret = cv_impl.wait(cv, mutex);
  }
  return ret;
}

enum pal_error pal_cv_wait_for(pal_cv_t cv, pal_mutex_t mutex,
                               uint64_t timeout_us) {
  enum pal_error ret =
      pal_require(pal_has_impl_cv() && cv != NULL && mutex != NULL);
  if (ret == PAL_SUCCESS) {
    ret = cv_impl.wait_for(cv, mutex, timeout_us);
  }
  return ret;
}
