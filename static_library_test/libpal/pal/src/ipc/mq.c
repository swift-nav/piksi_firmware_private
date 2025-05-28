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
#include <libpal/impl/ipc/mq.h>
#include <libpal/ipc/mq.h>
#include <libpal/require.h>
#include <pal_private.h>

static struct pal_impl_mq mq_impl = {
    .alloc = NULL,
    .free = NULL,
    .push = NULL,
    .pop = NULL,
};

void pal_reset_impl_mq(void) { memset(&mq_impl, 0, sizeof(mq_impl)); }

enum pal_error pal_set_impl_mq(struct pal_impl_mq *impl) {
  enum pal_error ret =
      pal_require(NULL != impl && NULL != impl->alloc && NULL != impl->free &&
                  NULL != impl->push && NULL != impl->pop

      );
  if (ret == PAL_SUCCESS) {
    mq_impl = *impl;
  }
  return ret;
}

bool pal_has_impl_mq() { return mq_impl.alloc != NULL; }

enum pal_error pal_mq_alloc(pal_mq_t *mq, size_t max_length) {
  enum pal_error ret =
      pal_require(pal_has_impl_mq() && mq != NULL && max_length != 0);
  if (ret == PAL_SUCCESS) {
    ret = mq_impl.alloc(mq, max_length);
  }
  return ret;
}

enum pal_error pal_mq_free(pal_mq_t *mq) {
  enum pal_error ret = pal_require(mq != NULL);
  if (ret != PAL_SUCCESS) {
    return ret;
  }
  if (!*mq) {
    return PAL_SUCCESS;
  }
  ret = pal_require(pal_has_impl_mq());
  if (ret == PAL_SUCCESS) {
    ret = mq_impl.free(*mq);
    *mq = NULL;
  }
  return ret;
}

enum pal_error pal_mq_push(pal_mq_t mq, void *msg, enum pal_blocking_mode mode,
                           uint64_t timeout_us) {
  enum pal_error ret = pal_require(pal_has_impl_mq() && mq != NULL &&
                                   pal_validate_blocking_mode(mode));
  if (ret == PAL_SUCCESS) {
    ret = mq_impl.push(mq, msg, mode, timeout_us);
  }
  return ret;
}

enum pal_error pal_mq_pop(pal_mq_t mq, void **msg, enum pal_blocking_mode mode,
                          uint64_t timeout_us) {
  enum pal_error ret =
      pal_require(pal_has_impl_mq() && mq != NULL && msg != NULL &&
                  pal_validate_blocking_mode(mode));
  if (ret == PAL_SUCCESS) {
    ret = mq_impl.pop(mq, msg, mode, timeout_us);
  }
  return ret;
}
