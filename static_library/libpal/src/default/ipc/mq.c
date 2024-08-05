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

#include <libpal/error.h>
#include <libpal/impl/ipc/mq.h>
#include <libpal/ipc/mq.h>
#include <pal_private.h>

#include "not_implemented.h"

static enum pal_error default_mq_alloc(size_t max_length, pal_mq_t *mq) {
  NOT_IMPLEMENTED();
  (void)max_length;
  (void)mq;
  return PAL_INVALID;
}

static enum pal_error default_mq_free(pal_mq_t mq) {
  (void)mq;
  NOT_IMPLEMENTED();
  return PAL_INVALID;
}

static enum pal_error default_mq_push(pal_mq_t mq, void *msg,
                                      enum pal_mq_blocking_mode mode,
                                      uint64_t timeout_us) {
  NOT_IMPLEMENTED();
  (void)mq;
  (void)msg;
  (void)mode;
  (void)timeout_us;
  return PAL_INVALID;
}

static enum pal_error default_mq_pop(pal_mq_t mq, void **msg,
                                     enum pal_mq_blocking_mode mode,
                                     uint64_t timeout_us) {
  NOT_IMPLEMENTED();
  (void)mq;
  (void)msg;
  (void)mode;
  (void)timeout_us;
  return PAL_INVALID;
}

static struct pal_impl_mq mq_impl = {.alloc = default_mq_alloc,
                                     .free = default_mq_free,
                                     .push = default_mq_push,
                                     .pop = default_mq_pop};

void pal_reset_impl_mq(void) {
  mq_impl.alloc = default_mq_alloc;
  mq_impl.free = default_mq_free;
  mq_impl.push = default_mq_push;
  mq_impl.pop = default_mq_pop;
}

void pal_set_impl_mq(struct pal_impl_mq *impl) {
  assert(NULL != impl);
  assert(NULL != impl->alloc);
  assert(NULL != impl->free);
  assert(NULL != impl->push);
  assert(NULL != impl->pop);

  mq_impl = *impl;
}

bool pal_has_impl_mq() { return mq_impl.alloc != default_mq_alloc; }

enum pal_error pal_mq_alloc(size_t max_length, pal_mq_t *mq) {
  return mq_impl.alloc(max_length, mq);
}

enum pal_error pal_mq_free(pal_mq_t *mq) {
  if (!mq) {
    return PAL_INVALID;
  }
  enum pal_error ret = mq_impl.free(*mq);
  if (ret == PAL_SUCCESS) {
    *mq = NULL;
  }
  return ret;
}

enum pal_error pal_mq_push(pal_mq_t mq, void *msg,
                           enum pal_mq_blocking_mode mode,
                           uint64_t timeout_us) {
  return mq_impl.push(mq, msg, mode, timeout_us);
}

enum pal_error pal_mq_pop(pal_mq_t mq, void **msg,
                          enum pal_mq_blocking_mode mode, uint64_t timeout_us) {
  return mq_impl.pop(mq, msg, mode, timeout_us);
}
