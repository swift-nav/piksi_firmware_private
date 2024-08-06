/*
 * Copyright (C) 2020 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBPAL_IMPL_BLOCKING_MODE_H
#define LIBPAL_IMPL_BLOCKING_MODE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Flags for indicating if an operation should block or not
 */
enum pal_blocking_mode {
  /**
   * The operation will return PAL_WOULD_BLOCK if it is not able to complete
   * immediately
   */
  PAL_NONBLOCKING,

  /**
   * The operation will block according to the timeout parameter of the function
   */
  PAL_BLOCKING,
};

static inline bool pal_validate_blocking_mode(enum pal_blocking_mode mode) {
  switch (mode) {
    case PAL_BLOCKING:
    case PAL_NONBLOCKING:
      return true;

    default:
      return false;
  }
}

#ifdef __cplusplus
}
#endif

#endif
