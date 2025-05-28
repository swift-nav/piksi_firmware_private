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

#ifndef LIBPAL_REQUIRE_H
#define LIBPAL_REQUIRE_H

#include <assert.h>
#include <stdbool.h>

#include <libpal/error.h>

#ifdef __cplusplus
extern "C" {
#endif

static inline enum pal_error pal_require(bool condition) {
  assert(condition);
  return condition ? PAL_SUCCESS : PAL_INVALID;
}

#ifdef __cplusplus
}
#endif

#endif
