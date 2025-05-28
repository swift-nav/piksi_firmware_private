/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_MEMCPY_S_H
#define LIBSWIFTNAV_MEMCPY_S_H

#if defined(_MSC_VER)
#include <memory.h>
#elif !defined(_CRT_MEMORY_DEFINED)
#include <assert.h>

#include <swiftnav/logging.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  MEMCPY_S_OK,
  MEMCPY_S_DEST_NULL,
  MEMCPY_S_SRC_NULL,
  MEMCPY_S_DEST_SIZE_ZERO,
  MEMCPY_S_SRC_SIZE_ZERO,
  MEMCPY_S_OVERLAP,
  MEMCPY_S_OVERSIZED
} memcpy_s_t;

memcpy_s_t memcpy_s(void *dest, size_t destsize, const void *src, size_t count);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* _CRT_MEMORY_DEFINED */

#define MEMCPY_S(d, ds, src, c)                                \
  do {                                                         \
    int memcpy_s_res = memcpy_s(d, ds, src, c);                \
    if (0 != memcpy_s_res) {                                   \
      log_error("MEMCPY_S failed with code %d", memcpy_s_res); \
      assert(false);                                           \
    }                                                          \
  } while (false)

#endif /* LIBSWIFTNAV_MEMCPY_S_H */
