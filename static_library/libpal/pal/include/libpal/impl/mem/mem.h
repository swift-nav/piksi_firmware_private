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

#ifndef LIBPAL_IMPL_MEM_MEM_H
#define LIBPAL_IMPL_MEM_MEM_H

#include <stddef.h>

#include <libpal/error.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Allocate a block of memory
 *
 * Starling will call this function to allocate arbitrarily sized blocks of
 * memory from the heap.
 *
 * On success this function must update \p ptr to point to a block of memory
 * which is at least \p size bytes long. The caller must be granted exclusive
 * access to this block
 *
 * If size is 0 then implementation must set *ptr either to NULL a pointer which
 * can later be successfully passed in to pal_mem_free() and return PAL_SUCCESS.
 *
 * If the platform implementation is unable to fulfil the request it must return
 * PAL_OOM
 *
 * @param ptr On success must be updated to point to the newly allocated memory
 * block
 * @param size Requested memory block size
 * @return PAL error code
 */
typedef enum pal_error (*pal_mem_alloc_t)(void **ptr, size_t size);

/**
 * Free a previously allocated block of memory
 *
 * Starling will call this function to release a block of memory previously
 * allocated by a call to pal_mem_alloc. Once freed starling will no longer
 * access the memory block. The platform implementation must release the
 * resource.
 *
 * @param ptr Pointer previously returned from pal_mem_alloc
 * @return PAL error code
 */
typedef enum pal_error (*pal_mem_free_t)(void *ptr);

/**
 * PAL memory management implementation definition
 */
struct pal_impl_mem {
  /// Implementation allocate memory routine
  pal_mem_alloc_t alloc;
  /// Implementation free memory routine
  pal_mem_free_t free;
};

/**
 * Install PAL memory management implementation in to API
 *
 * Call this function during pal_impl_init to register the implementation's
 * memory management module with the libpal API
 *
 * @param impl Memory management implementation definition
 * @return PAL error code
 */
enum pal_error pal_set_impl_mem(struct pal_impl_mem *impl);

#ifdef __cplusplus
}
#endif

#endif
