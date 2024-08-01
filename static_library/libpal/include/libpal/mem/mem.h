/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBPAL_MEM_MEM_H
#define LIBPAL_MEM_MEM_H

#include <libpal/error.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Check Global PAL Memory Implementation
 * @return True if Global PAL Memory Implementation has been set, otherwise
 * False.
 */
bool pal_has_impl_mem(void);

/**
 * Allocate memory from the heap
 *
 * This function will wherever possible allocate the requested amount of memory
 * from the heap and return it to the user. It is the functional equivelant of
 * malloc(). Once memory has been allocated it belong to the caller, it is the
 * callers responsibility to release the memory when it is no longer required by
 * passing the returned pointer to pal_mem_free()
 *
 * @param ptr On success will be updated to point to the first byte in a block
 * that is at least as big as the requested size
 * @param size Requested memory size in bytes
 * @return PAL error code
 */
enum pal_error pal_mem_alloc(void **ptr, size_t size);

/**
 * Free previously allocated memory
 *
 * When access to allocated memory is no longer required call this function to
 * release it back to the heap. The parameter to this function must be a block
 * of memory previously returned from a call to pal_mem_alloc. Failing to call
 * this function at the appropriate time can lead to memory leaks
 *
 * Once the block of memory has been successfully freed the caller's pointer
 * will be set to NULL. Any other references to the memory block maintained by
 * the caller must be considered invalid, it is illegal to try to access it.
 *
 * @param ptr Pointer to memory block allocated by pal_mem_alloc
 * @return PAL error code
 */
enum pal_error pal_mem_free(void **ptr);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // LIBPAL_MEM_H
