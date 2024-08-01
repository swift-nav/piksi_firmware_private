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

#ifndef LIBPAL_IMPL_MEM_MEM_H
#define LIBPAL_IMPL_MEM_MEM_H

#include <libpal/mem/mem.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * PAL memory management implementation
 *
 * This file defines the interface a PAL implementation must use to install its
 * own memory management ability in to the libpal API. The function pointer
 * names and signatures in this file match those in libpal/mem/mem.h. The PAL
 * implementation must provide a version of these functions which meet the
 * requirements stated in the documentation contained in that file.
 */

/**
 * Allocate a block of memory
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_mem_alloc() (see libpal/mem/mem.h)
 *
 * If size is 0 then implementation can set *ptr either to NULL a pointer which
 * can later be successfully passed in to pal_mem_free()
 *
 */
typedef enum pal_error (*pal_mem_alloc_t)(void **ptr, size_t size);

/**
 * Free a previously allocated block of memory
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_mem_free() (see libpal/mem/mem.h)
 *
 * If ptr is NULL the implementation must take no action.
 *
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
 */
void pal_set_impl_mem(struct pal_impl_mem *impl);

#ifdef __cplusplus
}
#endif

#endif
