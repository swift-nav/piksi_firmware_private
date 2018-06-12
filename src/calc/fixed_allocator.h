/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Kevin Dade <kevin@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef FIXED_ALLOCATOR_H
#define FIXED_ALLOCATOR_H

#include <stddef.h>

typedef struct fixed_allocator_t fixed_allocator_t;

fixed_allocator_t *fixed_allocator_init(size_t block_size, size_t capacity);
void *fixed_allocator_alloc(fixed_allocator_t *self);
void  fixed_allocator_free(fixed_allocator_t *self, void *block);

#endif
