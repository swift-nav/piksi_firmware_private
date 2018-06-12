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

#ifndef OBJECT_POOL_H
#define OBJECT_POOL_H

#include <stddef.h>

typedef struct object_pool_t object_pool_t;

object_pool_t *object_pool_init(size_t object_size, size_t capacity);
void *object_pool_alloc(object_pool_t *self);
void  object_pool_free(object_pool_t *self, void *object);

/**
 * Query the implementation for its limitations. Allows some
 * degree of sanity checking at the application level.
 */
size_t object_pool_implementation_get_max_count(void);
size_t object_pool_implementation_get_max_available_bytes(void);

#endif
