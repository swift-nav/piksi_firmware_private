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

#include <calc/object_pool.h>

#include <libswiftnav/logging.h>

#include <ch.h>
#include <stdint.h>

#define OBJECT_POOL_MAX_COUNT        8

/* Limit the static memory arena from which all object pool instances draw
 * their buffers. */
#define OBJECT_POOL_ARENA_SIZE_BYTES (2*1024)

static uint8_t arena[OBJECT_POOL_ARENA_SIZE_BYTES];

#define arena_start ((uintptr_t)arena)
#define arena_end   ((uintptr_t)arena + OBJECT_POOL_ARENA_SIZE_BYTES)

struct object_pool_t {
  memory_pool_t chibios_pool;  
};

static object_pool_t object_pools[OBJECT_POOL_MAX_COUNT];

static int num_object_pools_in_use = 0;
static uintptr_t arena_ptr = 0;

/******************************************************************************/
static size_t get_remaining_arena_bytes(void) {
  if (arena_ptr >= arena_end) {
    return 0;
  } else {
    return arena_end - arena_ptr;
  }
}

/******************************************************************************/
object_pool_t *object_pool_init(size_t object_size, size_t capacity) {
  if (num_object_pools_in_use >= OBJECT_POOL_MAX_COUNT) {
    log_error("Object Pool Error: no more allocators available.");
    return NULL;
  }
  const size_t buffer_size = capacity * object_size;
  if (buffer_size > get_remaining_arena_bytes()) {
    log_error("Object Pool Error: out of memory.");
    return NULL; 
  }
  object_pool_t *pool = &object_pools[num_object_pools_in_use++];
  chPoolObjectInit(&pool->chibios_pool, object_size, NULL);    

  if (0 == num_object_pools_in_use) {
    arena_ptr = MEM_ALIGN_NEXT(arena_start);
  }
  chPoolLoadArray(&pool->chibios_pool, (void *)arena_ptr, capacity);
  arena_ptr = MEM_ALIGN_NEXT(arena_ptr + buffer_size);

  return pool;
}

/******************************************************************************/
void *object_pool_alloc(object_pool_t *self) {
  return chPoolAlloc(&self->chibios_pool);
}

/******************************************************************************/
void object_pool_free(object_pool_t *self, void *object) {
  chPoolFree(&self->chibios_pool, object); 
}


