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

#include <calc/fixed_allocator.h>

#include <libswiftnav/logging.h>

#include <ch.h>
#include <stdint.h>

#define FIXED_ALLOCATOR_MAX_COUNT        8

#define FIXED_ALLOCATOR_ARENA_SIZE_BYTES (2*1024)

static uint8_t arena[FIXED_ALLOCATOR_ARENA_SIZE_BYTES];

#define arena_start ((uintptr_t)arena)
#define arena_end   ((uintptr_t)arena + FIXED_ALLOCATOR_ARENA_SIZE_BYTES)

struct fixed_allocator_t {
  memory_pool_t chibios_pool;  
};

static fixed_allocator_t fixed_allocators[FIXED_ALLOCATOR_MAX_COUNT];

static int num_fixed_allocators_in_use = 0;
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
fixed_allocator_t *fixed_allocator_init(size_t block_size, size_t capacity) {
  if (num_fixed_allocators_in_use >= FIXED_ALLOCATOR_MAX_COUNT) {
    log_error("Fixed Allocator Error: no more allocators available.");
    return NULL;
  }
  const size_t buffer_size = capacity * block_size;
  if (buffer_size > get_remaining_arena_bytes()) {
    log_error("Fixed Allocator Error: out of memory.");
    return NULL; 
  }
  fixed_allocator_t *allocator = &fixed_allocators[num_fixed_allocators_in_use++];
  chPoolObjectInit(&allocator->chibios_pool, block_size, NULL);    

  if (0 == num_fixed_allocators_in_use) {
    arena_ptr = MEM_ALIGN_NEXT(arena_start);
  }
  chPoolLoadArray(&allocator->chibios_pool, (void *)arena_ptr, capacity);
  arena_ptr = MEM_ALIGN_NEXT(arena_ptr + buffer_size);

  return allocator;
}

/******************************************************************************/
void *fixed_allocator_alloc(fixed_allocator_t *self) {
  return chPoolAlloc(&self->chibios_pool);
}

/******************************************************************************/
void fixed_allocator_free(fixed_allocator_t *self, void *block) {
  chPoolFree(&self->chibios_pool, block); 
}


