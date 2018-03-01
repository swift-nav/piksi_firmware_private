/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * 
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <starling/platform/mutex.h>
#include <ch.h>
#include <stdint.h>
#include <assert.h>

#define MAX_NUM_MUTEXES 32

const unsigned int STARLING_PLATFORM_MAX_NUM_MUTEXES = MAX_NUM_MUTEXES;

static mutex_t  mutex_array[MAX_NUM_MUTEXES];
static uint32_t mutex_array_bitfield = 0; 

_Static_assert(sizeof(mutex_array_bitfield) * 8 >= MAX_NUM_MUTEXES, 
               "Mismatched mutex bitfield size.");

MUTEX_DECL(internal_lock);

static bool is_mutex_in_use(unsigned int i) {
  return mutex_array_bitfield & (1 << i);
}

static void set_mutex_in_use_bit(unsigned int i, bool val) {
  if (val) {
    mutex_array_bitfield |= (1 << i);
  } else {
    mutex_array_bitfield &= ~(1 << i);
  }
}

/**
 * To create a mutex, we look for an available slot and initialize it if possible.
 */
starling_mutex_t* starling_mutex_create(void) {
  mutex_t *mut = NULL;
  chMtxLock(&internal_lock);
  for (unsigned int i = 0; i < MAX_NUM_MUTEXES; ++i) {
    if (!is_mutex_in_use(i)) {
      mut = mutex_array + i;
      chMtxObjectInit(mut);
      set_mutex_in_use_bit(i, true);
      break;
    }
  } 
  chMtxUnlock(&internal_lock);
  return (starling_mutex_t*)mut;
}

/**
 * To destroy a mutex, we simply disable its "in-use" bit.
 */
void starling_mutex_destroy(starling_mutex_t *mut) {
  mutex_t *chibi_mut = (mutex_t*)mut;
  assert(chibi_mut >= mutex_array);
  unsigned int index = chibi_mut - mutex_array; 
  assert(index < MAX_NUM_MUTEXES);
  chMtxLock(&internal_lock);
  set_mutex_in_use_bit(index, false);  
  chMtxUnlock(&internal_lock);
}

void starling_mutex_lock(starling_mutex_t *mut) {
  chMtxLock((mutex_t*)mut);
}

void starling_mutex_unlock(starling_mutex_t *mut) {
  chMtxUnlock((mutex_t*)mut);
}

bool starling_mutex_trylock(starling_mutex_t *mut) {
  return chMtxTryLock((mutex_t*)mut);
}

