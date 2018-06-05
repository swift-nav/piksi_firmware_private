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

#include "starling_platform.h"

#include <libswiftnav/logging.h>

/**
 * Figure out which subsystem of the PAL this error came from and
 * then print an appropriate message.
 */
void pal_handle_error(pal_rc_t code) {
  if (STARLING_PAL_OK == code) {
    return;
  } 

  /**
  * TODO(kevin) It may make sense to also encode the source of the
  * error into the actual return codes so that the handle error
  * function can reconstruct the information.
  */
  log_error("Error in Starling PAL implementation: " PRIi32, (int32_t)code);
}

/**
 * On the Piksi Multi, all data structures are statically allocated and all
 * we need to do is register them with the OS.
 */
pal_rc_t pal_init(void) {

}

/******************************************************************************
 * THREAD
 *****************************************************************************/

/**
 * For Piksi Multi, we support a maximum of two auxiliary threads.
 */
static_assert(STARLING_MAX_NUM_THREADS <= 2);


pal_rc_t pal_thread_run(unsigned int priority,
                        void (*fn)(void *), 
                        void *context) {

}

/******************************************************************************
 * QUEUE 
 *****************************************************************************/

void *pal_queue_create(const size_t capacity) {
  return NULL;
}

pal_rc_t pal_queue_push_back(void *q, const void *p) {
  return -1;
}

pal_rc_t pal_queue_pop_front(void *q, void **p) {
  return -1;
}

void pal_queue_destroy(void *q) {

}

/******************************************************************************
 * ALLOCATOR
 *****************************************************************************/

// STARTUP_ONLY
pal_rc_t pal_allocator_register(const size_t block_size, const size_t n_blocks) {
  return -1;
}

pal_rc_t pal_allocator_alloc(const size_t block_size, void **p) {
  return -1;
}

pal_rc_t pal_allocator_free(void *p) {
  return -1;
}


