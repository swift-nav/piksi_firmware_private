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

#include <starling/platform/thread.h>
#include <ch.h>
#include <stdint.h>
#include <assert.h>

#define MAX_NUM_THREADS 2

const unsigned int STARLING_PLATFORM_MAX_NUM_THREADS = MAX_NUM_THREADS;

#define THREAD_STACK_SIZE (6 * 1024 * 1024)

// Lock used to synchronize the implementation internals. 
MUTEX_DECL(internal_lock);

// Internal working areas for Chibi static threads.
THD_WORKING_AREA(wa_thread_1, THREAD_STACK_SIZE);
THD_WORKING_AREA(wa_thread_2, THREAD_STACK_SIZE);

// Used for internal thread bookkeeping.
typedef struct thread_info_t {
  void * const working_area; // Immutable working area pointer.
  bool in_use;
  void(*fn)(void*);
  void *user;
};

// Initialize thread info blocks. Note that the working area pointers are immutable.
static thread_info_t thread_info_array[MAX_NUM_THREADS] = {
  {wa_thread_1, false, NULL, NULL},
  {wa_thread_2, false, NULL, NULL},
};

/**
 * We need to wrap the user function and data in order to perform
 * the internal bookkeeping. The argument is assumed to point to
 * one of the static thread info elements.
 */
static void wrap_thread_fn(void* arg) {
  const thread_info_t *thread_info = (thread_info_t*)arg;
  // If this function is running, the thread must be "in_use".
  assert(thread_info->in_use);
  // Let the user function run.
  if (thread_info->fn) {
    thread_info->fn(thread_info->user);
  }
  // On completion, "free" the working area. Set the pointers to NULL for debuggin.
  chMtxLock(&internal_lock);
  thread_info->in_use = false;
  thread_info->fn = NULL;
  thread_info->user = NULL;
  chMtxUnlock(&internal_lock);
  // TODO(kevin) THIS COULD BE A MAJOR BUG IF CHIBIOS MAKES USE OF THE 
  // WORKING AREA FOR ITS OWN OS-SPECIFIC DATA STRUCTURES. THE CURRENT ASSUMPTION
  // IS THAT IT ALLOCATES THREAD STATE IN AN OS-PROTECTED AREA SO THE ONLY
  // CODE THAT EVER TOUCHES THE WORKING AREA IS THE USER FUNCTION.
}

/**
 * Pick from the available working areas to initialize a thread. 
 */
starling_thread_t* starling_thread_create(unsigned int priority,
                                          void(*fn)(void*),
                                          void* user) 
  thread_info_t *thread_info = NULL;
  // Look for a thread that isn't in use.
  chMtxLock(&internal_lock);
  for (unsigned int i = 0; i < MAX_NUM_THREADS; ++i) {
    if (!thread_info_array[i].in_use) {
      thread_info = &thread_info_array[i]; 
      thread_info->in_use = true;
      break;
    }
  }
  chMtxUnlock(&internal_lock);
  // Once we have safely claimed a thread (or not), we are free
  // to intialize it.
  if (thread_info) {
    thread_info->fn = fn;
    thread_info->user = user;
    return chThdCreateStatic(area, THREAD_STACK_SIZE, priority, 
                             &wrap_thread_fn, thread_info); 
  } else {
    return NULL;
  }
}

/**
 * Let ChibiOS block until the thread is complete.
 */
void starling_thread_join(starling_thread_t* thd) {
  (void)chThdWait((Thread*)thd);
}

/**
 * Tell ChibiOS to signal the target thread should terminate.
 */
void starling_thread_terminate(starling_thread_t* thd) {
  chThdTerminate((Thread*)thd);
}

/**
 * Use the ChibiOS yield functionality.
 */
void starling_thread_yield(void) {
  chThdYield();
}




