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
#include <chthreads.h>
#include <stdint.h>
#include <limits.h>
#include <assert.h>

/**
 * This implementation is a little bit complicated. Because ChibiOS
 * places some OS data structures in the working stack area,
 * we have to be very careful about re-purposing stack areas.
 * 
 * Here is an overview of the algorithm used:
 *
 * Over the lifetime of the program, each thread is assigned a unique ID.
 * These are just monotonically increasing unsigned integers. This places
 * an artificial limit on the number of threads that can be created over 
 * the course of the programs execution.
 *
 * There are a limited number of thread working areas (memory regions), so
 * this in turn limits the number of concurrent threads we can support. When
 * a thread is done executing, we want to be able to reclaim the memory region
 * for reassignment. However, a thread technically isn't dead until all other
 * threads wishing to "join" with it have had the opportunity to do so.
 *
 * To solve this problem, we make use of the monotically increasing unique 
 * thread IDs. Any attempt to join with a thread ID that is currently active
 * will block until it completes. Attempts to join older thread IDs (with a
 * lower ID than any current threads) will immediately succeed. An attempt
 * to join a future thread is an error.
 *
 * Whenever an we try to create a new thread, we will first sweep all possible
 * thread working areas and reclaim any dead threads.
 */

#define MAX_NUM_THREADS 2

const unsigned int STARLING_PLATFORM_MAX_NUM_THREADS = MAX_NUM_THREADS;

#define THREAD_STACK_SIZE (6 * 1024 * 1024)

// Lock used to synchronize the implementation internals. 
static MUTEX_DECL(internal_lock);

// Internal working areas for Chibi static threads.
THD_WORKING_AREA(wa_thread_1, THREAD_STACK_SIZE);
THD_WORKING_AREA(wa_thread_2, THREAD_STACK_SIZE);

// Some handy constants used for keeping track of thread IDs.
#define THREAD_ID_UNINITIALIZED 0
#define THREAD_ID_INITIAL       1
#define THREAD_ID_MAX           UINT_MAX

// Monotonically increasing count of threads allocated over the course of the runtime.
static unsigned int next_thread_id = THREAD_ID_INITIAL;

/**
 * Used for internal bookkeeping. Note that the variables "in_use" and
 * "thread_id" are always used together in a transactional manner, and
 * must thus be synchronized correctly with the implementation internal lock.
 */
typedef struct starling_thread_t {
  void *const working_area;  // Immutable working area pointer.
  thread_t *chibi_tp;        // Pointer to the underlying Chibi thread struct.
  bool in_use;               // Indicates if this working area is in use or not.
  unsigned int thread_id;    // Monotonically increasing unique thread ID.
} starling_thread_t;

// Initialize working area info blocks. Note that the working area pointers are immutable.
static starling_thread_t starling_thread_info[MAX_NUM_THREADS] = {
  {wa_thread_1, NULL, false, THREAD_ID_UNINITIALIZED},
  {wa_thread_2, NULL, false, THREAD_ID_UNINITIALIZED},
};

/**
 * Look over all of the working areas and reclaim the ones which we believe
 * to be unavailable, but are actually marked as dead by the OS. Must be
 * very careful not to write race conditions or deadlock. Because this memory
 * is technically owned by the OS, must use the system lock as well.
 *
 * This is the only place in this code with nested locks. Furthermore the system
 * lock has narrower scope so there should be no deadlock within this module, and
 * the system lock is guaranteed to release.
 */
static void reclaim_dead_working_areas(void) {
  chMtxLock(&internal_lock);
  chSysLock();
  for (unsigned int i = 0; i < MAX_NUM_THREADS; ++i) {
    starling_thread_t *thd = &starling_thread_info[i];
    // Working areas that are marked in_use are candidates for being reclaimed.
    if (thd->in_use && thd->chibi_tp->p_state == CH_STATE_FINAL) {
      thd->in_use = false;
      thd->thread_id = THREAD_ID_UNINITIALIZED;
    }
  }
  chSysUnlock();
  chMtxUnlock(&internal_lock);
}

/**
 * Traverse the working areas looking for an available one. Must be
 * careful to synchronize against other accesses of the is_available
 * field.
 */
static starling_thread_t *find_available_working_area(void) {
  starling_thread_t *thd = NULL;
  chMtxLock(&internal_lock);
  for (unsigned int i = 0; i < MAX_NUM_THREADS; ++i) {
    starling_thread_t *curr = &starling_thread_info[i];
    if (!curr->in_use) {
      thd = curr;
      thd->in_use = true;
      thd->thread_id = next_thread_id++;
      break;
    }
  }
  chMtxUnlock(&internal_lock);
  return thd;
}

/**
 * Compare the given thread ID to the thread IDs currently active in
 * the working areas. Note that an ID being active is not the same as
 * the thread being "alive". This just means that the given thread_id
 * is currently registered to one of the working areas. This is important
 * because it means that the ChibiOS Thread struct which lives in the working
 * area is valid and corresponds to the requested ID.
 */
bool is_thread_id_active(unsigned int thread_id) {
  bool match_found = false;
  chMtxLock(&internal_lock);
  for (unsigned int i = 0; i < MAX_NUM_THREADS; ++i) {
    if (starling_thread_info[i].thread_id == thread_id) {
      match_found = true;
      break;
    }
  }
  chMtxUnlock(&internal_lock);
  return match_found;
}

/**
 * Pick from the available working areas to initialize a thread. 
 */
starling_thread_t* starling_thread_create(unsigned int priority,
                                          void(*fn)(void*),
                                          void* user) 
{
  if (next_thread_id == THREAD_ID_MAX) {
    return NULL;
  }
  reclaim_dead_working_areas();
  starling_thread_t *thd = find_available_working_area();
  if (thd) {
    thd->chibi_tp = chThdCreateStatic(thd->working_area, 
                                     THREAD_STACK_SIZE,
                                     priority, fn, user); 
  }
  return thd;
}

/**
 * Let ChibiOS block until the thread is complete. Attempting to
 * join a dead or non-existant thread returns immediately.
 */
void starling_thread_join(starling_thread_t* thd) {
  assert(thd);
  // TODO(kevin) This is techinically an incorrect implementation.
  // A thread could become non-current between checking and waiting on it.
  // This would mean the call to chThdWait has a potentially invalid pointer.
  if (is_thread_id_active(thd->thread_id)) {
    chThdWait(thd->chibi_tp);
  }
}

/**
 * Tell ChibiOS to signal the target thread should terminate. Attempting
 * to terminate a dead or non-existant thread returns immediately. */
void starling_thread_terminate(starling_thread_t* thd) {
  assert(thd);
  // TODO(kevin) Incorrect for same reason as above.
  if (is_thread_id_active(thd->thread_id)) {
    chThdTerminate(thd->chibi_tp);
  } 
}

/**
 * Use the ChibiOS yield functionality to yield the current thread.
 */
void starling_thread_yield(void) {
  chThdYield();
}




