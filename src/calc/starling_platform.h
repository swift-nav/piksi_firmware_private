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

#ifndef STARLING_CALC_STARLING_PLATFORM_H
#define STARLING_CALC_STARLING_PLATFORM_H

/**
 * Starling Platform Abstraction Layer
 * ===================================
 * The Platform Abstraction Layer (PAL) serves to provide the Starling Engine
 * with everything it needs to run on a given software platform. Apart from
 * these platform specific "primitives", the Starling Engine is designed to be
 * completely portable.
 *
 * The Starling Engine manages a handful of threads over which it spreads the
 * GNSS estimation workload. Interface with and between these threads is handled
 * by message passing over mailboxes. In order to allow efficient
 * platform-specific implementation of these primitives, they are abstracted in
 * this layer.
 *
 * PAL implementers should make the following assumptions:
 *
 * 1. All platform primitives will be allocated only at application start.
 * 2. The worst-case requirements for each primitive are provided as constants.
 * 3. Message passing is by pointer. Alloc/dealloc is handled by the allocator.
 *
 *
 * NOTE:
 * Functions which are guaranteed to be called only once at startup are
 * marked with a STARTUP_ONLY comment.
 */

#include <stddef.h>
#include <stdint.h>

/******************************************************************************
 * GENERAL
 *****************************************************************************/

#define STARLING_PAL_OK 0x0

/**
 * Generic return type used by many functions in the API.
 * If a function returns successfully, it should return
 * STARLING_PAL_OK. When unsuccessful, functions are permitted
 * to return any other value with implementation-specific meaning.
 *
 * Functions which cannot possibly fail do not use this return type.
 */
typedef int pal_rc_t;

/**
 * In order to monitor the health of the Starling engine, the platform
 * should provide watchdog functionality. The Starling engine will kick the
 * watchdog every time it completes an operation. It is up to the platform
 * to monitor in a manner consistent with the way Starling is being used.
 */
void pal_watchdog_kick(void);

/**
 * Handle an implementation-specific error. The Starling Engine
 * may invoke this function on any return code which is not
 * STARLING_PAL_OK.
 */
void pal_handle_error(pal_rc_t code);

/*
 * Initialize the PAL. No primitive-related functions will be called
 * until after the PAL has been successfully initialized. On a
 * side-note, it is likely that much of the Starling API will not
 * work until it has had a chance to initialize its internal
 * data structures.
 *
 * Any return value besides STARLING_PAL_OK will be treated as a critical
 * system failure.

 * STARTUP_ONLY
 */
pal_rc_t pal_init(void);

/******************************************************************************
 * THREAD
 *****************************************************************************/

/* Maximum number of threads requested by the Starling engine. */
#define STARLING_MAX_NUM_THREADS 2
/* Maximum thread stack size requirement (in bytes) of Starling threads. */
#define STARLING_MAX_THREAD_STACK 0x600000

/**
 * The Starling engine requires a specific set of supported thread
 * priorities. It is up to the implementation to meet these needs in a
 * sensible way.
 */
enum {
  /* Tasks at this priority should be serviced as close to realtime as possible.
   */
  STARLING_PAL_PRIORITY_REALTIME,
  /* Tasks at this priority have no real timing constraints. */
  STARLING_PAL_PRIORITY_BACKGROUND,
};
typedef uint8_t pal_priority_t;

/**
 * Task info accepted by the platform thread launcher.
 */
typedef struct pal_thread_task_t {
  const char *name;
  pal_priority_t priority;
  void (*fn)(void *);
  void *context;
} pal_thread_task_t;

/**
 * Give the platform implementation a set of tasks to run individually
 * on separate threads. The implementation may adjust the actual thread
 * priority in the OS, as long as the relative prioritization among
 * Starling threads is preserved. Elements of the task array should either
 * point to a valid task struct, or be NULL.
 *
 * Any return value besides STARLING_PAL_OK will be treated as a critical
 * system failure.
 *
 * STARTUP_ONLY
 */
pal_rc_t pal_run_thread_tasks(
    pal_thread_task_t *const tasks[STARLING_MAX_NUM_THREADS]);

/******************************************************************************
 * MUTEX
 *****************************************************************************/

/**
 * Mutexes are indentified and accessed by index only because they are
 * finite in number.
 *
 * TODO(kevin) Ultimately we do not want mutexes in the platform layer
 * and they should be replaced with message passing over mailbox.
 */

// Maximum number of mutexes requested by Starling engine.
#define STARLING_MAX_NUM_MUTEXES 16

/**
 * Used to identify mutexes. Valid values are in the range
 * [0, STARLING_MAX_NUM_MUTEXES-1].
 */
typedef uint32_t mutex_id_t;

void pal_mutex_init(mutex_id_t id);
void pal_mutex_lock(mutex_id_t id);
void pal_mutex_unlock(mutex_id_t id);

/******************************************************************************
 * MAILBOX
 *****************************************************************************/

/**
 * Mailboxes are identified and accessed by index only. This decision
 * was made so that the "finite-ness" of the mailboxes is encoded in
 * the API. Obviously, it is an error to index a mailbox which doesn't exist.
 */

// Maximum number of mailboxes requested by the Starling engine.
#define STARLING_MAX_NUM_MAILBOXES 16
// Maximum number of entries that mailboxes must support.
#define STARLING_MAX_MAILBOX_CAPACITY 4

/**
 * Used to identify mailboxes. Valid values are in the range
 * [0, STARLING_MAX_NUM_MAILBOXES-1].
 */
typedef uint32_t mailbox_id_t;

/**
 * Initialize the id'th mailbox. This function will only be called
 * once per mailbox at startup.
 *
 * STARTUP_ONLY
 */
pal_rc_t pal_mailbox_init(mailbox_id_t id, const size_t capacity);

/**
 * Post a message to the mailbox. This may fail if the mailbox
 * is full.
 */
pal_rc_t pal_mailbox_post(mailbox_id_t id, const void *p);

/**
 * Try and perform a non-blocking fetch operation on the mailbox.
 * This function should return immediately. If the mailbox is empty
 * or some other error occurs, a platform-specific error code
 * is returned.
 */
pal_rc_t pal_mailbox_fetch_immediate(mailbox_id_t id, void **p);

/**
 * Try and perform a blocking fetch operation on the mailbox.
 * This function should block for some platform dependent amount
 * of time. By calling this function instead of the immediate variant,
 * the Starling library is indicating that a thread should pend on something
 * arriving in this mailbox. It is up to the platform implementation to
 * do something sensible (block, sleep, etc.). Return code is the same
 * as the immediate variant, although the platform may also return a
 * timeout error code.
 */
pal_rc_t pal_mailbox_fetch_timeout(mailbox_id_t id, void **p);

/******************************************************************************
 * ALLOCATOR
 *****************************************************************************/

// STARTUP_ONLY
pal_rc_t pal_allocator_register(const size_t block_size, const size_t n_blocks);

pal_rc_t pal_allocator_alloc(const size_t block_size, void **p);

pal_rc_t pal_allocator_free(void *p);

#endif
