/*
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

#ifndef LIBPAL_THREAD_THREAD_H
#define LIBPAL_THREAD_THREAD_H

#include <libpal/error.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Use the default stack size as defined by the platform
#define PAL_THREAD_DEFAULT_STACKSIZE 0
// Use default thread priority as defined by the platform
#define PAL_THREAD_DEFAULT_PRIO 0

// PAL Thread Context
typedef void *(*pal_thread_entry_t)(void *);
typedef void *pal_thread_t;

/**
 * Thread priority wrangling.
 *
 * Use pal_thread_mk_prio to create a value suitable to be passed to
 * pal_thread_create. high is a boolean value, specify true to indicate this
 * thread should run as high priority (non-preemptible), false for normal
 * priority. The prio argument accepts a value between 1-99 to indicate
 * relative thread priority with 1 being the lowest and 99 highest.
 */
#define PAL_THREAD_MAX_PRIO 99

/**
 * Construct a thread priority value
 * @param high True for realtime threads, false otherwise
 * @param prio Relative thread priority, 1-99, 99 is highest
 * @return Thread priority suitable to be passed to pal_thread_create()
 */
static inline uint8_t pal_thread_mk_prio(bool high, uint8_t prio) {
  if (prio > PAL_THREAD_MAX_PRIO) prio = PAL_THREAD_MAX_PRIO;
  return (uint8_t)((high ? 0x80 : 0) | prio);
}

/**
 * Test whether value constructed by pal_thread_mk_prio() describes a real time
 * thread
 * @param prio Value constructed by pal_thread_mk_prio()
 * @return True for high priority (realtime) thread, false otherwise
 */
static inline bool pal_thread_is_high_prio(uint8_t prio) {
  return (prio & 0x80) != 0;
}

/**
 * Extract thread priority value
 * @param prio Value constructed by pal_thread_mk_prio()
 * @return Thread priority
 */
static inline uint8_t pal_thread_get_prio(uint8_t prio) { return prio & 0x7f; }

/**
 * Check global PAL threading implementation
 * @return True if global PAL threading implementation has been set, otherwise
 * False.
 */
bool pal_has_impl_thread(void);

/**
 * Start a new thread
 *
 * Create a thread with the given entry point and parameters.
 *
 * The entry point can be any address in executable space which accepts a single
 * void* pointer. Once started execution will continue from that point.
 *
 * The stack size for the newly created thread can be specified in bytes. There
 * may be platform restrictions on stack sizes. If a custom stack size is not
 * important use the constant PAL_THREAD_DEFAULT_STACKSIZE.
 *
 * The priority of the new thread can be specified, the PAL implementation will
 * translate it in to a value appropriate for the platform. Construct a priority
 * value using pal_thread_mk_prio(). If a custom priority is not required use
 * the constant PAL_THREAD_DEFAULT_PRIO.
 *
 * @param thread On success will be set to a handle representing the new thread
 * which can be passed to pal_thread_join
 * @param entry Thread entry point. Must be of type void *(*fn)(void*)
 * @param ctx Parameter to be passed to the entry point
 * @param stacksize Stack size in bytes
 * @param prio PAL thread priority level
 * @return PAL error code
 */
enum pal_error pal_thread_create(pal_thread_t *thread, pal_thread_entry_t entry,
                                 void *ctx, size_t stacksize, uint8_t prio);

/**
 * Set a human readable name for the current thread.
 *
 * Support for this function may be limited on certain platforms, how it is
 * visible to other parts of the system is very platform specific.
 *
 * @param name New name for current thread
 * @return PAL error code
 */
enum pal_error pal_thread_set_name(const char *name);

/**
 * Wait for a thread to terminate
 *
 * This function will block the caller until the specified thread exits, that is
 * until the thread returns from its entry point or calls pal_thread_exit. The
 * optional parameter retval can be used to obtain the return code of the
 * thread's entry point function, or the parameter it passed to pal_thread_exit.
 *
 * @param thread PAL thread handle returned from pal_thread_create
 * @param retval (optional) On exit will be set to the threads exit code
 * @return PAL error code
 */
enum pal_error pal_thread_join(pal_thread_t thread, void **retval);

/**
 * Immediately terminate the current thread
 *
 * This is equivelant to returning from the thread's entry point function. The
 * current thread will be immediately terminated. The parameter given to this
 * function will be returned to any other thread that has joined this one.
 *
 * This function does not return
 *
 * @param code Thread exit code
 */
void pal_thread_exit(void *code);

/**
 * Signals the thread to stop whatever blocking operation it currently is on and
 * or potentially any that will come across in the future and immediately return
 * with a platform error of #PAL_INTERRUPT. If the user calls this function
 * while while the thread is attending to a non blocking function, the next
 * blocking function will receive this signal and respond to as mentioned
 * previously. Subsequent blocking functions will not be interrupted until a new
 * signal is emitted via this function.
 *
 * It is worth mentioning that all blocking operations within the platform are
 * interruptable except for the following:
 *
 *   - mutex lock
 *   - condition variables
 *
 * The above synchronization operation cannot be interrupted, otherwise they
 * would fail to provide their respective purpose of guaranteeing correct
 * execution within critical regions.
 *
 * @param thread thread to interrupt
 * @return PAL error code
 */
enum pal_error pal_thread_interrupt(pal_thread_t thread);

#ifdef __cplusplus
}
#endif

#endif
