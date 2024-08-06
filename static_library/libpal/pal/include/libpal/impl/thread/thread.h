/*
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBPAL_IMPL_THREAD_THREAD_H
#define LIBPAL_IMPL_THREAD_THREAD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <libpal/error.h>
#include <libpal/impl/thread/thread.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Thread etry point signature
 */
typedef void (*pal_thread_entry_t)(void *);

/**
 * PAL Thread handle
 */
typedef void *pal_thread_t;

/**
 * Default stack size
 *
 * When passed to #pal_thread_create_t the platform implementation is free to
 * choose an appropriate default stack size for the new thread
 */

#define PAL_THREAD_DEFAULT_STACKSIZE 0

/**
 * Start a new thread
 *
 * Create a new thread of execution with the given entry point and parameters.
 *
 * The platform implementation must create a new thread with its own stack and
 * start execution at the function given in the \p entry parameter. The \p ctx
 * parameter must be passed directly in to the entry function.
 *
 * The \p stacksize parameter specifies the minimum stack size required for the
 * new thread. The special value of 0 (PAL_THREAD_DEFAULT_STACKSIZE) allows the
 * platform implementation to use an appropriate default stack size.
 *
 * On success this function must create a new thread handle and return it in the
 * \p thread parameter. This handle will be used in later called to
 * pal_thread_interrupt (if interrupt support is provided)  and pal_thread_join
 *
 * @param thread On success must be updated with a handle representing the newly
 * created thread
 * @param name Thread name, may be null for platform default or no name
 * @param entry Entry point for the new thread
 * @param ctx Context parameter to pass directly in to the entry function
 * @param stacksize Requested stack size, 0 for platform default
 * @return PAL error code
 */
typedef enum pal_error (*pal_thread_create_t)(pal_thread_t *thread,
                                              const char *name,
                                              pal_thread_entry_t entry,
                                              void *ctx, size_t stacksize);

/**
 * Wait for a thread to exit
 *
 * This function will be called with the handle to a thread previously created
 * by a call to pal_thread_create. It must block the caller until such a time as
 * the specified thread has exitied its entry function.
 *
 * @param thread Thread handle
 * @return PAL error code
 */
typedef enum pal_error (*pal_thread_join_t)(pal_thread_t thread);

/**
 * Interrupts the specified thread
 *
 * IO function have a "mode" parameter to control the blocking behaviour.
 * Whenever an IO function is called with the PAL_BLOCKING mode parameter it may
 * block for a specified period or indefinitely. Interrupts are a way for the
 * user of libpal to cause the immediate early return of one of these blocking
 * functions.
 *
 * Interrupts are an optional feature, a PAL implementation is not required to
 * provide an implementation for this function.
 *
 * If the PAL implementation provides interrupt support it must allow for any
 * blocking IO function to respond. When a thread is blocked on an IO function a
 * different thread may call this function with the handle of the blocked
 * thread. The implementation must cause the blocked thread to wake up and
 * immediately return PAL_INTERRUPT without completing the task it is meant to
 * be doing.
 *
 * This requirement applies to the following PAL functions
 * - pal_io_read
 * - pal_io_write
 * - pal_udp_send_to
 * - pal_udp_receive_from
 * - pal_tcp_client_open
 * - pal_tcp_accept
 *
 * All other blocking functions are not required to respond to interrupts
 *
 * @param thread Thread to interrupt, handle returned from pal_thread_create
 * @return PAL error code
 */
typedef enum pal_error (*pal_thread_interrupt_t)(pal_thread_t thread);

/**
 * PAL thread implementation definition
 */
struct pal_impl_thread {
  /// Implementation start new thread routine
  pal_thread_create_t create;
  /// Implementation join thread routine
  pal_thread_join_t join;
  // Implementation for thread interruption
  pal_thread_interrupt_t interrupt;
};

/**
 * Install PAL threading implementation in the API
 *
 * Call this function during pal_impl_init to register the implementation's
 * threading module with the libpal API
 *
 * @param impl Threading implementation definition
 * @return PAL error code
 */
enum pal_error pal_set_impl_thread(struct pal_impl_thread *impl);

#ifdef __cplusplus
}
#endif

#endif
