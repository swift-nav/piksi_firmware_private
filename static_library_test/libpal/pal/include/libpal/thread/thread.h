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

#ifndef LIBPAL_THREAD_THREAD_H
#define LIBPAL_THREAD_THREAD_H

#include <libpal/impl/thread/thread.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Check global PAL threading implementation
 * @return True if global PAL threading implementation has been set, otherwise
 * False.
 */
bool pal_has_impl_thread(void);

/**
 * Check platform support for thread interrupts
 *
 * A user of libpal can use this function to verify that the underlying platform
 * implementation supports thread interrupt. If this function returns false it
 * is not safe to call #pal_thread_interrupt and the user of libpal must
 * consider using timeouts on blocking IO operations
 *
 * @return true if the PAL implementation supports thread interrupts
 */
bool pal_has_impl_thread_interrupt(void);

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
 * @param thread On success will be set to a handle representing the new thread
 * which can be passed to pal_thread_join
 * @param name Thread name, may be null
 * @param entry Thread entry point. Must be of type void *(*fn)(void*)
 * @param ctx Parameter to be passed to the entry point
 * @param stacksize Stack size in bytes
 * @return PAL error code
 */
enum pal_error pal_thread_create(pal_thread_t *thread, const char *name,
                                 pal_thread_entry_t entry, void *ctx,
                                 size_t stacksize);

/**
 * Wait for a thread to terminate
 *
 * This function will block the caller until the specified thread exits, that is
 * until the thread returns from its entry point.
 *
 * @param thread PAL thread handle returned from pal_thread_create
 * @return PAL error code
 */
enum pal_error pal_thread_join(pal_thread_t thread);

/**
 * Signals the thread to stop whatever blocking IO operation it currently is on
 * and or potentially any that will come across in the future and immediately
 * return with a platform error of #PAL_INTERRUPT. If the user calls this
 * function while while the thread is attending to a non blocking function, the
 * next blocking function will receive this signal and respond to as mentioned
 * previously. Subsequent blocking functions will not be interrupted until a new
 * signal is emitted via this function.
 *
 * Only certain IO functions will respond to an interrupt. The complete list of
 * functions is:
 * - pal_io_read
 * - pal_io_write
 * - pal_tcp_accept
 * - pal_tcp_client_open
 * - pal_udp_send_to
 * - pal_udp_receive_from
 *
 * All other functions in the libpal will not respond to thread interrupts even
 * if they block.
 *
 * Thread interrupts are an optional feature and the user of libpal must not
 * assume that the underlying platform implementation has provided support for
 * them. Before calling this function the user must call
 * #pal_has_impl_thread_interrupt to verify platform support
 *
 * @param thread thread to interrupt
 * @return PAL error code
 */
enum pal_error pal_thread_interrupt(pal_thread_t thread);

#ifdef __cplusplus
}
#endif

#endif
