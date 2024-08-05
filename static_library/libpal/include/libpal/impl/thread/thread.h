/**
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

#ifndef LIBPAL_IMPL_THREAD_THREAD_H
#define LIBPAL_IMPL_THREAD_THREAD_H

#include <libpal/thread/thread.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * PAL Thread Implementation
 *
 * This file defines the interface a PAL implementation must use to installed
 * its own threading ability in to the libpal API
 *
 * The function pointer names and signatures in this file match those in
 * libpal/thread/thread.h. The PAL implementation must provide a version of
 * these functions which meet the requirements stated in the documentation
 * contained in that file.
 */

/**
 * Start a new thread
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_thread_create() (see libpal/thread/thread.h)
 *
 */
typedef enum pal_error (*pal_thread_create_t)(pal_thread_t *thread,
                                              pal_thread_entry_t entry,
                                              void *ctx, size_t stacksize,
                                              uint8_t prio);
/**
 * Set the current thread's name
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_thread_set_name() (see libpal/thread/thread.h)
 *
 */
typedef enum pal_error (*pal_thread_set_name_t)(const char *name);

/**
 * Wait for a thread to exit
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_thread_join() (see libpal/thread/thread.h)
 *
 */
typedef enum pal_error (*pal_thread_join_t)(pal_thread_t thread, void **retval);

/**
 * Terminate the current thread
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_thread_exit() (see libpal/thread/thread.h)
 *
 */
typedef void (*pal_thread_exit_t)(void *code);

/**
 * Interrupts the specified thread
 *
 * A PAL implementation must define this function according to the requirements
 * of #pal_thread_interrupt().
 */
typedef enum pal_error (*pal_thread_interrupt_t)(pal_thread_t thread);

/**
 * PAL thread implementation definition
 */
struct pal_impl_thread {
  /// Implementation start new thread routine
  pal_thread_create_t create;
  /// Implementation set current thread name routine
  pal_thread_set_name_t set_name;
  /// Implementation join thread routine
  pal_thread_join_t join;
  /// Implementation terminate current thread routine
  pal_thread_exit_t exit;
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
 */
void pal_set_impl_thread(struct pal_impl_thread *impl);

#ifdef __cplusplus
}
#endif

#endif
