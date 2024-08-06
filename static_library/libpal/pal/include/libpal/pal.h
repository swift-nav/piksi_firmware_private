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

#ifndef LIBPAL_PAL_H
#define LIBPAL_PAL_H

#include <libpal/error.h>

#include <libpal/chrono/monotonic_clock.h>
#include <libpal/identifier/identifier.h>
#include <libpal/io/file.h>
#include <libpal/io/serial.h>
#include <libpal/io/stdstream.h>
#include <libpal/io/tcp.h>
#include <libpal/io/udp.h>
#include <libpal/ipc/mq.h>
#include <libpal/mem/mem.h>
#include <libpal/synch/condition_var.h>
#include <libpal/synch/mutex.h>
#include <libpal/thread/thread.h>
#include <libpal/watchdog/watchdog.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize PAL Implementation
 *
 * This function will explicitly initialize the underlying PAL implementation.
 * PAL must be initialized by a call to pal_init() before calling any other PAL
 * function.
 *
 * If any of the "has impl" functions such as pal_has_impl_mutex are called
 * before this function they will return false. Only after PAL has been
 * initialized can it accurately be known if an implementation actually has some
 * specific feature.
 *
 * Calling this function multiple times will have no effect after the first
 * call. Once a PAL implementation has been initialized it remains so until
 * the process terminates or a call to pal_deinit().
 *
 * If this function returns anything other than PAL_SUCCESS then initialization
 * failed and it is not safe to call any other PAL function.
 *
 * @return PAL error code
 */
enum pal_error pal_init(void);

/**
 * Deinitialize PAL Implementation
 *
 * This function will deinitialize the underlying PAL implementation.
 *
 * After this function returns it will be as though PAL was never initialized,
 * as though pal_init was never called. Calling any of the "has impl" functions
 * after this will again return false. Every PAL API call will be reset to the
 * default implementation.
 *
 * After this function returns any PAL handles which had not been properly freed
 * will become invalid. Trying to use them will have undefined effects.
 *
 * PAL can be reinitialized by calling pal_init again and used in the normal way
 * afterwards.
 *
 * Trying to reinitialize PAL is potentially very dangerous, especially when
 * dealing with PAL resources allocated in the constructors of static global c++
 * objects. In most production code there is no reason to call this function at
 * any point other than immediately before returning from main(). It is mostly
 * useful for testing purposes.
 *
 * The force argument is provided to circumvent the init/deinit counters that
 * provide a mechanism for multiple libraries to call pal_init and pal_deinit
 * without interfering with each other. This should not be used save for a
 * final integration when it is known for certain that no usage of PAL handles
 * or APIs will be performed, typically in a main method or similar top level
 * scope.
 *
 * @return PAL error code
 */
enum pal_error pal_deinit(bool force);

/**
 * Signal initialization complete
 *
 * A product which uses libpal to abstract platform facilities can use this
 * function to signal the underlying implementation that initialization is
 * complete. Before calling this function the product must have already
 * initialized libpal with a call to #pal_init() and must have finished
 * allocating all resources required for normal running.
 *
 * A platform implementation may use this function to enter a protected mode
 * where further resource allocation is no longer possible. Therefore the
 * product must make sure that it has already taken all required platform
 * resources before calling this function.
 *
 * After this function is called it must be assumed that any further calls to
 * libpal functions which allocate resources will no longer succeed, with the
 * following exceptions:
 * - pal_tcp_client_open
 * - pal_tcp_accept
 *
 * All handles which were allocated before calling this function will remain
 * valid and the full set of valid operations may be performed on them.
 *
 * @return PAL error code
 */
enum pal_error pal_init_complete(void);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // LIBPAL_PAL_H
