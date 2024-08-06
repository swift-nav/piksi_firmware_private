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

#ifndef LIBPAL_IMPL_IMPL_H
#define LIBPAL_IMPL_IMPL_H

#include <libpal/error.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize an implementation
 *
 * A PAL implementation must define this function. It is an opportunity for an
 * implementation to initialize the platform in any way required before any
 * other piece of code tries to make use of the features it provides.
 *
 * This function must install itself in to the PAL API by calling the relevant
 * "set impl" function for each module it implements. Without doing this
 * functional code will not be able to access the PAL implementation.
 *
 * This function will be called at some time during startup. It is guaranteed
 * that this function will be called before any other call in to the
 * implementation.
 *
 * @return PAL error code
 */
enum pal_error pal_impl_init(void);

/**
 * Deinitialize an implementation
 *
 * A PAL implementation must define this function. It will be called at the end
 * of a process's execution at a point when PAL will no longer be needed. In
 * this function an implementation should release any resources it had allocated
 * and return the platform to the state it was in before pal_impl_init was
 * called.
 *
 * An implementation does not need to uninstall its module implementations, this
 * will be handled automatically by libpal proper.
 *
 * @return PAL error code
 */
enum pal_error pal_impl_deinit(void);

/**
 * Signal initialization complete
 *
 * A product based on libpal will call this function once and only once after it
 * has allocated all resources required for normal running. A PAL implementation
 * may take the call to this function as a signal that there will be no further
 * resource allocations made by the product.
 *
 * Should the underlying platform expose some sort of protected or safe mode
 * then the call to this function should be the signal to enter it.
 *
 * After this function has been called there should be no further calls to
 * allocating PAL functions with the following exceptions
 * - pal_tcp_client_open
 * - pal_tcp_accept
 *
 * Both of the above functions return a new TCP socket handle and must be
 * allowed to succeed even after this function has been called.
 *
 * All PAL handles which were allocated before the call to this function must
 * remain valid after the call, and all valid operations on those handles must
 * proceed as normal. PAL handles will be released back to the implementation
 * before a call to #pal_impl_deinit in the future.
 *
 * @return PAL error code
 */
enum pal_error pal_impl_init_complete(void);

#ifdef __cplusplus
}
#endif
#endif
