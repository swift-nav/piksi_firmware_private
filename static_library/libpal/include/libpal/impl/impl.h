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

#ifndef LIBPAL_IMPL_IMPL_H
#define LIBPAL_IMPL_IMPL_H

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
 */
void pal_impl_init(void);

/**
 * Deinitialize an implementation
 *
 * A PAL implementation must define this function. It will be called at the end
 * of a process's execution at a point when PAL will no longer be needed. In
 * this function an implementation should release any resources it had allocated
 * and return the platform to the state it was in before pal_init_impl was
 * called.
 *
 * An implementation does not need to uninstall its module implementations, this
 * will be handled automatically by libpal proper.
 */
void pal_impl_deinit(void);

#ifdef __cplusplus
}
#endif
#endif
