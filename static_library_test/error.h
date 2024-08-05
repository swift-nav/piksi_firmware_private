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

#ifndef LIBPAL_ERROR_H
#define LIBPAL_ERROR_H

enum pal_error {
  /**
   * The operation completed succefully
   */
  PAL_SUCCESS,

  /**
   * Invalid arguments to function
   */
  PAL_INVALID,

  /**
   * An unspecified error occured during the function
   */
  PAL_ERROR,

  /**
   * The operation timeout out
   */
  PAL_TIMEOUT,

  /**
   * A function called in non-blocking mode would have blocked program execution
   * so did not do anything
   */
  PAL_WOULD_BLOCK,

  /**
   * An error occurred while interfacing with the IO device
   */
  PAL_IO_ERROR,

  /**
   * libpal has run out of resources, memory allocation error
   */
  PAL_OOM,

  /**
   * Failed to resolve hostname
   */
  PAL_RESOLVE_FAIL,

  /**
   * Failed to establish connection
   */
  PAL_CONNECT_FAIL,

  /**
   * Failed to bind to port
   */
  PAL_BIND_FAIL,

  /**
   * Failed to listen on port
   */
  PAL_LISTEN_FAIL,

  /**
   * Operation interrupted
   */
  PAL_INTERRUPT,
};

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Format a PAL error code as a human readable string
 *
 * @param err Error code returned from some other libpal function
 * @return Human readable error string
 */
const char *pal_strerror(enum pal_error err);

#ifdef __cplusplus
}
#endif

#endif  // LIBPAL_ERROR_H
