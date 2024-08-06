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

#include <libpal/error.h>

/**
 * Format PAL error code as human readable string
 *
 * Returns a human readable string representing a PAL return code.
 *
 * @param err PAL return/error code returned from other PAL functions
 * @return Human readable string
 */
const char *pal_strerror(enum pal_error err) {
  switch (err) {
    case PAL_SUCCESS:
      return "Success";
    case PAL_INVALID:
      return "Invalid";
    case PAL_ERROR:
      return "Unspecified error";
    case PAL_TIMEOUT:
      return "Timeout";
    case PAL_IO_ERROR:
      return "IO error";
    case PAL_OOM:
      return "Out of memory";
    case PAL_RESOLVE_FAIL:
      return "Failure in name resolution";
    case PAL_CONNECT_FAIL:
      return "Failed to establish connection";
    case PAL_BIND_FAIL:
      return "Failed to bind to port";
    case PAL_LISTEN_FAIL:
      return "Failed to listen on port";
    case PAL_WOULD_BLOCK:
      return "Non-blocking call would have blocked";
    case PAL_INTERRUPT:
      return "Blocking call has been interrupted";
    default:
      return "Unknown error";
  }
}
