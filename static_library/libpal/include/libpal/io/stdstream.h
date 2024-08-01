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

#ifndef LIBPAL_IO_STDSTREAM_H
#define LIBPAL_IO_STDSTREAM_H

#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>

#include <libpal/io/io.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Check global PAL standard stream implementation
 * @return true if global PAL standard stream implementation has been set,
 * otherwise false
 */
bool pal_has_impl_stdstream(void);

/**
 * Get the IO handle for the standard input stream
 *
 * The handle returned from this function can be passed to pal_io_read to get
 * data from stdin. It can also be passed to pal_io_watch, but will not be
 * available for writing and may not be closed.
 *
 * @param io On success will be set to a handle representing the standard input
 * stream
 * @return PAL error code
 */
enum pal_error pal_get_stdin(pal_io_t *io);

/**
 * Get the IO handle for the standard output stream
 *
 * The handle returned from this function can be passed to pal_io_write to write
 * data to stdout. It may not be used for reading and may not be closed.
 *
 * @param io On success will be set to a handle representing the standard output
 * stream
 * @return PAL error code
 */
enum pal_error pal_get_stdout(pal_io_t *io);

/**
 * Get the IO handle for the standard error stream
 *
 * The handle returned from this function can be passed to pal_io_write to write
 * data to stderr. It may not be used for reading and may not be closed.
 *
 * @param io On success will be set to a handle representing the standard error
 * output stream
 * @return PAL error code
 */
enum pal_error pal_get_stderr(pal_io_t *io);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // LIBPAL_IO_STDSTREAM_H
