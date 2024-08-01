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

#ifndef LIBPAL_IMPL_IO_STDSTREAM_H
#define LIBPAL_IMPL_IO_STDSTREAM_H

#include <libpal/io/io.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * PAL standard stream IO implementation
 *
 * This file defines the interface a PAL implementation must use to install its
 * own standard stream handling ability in to the libpal API
 *
 * The function pointer names and signatures in this file match those in
 * libpal/io/stdstream.h. The PAL implementation must provide a version of these
 * functions which meet the requirements stated in the documentation contained
 * in that file.
 */

/**
 * Get a handle to a standard stream device
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_get_stdout()/pal_get_stderr()/pal_get_stdin() (see
 * libpal/io/stdstream.h)
 *
 */
typedef enum pal_error (*pal_stdstream_get_io_t)(pal_io_t *io);

/**
 * PAL Standard stream IO implementation definition
 */
struct pal_impl_stdstream {
  /// Implementation get standard input stream routine
  pal_stdstream_get_io_t get_stdin;
  /// Implementation get standard output stream routine
  pal_stdstream_get_io_t get_stdout;
  /// Implementation get standard error stream routine
  pal_stdstream_get_io_t get_stderr;
};

/**
 * Install PAL standard stream IO implementation in to API
 *
 * Call this function during pal_impl_init to register the implementation's
 * standard stream IO module with the libpal API
 *
 * @param impl Standard stream IO implementation definition
 */
void pal_set_impl_stdstream(struct pal_impl_stdstream *impl);

#ifdef __cplusplus
}
#endif

#endif
