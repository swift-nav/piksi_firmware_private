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

#ifndef LIBPAL_IMPL_IO_IO_H
#define LIBPAL_IMPL_IO_IO_H

#include <libpal/io/io.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * PAL IO Implementation
 *
 * This file defines the interface a PAL implementation must use to install its
 * own IO capabilities in to the libpal API
 *
 * The capabilities described in this file must be define by the implementation
 * if any of the supported IO device types (files, TCP, serial, etc). All IO
 * descriptors regardless of the underlying type use the same set of functions
 * from this file to read, write, close, etc.
 *
 * The function pointer names and signatures in this file match those in
 * libpal/io/io.h. The PAL implementation must provide a version of these
 * functions which meet the requirements stated in the documentation contained
 * in that file.
 */

/**
 * Read from an IO descriptor
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_io_read() (see libpal/io/io.h)
 *
 */
typedef enum pal_error (*pal_io_read_t)(pal_io_t io, uint8_t *buf, size_t n,
                                        size_t *nread, uint64_t timeout_us);

/**
 * Write to an IO descriptor
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_io_write() (see libpal/io/io.h)
 *
 */
typedef enum pal_error (*pal_io_write_t)(pal_io_t io, const uint8_t *buf,
                                         size_t n, size_t *nwritten,
                                         uint64_t timeout_us);

/**
 * Close an open IO descriptor
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_io_close() (see libpal/io/io.h)
 *
 */
typedef enum pal_error (*pal_io_close_t)(pal_io_t io);

/**
 * Flush an IO descriptor
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_io_flush() (see libpal/io/io.h)
 *
 */
typedef enum pal_error (*pal_io_flush_t)(pal_io_t io);

/**
 * Watch several IO descriptors
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_io_watch() (see libpal/io/io.h)
 *
 */
typedef enum pal_error (*pal_io_watch_t)(pal_io_t *readset, size_t *nread,
                                         pal_io_t *writeset, size_t *nwrite);

/**
 * Test an IO descriptor for EOF condition
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_io_eof() (see libpal/io/io.h)
 *
 */
typedef enum pal_error (*pal_io_eof_t)(pal_io_t io, bool *eof);

/**
 * Return the underlying type of an IO descriptor
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_io_get_type() (see libpal/io/io.h)
 *
 */
typedef enum pal_error (*pal_io_get_type_t)(pal_io_t io,
                                            enum pal_io_type *type);

/**
 * PAL IO implementation definition
 */
struct pal_impl_io {
  /// Implementation IO read routine
  pal_io_read_t read;
  /// Implementation IO write routine
  pal_io_write_t write;
  /// Implementation IO descriptor close routine
  pal_io_close_t close;
  /// Implementation IO descriptor flush routine
  pal_io_flush_t flush;
  /// Implementation watch multiple IO descriptors routine
  pal_io_watch_t watch;
  /// Implementation test EOF condition routine
  pal_io_eof_t eof;
  /// Implementation get IO descriptor type routine
  pal_io_get_type_t get_type;
};

/**
 * Install PAL IO implementation in to API
 *
 * Call this function during pal_impl_init to register the implementation IO
 * module with the libpal API
 *
 * @param impl IO implementation definition
 */
void pal_set_impl_io(struct pal_impl_io *impl);

#ifdef __cplusplus
}
#endif

#endif
