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

#ifndef LIBPAL_IO_IO_H
#define LIBPAL_IO_IO_H

#include <libpal/error.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

// Generic IO handle
typedef void *pal_io_t;

/**
 * IO device access mode. To be specified when opening an IO device for the
 * first time
 */
enum pal_access_mode {
  /**
   * Invalid, do not use
   */
  PAL_ACCESS_INVALID,

  /**
   * Open in read only mode. Write operations will return an error
   */
  PAL_ACCESS_RDONLY,

  /**
   * Open in write only mode. Read operations will return an error
   */
  PAL_ACCESS_WRONLY,

  /**
   * Open in read/write mode. Calls to pal_io_read and pal_io_write are valid
   * for this device
   */
  PAL_ACCESS_RDWR,
};

/**
 * IO descriptor type. Can be used for introspection after a handle has been
 * created
 */
enum pal_io_type {
  /**
   * Invalid, do not use
   */
  PAL_IO_INVALID,

  /**
   * Serial device, created with pal_serial_open
   */
  PAL_IO_SERIAL,

  /**
   * File device, created with pal_file_open
   */
  PAL_IO_FILE,

  /**
   * TCP Server, created with pal_tcp_server_open
   */
  PAL_IO_TCPSERV,

  /**
   * TCP socket, created with pal_tcp_client_open or pal_tcp_server_accept
   */
  PAL_IO_TCPSOCK,

  /**
   * Standard stream, returned from pal_get_stdin, pal_get_stdout, or
   * pal_get_stderr
   */
  PAL_IO_STDSTREAM,
};

/**
 * Check global PAL IO implementation
 *
 * @return true if global PAL IO implementation has been set, otherwise false
 */
bool pal_has_impl_io(void);

/**
 * Read data from an IO device
 *
 * This function will read up to \p n bytes from the specified IO device and
 * write them into the provided buffer. Reading is a blocking operation, caller
 * has the option of setting a maximum duration for which the function should be
 * blocked for via the \p timeout_us parameter.
 *
 * If the caller sets the \p n parameter as zero, than the function will return
 * the current error status of the IO device.
 *
 * The provided buffer must be at least \p n bytes in size to prevent overflows.
 * This function may read fewer than the requested bytes, this is not
 * necessarily an error. The number of bytes actually read will be saved in to
 * the \p nread memory address, as a such, the caller must always provide a
 * valid address otherwise a #PAL_INVALID error will be returned to the caller.
 * The \p nread will always be updated with the correct information, regardless
 * of the situation, if an error (other than #PAL_INVALID) has taken place and
 * no information was read, the memory address will be updated with the value of
 * zero.
 *
 * @param_io IO device handle
 * @param buf Buffer to read data in to
 * @param n Number of bytes to read
 * @param nread Number of bytes actually read
 * @param timeout_us Timeout in microseconds
 * @return PAL error code
 */
enum pal_error pal_io_read(pal_io_t io, uint8_t *buf, size_t n, size_t *nread,
                           uint64_t timeout_us);

/**
 * Write data to an IO device
 *
 * This function will write up to \p n bytes from the specified buffer to the IO
 * device. Writing is a blocking operation, caller has the option of setting a
 * maximum duration for which the function should be blocked for via the \p
 * timeout_us parameter.
 *
 * If the caller sets the \p n parameter as zero, than the function will return
 * the current error status of the IO device.
 *
 * The provided buffer \p buf must be at least \p n bytes in size. This function
 * may write fewer than the requested bytes, this is not necessarily an error.
 * The number of bytes actually written will be saved in to the \p nwritten
 * memory address, as a such, the caller must always provide a valid address
 * otherwise a #PAL_INVALID error will be returned to the caller. The \p
 * nwritten will always be updated with the correct information, regardless of
 * the situation, if an error (other than #PAL_INVALID) has taken place and no
 * information was written, the memory address will be updated with the value of
 * zero.
 *
 * @param io IO device handle
 * @param buf Buffer from which to write data
 * @param n Number of bytes to write
 * @param nwritten Number of bytes actually written
 * @param timeout_us Timeout in microseconds
 * @return PAL error code
 */
enum pal_error pal_io_write(pal_io_t io, const uint8_t *buf, size_t n,
                            size_t *nwritten, uint64_t timeout_us);

/**
 * Close an open IO handle
 *
 * Calling this function will make the device unavailable for more read or
 * writes. It may be opened again by calling the appropriate open function.
 * After this function returns success the handle will be set to NULL must not
 * be reused.
 *
 * @param io Pointer to open IO handle created by a PAL 'open' function
 * @return PAL error code
 */
enum pal_error pal_io_close(pal_io_t *io);

/**
 * Flush all pending data to IO device
 *
 * If any data written to this device has been buffered internally this function
 * cause it to be immediately flushed out to the underlying device. This is not
 * a valid call for IO devices opened in read only mode.
 *
 * @param _io IO device handle
 * @return PAL error code
 */
enum pal_error pal_io_flush(pal_io_t io);

/**
 * Watch multiple IO devices for activity
 *
 * This function will block until some activity is detected on one or more given
 * IO handles. IO handles can be watched for read or write activity.
 *
 * readset and nread specify the handles to be watched for read activity. On
 * entry readset will be populated with all the open IO handles to be watched,
 * and nread will specify the number of handles in readset. On successful
 * return readset will contain the set of given IO devices which have at least
 * 1 byte of data waiting to be read, and nread will be updated accordingly. It
 * is an error to pass an IO handle in readset which is not opened in a mode
 * that allows reading.
 *
 * writeset and nwrite specify the handles to be watched for write
 * availability. On entry writeset will be populated with all the open IO
 * handles to be watched, and nwrite will specify the number of handles in
 * writeset. On successful return writeset will contain the set of handles
 * which are available to be written to, and nwrite will be updated accordingly.
 * It is an error to pass an IO handle in writeset which is not opened in a
 * mode that allows writing.
 *
 * @param readset Set of IO handles to watch for read activity
 * @param nread Size of readset
 * @param writeset Set of IO handles to watch for write availability
 * @param nwrite Size of writeset
 * @return PAL error code
 */
enum pal_error pal_io_watch(pal_io_t *readset, size_t *nread,
                            pal_io_t *writeset, size_t *nwrite);

/**
 * Test whether an IO descriptor is in EOF condition
 *
 * The term EOF does not apply exclusively to files, it is a general condition
 * where the descriptor has become unavailable for further IO activity but has
 * not yet been closed. This could be that a file has been read to the end, or a
 * TCP socket has been unexpectedly disconnected, or any other condition that
 * has left the device in an unusable state. A descriptor which is in the EOF
 * state can not be removed from it, the only valid operation is to call
 * pal_io_close to clean up any resources.
 *
 * @param io IO descriptor
 * @param eof On success will be set true if the IO device is in EOF state,
 * false otherwise
 * @return PAL error code
 */
enum pal_error pal_io_eof(pal_io_t io, bool *eof);

/**
 * Determine the underlying type of an IO descriptor
 *
 * All IO descriptors have the same C type, pal_io_t, but operate on different
 * communication mediums. IO descriptors are created by different function which
 * apply to a specific transport type such as pal_file_open, or
 * pal_tcp_client_open. At runtime a caller can use this function to determine
 * the underlying type of a previously created IO descriptor, but knowledge of
 * this should not affect the usage of that descriptor - every descriptor is
 * still read from and written to by the same functions.
 *
 * @param io PAL IO descriptor
 * @param type On success will be set to the underlying IO descriptor type
 * @return PAL error code
 */
enum pal_error pal_io_get_type(pal_io_t io, enum pal_io_type *type);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // LIBPAL_IO_IO_H
