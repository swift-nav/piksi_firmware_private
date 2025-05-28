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

#ifndef LIBPAL_IMPL_IO_FILE_H
#define LIBPAL_IMPL_IO_FILE_H

#include <stddef.h>
#include <stdint.h>

#include <libpal/error.h>
#include <libpal/impl/blocking_mode.h>
#include <libpal/impl/io/access_mode.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void *pal_file_t;

/**
 * Open a file
 *
 * This function must locate and open a file on the platform filesystem and open
 * it with the given access mode. If the file was opened successfully the
 * function must create a unique handle for the file, set it in the value
 * pointed to by the \p io parameter, and return PAL_SUCCESS. The handle created
 * by this function will be passed back in to calls to pal_impl_io_read,
 * pal_impl_io_write, and finally pal_impl_io_close.
 *
 * Should any error occur during location of opening of the file this function
 * must return some other appropriate error code
 *
 * Starling does not require read/write access to files, the \p mode parameter
 * will never be set to PAL_ACCESS_RDWR.
 *
 * @param io On success the function must set this to a unique handle for the
 * opened file
 * @param path File path
 * @param mode Access mode (PAL_ACCESS_RDONLY or PAL_ACCESS_WRONLY)
 */
typedef enum pal_error (*pal_file_open_t)(pal_file_t *file, const char *path,
                                          enum pal_access_mode mode);

/**
 * Flush a file IO descriptor
 *
 * The function must immediately flush any buffered unwritten data (if any) to
 * the open file device, blocking for as long as is necessary.
 *
 * @param io File type IO device handle as returned from #pal_file_open_t
 * @return PAL error code
 */
typedef enum pal_error (*pal_file_flush_t)(pal_file_t file);

/**
 * Read from a file descriptor
 *
 * This function will be called to read data from a previously opened file
 * descriptor. The handle passed in the \p io parameter will be that which was
 * returned from an earlier call to #pal_file_open_t
 *
 * This function must read up to \p n bytes from the file descriptor and write
 * them in to the buffer pointed at by \p buf. Should a call to this function
 * succeed in reading any amount of data at all the actual number of bytes read
 * from the device must be written in to the value pointed at by the \p nread
 * parameter.
 *
 * It is acceptable for this function to not read any data at all. In this case
 * it may return PAL_SUCCESS without altering any other parameters providing
 * that the reason for not reading anything is not due to an error.
 *
 * \p mode controls the blocking behaviour of this function. If set to
 * PAL_NONBLOCKING a call to this function must return PAL_WOULD_BLOCK
 * immediately if it is unable to read any data from the file descriptor. If it
 * is able to read at least 1 byte it should do so, updating the \p nread
 * parameter as required.
 *
 * When \p mode is set to PAL_BLOCKING this function may block the caller for up
 * to the microsecond duration specified in the \p timeout_us parameter. If at
 * any point before the timeout expires some data becomes available on the file
 * descriptor this function must write it in to the \p buf, update \p nread, and
 * return PAL_SUCCESS. Should the timeout period expire without any data
 * becoming available this function must return PAL_TIMEOUT without altering any
 * other parameters.
 *
 * The special timeout value of 0 means block indefinitely.
 *
 * @param file File descriptor
 * @param buf Buffer into which to write received data
 * @param n Maximum number of bytes to read from the file descriptor
 * @param nread On success must be updated with the actual number of bytes
 * written in the \p buf
 * @param mode Blocking mode
 * @param timeout_us Maximum blocking duration, only valid when \p mode is
 * PAL_BLOCKING
 * @return PAL error code
 */
typedef enum pal_error (*pal_file_read_t)(pal_file_t file, uint8_t *buf,
                                          size_t n, size_t *nread,
                                          enum pal_blocking_mode mode,
                                          uint64_t timeout_us);

/**
 * Write to a file descriptor
 *
 * This function will be called to write data to a previously opened file
 * descriptor. The handle passed in the \p io parameter will be that which was
 * returned from an earlier call to #pal_file_open_t.
 *
 * This function must write up to \p n bytes to the file descriptor taken from
 * the buffer pointed at by \p buf. Should a call to this function succeed in
 * writing any amount of data at all the actual number of bytes written to the
 * device must be saved in to the value pointed at by the \p nwritten parameter.
 *
 * It is acceptable for this function to not write any data at all. In this case
 * it may return PAL_SUCCESS without altering any other parameters.
 *
 * \p mode controls the blocking behaviour of this function. If set to
 * PAL_NONBLOCKING a call to this function must return PAL_WOULD_BLOCK
 * immediately if it is unable to write any data to the file descriptor. If it
 * is able to write at least 1 byte it should do so, updating the \p nwritten
 * parameter as required.
 *
 * When \p mode is set to PAL_BLOCKING this function may block the caller for up
 * to the microsecond duration specified in the \p timeout_us parameter. If at
 * any point before the timeout expires some data can be written to the file
 * descriptor this function must do so, update \p nwritten, and return
 * PAL_SUCCESS. Should the timeout period expire without it being possible to
 * write any data this function must return PAL_TIMEOUT without altering any
 * other parameters.
 *
 * The special timeout value of 0 means block indefinitely.
 *
 * @param file File descriptor
 * @param buf Buffer from which to write data
 * @param n Maximum number of bytes to write to the file descriptor
 * @param nwritten On success must be updated with the actual number of bytes
 * written to the file descriptor
 * @param mode Blocking mode
 * @param timeout_us Maximum blocking duration, only valid when \p mode is
 * PAL_BLOCKING
 * @return PAL error code
 */
typedef enum pal_error (*pal_file_write_t)(pal_file_t file, const uint8_t *buf,
                                           size_t n, size_t *nwritten,
                                           enum pal_blocking_mode mode,
                                           uint64_t timeout_us);

/**
 * Close an open file descriptor
 *
 * When the user of libpal no longer requires an opened file it will call
 * this function. The platform implementation can finalise any pending write
 * operations, discard any data in read buffers (if any), free associated
 * resources, and release the handle.
 *
 * @param file File descriptor previously returned from #pal_file_open_t
 * @return PAL error code
 */
typedef enum pal_error (*pal_file_close_t)(pal_file_t file);

/**
 * Test a file descriptor for EOF condition
 *
 * An EOF condition is one where the file descriptor has become invalid and no
 * further read or write operations may be performed on it. Such a condition
 * might be a literal end-of-file, or any other terminal condition for the
 * underlying platform handle. When a file descriptor is in the EOF state the
 * only valid operation which can be performed on it is close.
 *
 * This function must set the value pointer to by \p eof to true if the file
 * descriptor is in an EOF state.
 *
 * @param file File descriptor returned from #pal_file_open_t
 * @param eof On success this function must indicate the EOF state of the device
 * @return PAL error code
 */
typedef enum pal_error (*pal_file_eof_t)(pal_file_t file, bool *eof);

/**
 * PAL File IO implementation definition
 */
struct pal_impl_file {
  pal_file_open_t open;
  pal_file_flush_t flush;
  pal_file_read_t read;
  pal_file_write_t write;
  pal_file_close_t close;
  pal_file_eof_t eof;
};

/**
 * Install PAL file IO implementation in to API
 *
 * Call this function during pal_impl_init to register the implementation's file
 * IO module with the libpal API
 *
 * @param impl File IO implementation definition
 * @return PAL error code
 */
enum pal_error pal_set_impl_file(struct pal_impl_file *impl);

#ifdef __cplusplus
}
#endif

#endif
