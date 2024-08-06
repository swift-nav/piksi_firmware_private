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

#ifndef LIBPAL_IO_FILE_H
#define LIBPAL_IO_FILE_H

#include <libpal/impl/io/file.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Check Global PAL File Implementation
 * @return true if global file implementation has be set, otherfile
 * false
 */
bool pal_has_impl_file(void);

/**
 * PAL file open routine.
 *
 * Opens a file and make ready for future access according to the given mode. On
 * success this function will provide a file descriptor in the file parameter
 * which can be used for future read and/or write operations. When the file is
 * no longer needed it should be closed by passing the file descriptor back to
 * pal_file_close(). The new file descriptor will only be valid if this function
 * returns PAL_SUCCESS
 *
 * When opened the file descriptor must be set to reference the first byte of
 * the file, that is to say the first read or write operation on the descriptor
 * will operate on the first byte of the file. When opened in write only mode
 * (PAL_WRONLY) the file will be truncated, when opened in read/write mode
 * (PAL_RDWR) it will not be truncated but the first write operation will still
 * occur at the first byte of the file. When opened with write access
 * (PAL_WRONLY or PAL_RDWR) and the file does not already exist it must be
 * created.
 *
 * @param file PAL file descriptor to be used for later access
 * @param name File name/path
 * @param mode Access mode, PAL_RDONLY, PAL_WRONLY, PAL_RDWR
 * @return PAL error code
 */
enum pal_error pal_file_open(pal_file_t *file, const char *path,
                             enum pal_access_mode mode);

/**
 * Flush all pending data to disk
 *
 * If any data written to this file has been buffered internally this function
 * causes it to be immediately flushed out to the underlying device. This is not
 * a valid call for files opened in read only mode.
 *
 * @param file File descriptor
 * @return PAL error code
 */
enum pal_error pal_file_flush(pal_file_t file);

/**
 * Read data from a file
 *
 * This function will read up to \p n bytes from the specified file descriptor
 * and write them into the provided buffer.
 *
 * Reading can be done either as a blocking or non blocking operation as
 * controlled by the \p mode parameter. When operating in non blocking mode this
 * function shall return immediately with a code of #PAL_WOULD_BLOCK if it is
 * not able to immediately read any data from the file descriptor. In blocking
 * mode the call shall block until either data becomes available to read, an
 * error occurs on the file descriptor, or the timeout period given in \p
 * timeout_us expires. The timeout must be specified in microseconds, the
 * special value of 0 means block indefinitely. Should the timeout expire the
 * function will return PAL_TIMEOUT.
 *
 *
 * If the caller sets the \p n parameter as zero, than the function will return
 * the current error status of the file descriptor.
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
 * @param file File descriptor
 * @param buf Buffer to read data in to
 * @param n Number of bytes to read
 * @param nread Number of bytes actually read
 * @param mode Blocking mode
 * @param timeout_us Timeout in microseconds
 * @return PAL error code
 */
enum pal_error pal_file_read(pal_file_t file, uint8_t *buf, size_t n,
                             size_t *nread, enum pal_blocking_mode mode,
                             uint64_t timeout_us);

/**
 * Write data to a file
 *
 * This function will write up to \p n bytes from the specified buffer to the
 * file descriptor
 *
 * Writing can be done either as a blocking or non blocking operation as
 * controlled by the \p mode parameter. When operating in non blocking mode this
 * function shall return immediately with a code of #PAL_WOULD_BLOCK if it is
 * not able to immediately read any data from the file descriptor. In blocking
 * mode the call shall block until either some or all of the data has been
 * written, an error occurs on the file descriptor, or the timeout period given
 * in \p timeout_us expires. The timeout must be specified in microseconds, the
 * special value of 0 means block indefinitely. Should the timeout expire the
 * function will return PAL_TIMEOUT.
 *
 * If the caller sets the \p n parameter as zero, then the function will return
 * the current error status of the file descriptor.
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
 * @param file File descriptor
 * @param buf Buffer from which to write data
 * @param n Number of bytes to write
 * @param nwritten Number of bytes actually written
 * @param mode Blocking mode
 * @param timeout_us Timeout in microseconds
 * @return PAL error code
 */
enum pal_error pal_file_write(pal_file_t file, const uint8_t *buf, size_t n,
                              size_t *nwritten, enum pal_blocking_mode mode,
                              uint64_t timeout_us);

/**
 * Close an open file descriptor
 *
 * Calling this function will make the file descriptor unavailable for more read
 * or writes. It may be opened again by calling #pal_file_open with the same
 * path. After this function returns success the handle will be set to NULL must
 * not be reused.
 *
 * @param file Pointer to open file descriptor
 * @return PAL error code
 */
enum pal_error pal_file_close(pal_file_t *file);

/**
 * Test whether a file descriptor is in EOF condition
 *
 * The term EOF means that this file descriptor is unavailable for further IO
 * activity but has not yet been closed. This is usually because the file has
 * been read to the end. The only valid operation on a file descriptor in this
 * state is to call #pal_file_close to clean up any resources.
 *
 * @param file File descriptor
 * @param eof On success will be set true if the file descriptor is in EOF
 * state, false otherwise
 * @return PAL error code
 */
enum pal_error pal_file_eof(pal_file_t file, bool *eof);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // LIBPAL_IO_FILE_H
