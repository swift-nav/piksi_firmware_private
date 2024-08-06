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

#ifndef LIBPAL_IO_STDSTREAM_H
#define LIBPAL_IO_STDSTREAM_H

#include <stdbool.h>

#include <libpal/impl/io/stdstream.h>

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
 * Write data to stdout
 *
 * This function will write up to \p n bytes from the specified buffer to stdout
 *
 * Writing can be done either as a blocking or non blocking operation as
 * controlled by the \p mode parameter. When operating in non blocking mode this
 * function shall return immediately with a code of #PAL_WOULD_BLOCK if it is
 * not able to immediately read any data from stdout. In blocking
 * mode the call shall block until either some or all of the data has been
 * written, an error occurs on stdout, or the timeout period given in \p
 * timeout_us expires. The timeout must be specified in microseconds, the
 * special value of 0 means block indefinitely. Should the timeout expire the
 * function will return PAL_TIMEOUT.
 *
 * If the caller sets the \p n parameter as zero, then the function will return
 * the current error status of stdout.
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
 * @param buf Buffer from which to write data
 * @param n Number of bytes to write
 * @param nwritten Number of bytes actually written
 * @param mode Blocking mode
 * @param timeout_us Timeout in microseconds
 * @return PAL error code
 */
enum pal_error pal_stdout_write(const uint8_t *buf, size_t n, size_t *nwritten,
                                enum pal_blocking_mode mode,
                                uint64_t timeout_us);

/**
 * Flush all pending data on stdout
 *
 * If any data written to stdout has been buffered internally this function
 * causes it to be immediately flushed out.
 *
 * @return PAL error code
 */
enum pal_error pal_stdout_flush(void);

/**
 * Write data to stderr
 *
 * This function will write up to \p n bytes from the specified buffer to stderr
 *
 * Writing can be done either as a blocking or non blocking operation as
 * controlled by the \p mode parameter. When operating in non blocking mode this
 * function shall return immediately with a code of #PAL_WOULD_BLOCK if it is
 * not able to immediately read any data from stderr. In blocking
 * mode the call shall block until either some or all of the data has been
 * written, an error occurs on stderr, or the timeout period given in \p
 * timeout_us expires. The timeout must be specified in microseconds, the
 * special value of 0 means block indefinitely. Should the timeout expire the
 * function will return PAL_TIMEOUT.
 *
 * If the caller sets the \p n parameter as zero, then the function will return
 * the current error status of stderr.
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
 * @param buf Buffer from which to write data
 * @param n Number of bytes to write
 * @param nwritten Number of bytes actually written
 * @param mode Blocking mode
 * @param timeout_us Timeout in microseconds
 * @return PAL error code
 */
enum pal_error pal_stderr_write(const uint8_t *buf, size_t n, size_t *nwritten,
                                enum pal_blocking_mode mode,
                                uint64_t timeout_us);

/**
 * Flush all pending data on stderr
 *
 * If any data written to stderr has been buffered internally this function
 * causes it to be immediately flushed out.
 *
 * @return PAL error code
 */
enum pal_error pal_stderr_flush(void);

/**
 * Read data from stdin
 *
 * This function will read up to \p n bytes from stdin and
 * write them into the provided buffer.
 *
 * Reading can be done either as a blocking or non blocking operation as
 * controlled by the \p mode parameter. When operating in non blocking mode this
 * function shall return immediately with a code of #PAL_WOULD_BLOCK if it is
 * not able to immediately read any data from stdin. In blocking
 * mode the call shall block until either data becomes available to read, an
 * error occurs on stdin, or the timeout period given in \p timeout_us
 * expires. The timeout must be specified in microseconds, the special value of
 * 0 means block indefinitely. Should the timeout expire the function will
 * return PAL_TIMEOUT.
 *
 *
 * If the caller sets the \p n parameter as zero, than the function will return
 * the current error status of stdin.
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
 * @param buf Buffer to read data in to
 * @param n Number of bytes to read
 * @param nread Number of bytes actually read
 * @param mode Blocking mode
 * @param timeout_us Timeout in microseconds
 * @return PAL error code
 */
enum pal_error pal_stdin_read(uint8_t *buf, size_t n, size_t *nread,
                              enum pal_blocking_mode mode, uint64_t timeout_us);

/**
 * Test whether stdin is in EOF condition
 *
 * The term EOF means that stdin is unavailable for further IO activity.
 *
 * @param eof On success will be set true if stdin is in EOF state,
 * false otherwise
 * @return PAL error code
 */
enum pal_error pal_stdin_eof(bool *eof);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // LIBPAL_IO_STDSTREAM_H
