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

#ifndef LIBPAL_IMPL_IO_STDSTREAM_H
#define LIBPAL_IMPL_IO_STDSTREAM_H

#include <stddef.h>
#include <stdint.h>

#include <libpal/error.h>
#include <libpal/impl/blocking_mode.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Write to stdout
 *
 * This function will be called to write data to stdout.
 *
 * This function must write up to \p n bytes to stdout taken from the
 * buffer pointed at by \p buf. Should a call to this function succeed in
 * writing any amount of data at all the actual number of bytes written to the
 * device must be saved in to the value pointed at by the \p nwritten parameter.
 *
 * It is acceptable for this function to not write any data at all. In this case
 * it may return PAL_SUCCESS without altering any other parameters.
 *
 * \p mode controls the blocking behaviour of this function. If set to
 * PAL_NONBLOCKING a call to this function must return PAL_WOULD_BLOCK
 * immediately if it is unable to write any data to stdout. If it is able
 * to write at least 1 byte it should do so, updating the \p nwritten parameter
 * as required.
 *
 * When \p mode is set to PAL_BLOCKING this function may block the caller for up
 * to the microsecond duration specified in the \p timeout_us parameter. If at
 * any point before the timeout expires some data can be written to stdout
 * this function must do so, update \p nwritten, and return PAL_SUCCESS.
 * Should the timeout period expire without it being possible to write any data
 * this function must return PAL_TIMEOUT without altering any other parameters.
 *
 * The special timeout value of 0 means block indefinitely.
 *
 * @param buf Buffer from which to write data
 * @param n Maximum number of bytes to write to stdout
 * @param nwritten On success must be updated with the actual number of bytes
 * written to stdout
 * @param mode Blocking mode
 * @param timeout_us Maximum blocking duration, only valid when \p mode is
 * PAL_BLOCKING
 * @return PAL error code
 *
 */
typedef enum pal_error (*pal_stdout_write_t)(const uint8_t *buf, size_t n,
                                             size_t *nwritten,
                                             enum pal_blocking_mode mode,
                                             uint64_t timeout_us);

/**
 * Flush stdout
 *
 * The function must immediately flush any buffered unwritten data (if any) to
 * stdout, blocking for as long as is necessary.
 *
 * @return PAL error code
 */
typedef enum pal_error (*pal_stdout_flush_t)(void);

/**
 * Write to stderr
 *
 * This function will be called to write data to stderr.
 *
 * This function must write up to \p n bytes to stderr taken from the
 * buffer pointed at by \p buf. Should a call to this function succeed in
 * writing any amount of data at all the actual number of bytes written to the
 * device must be saved in to the value pointed at by the \p nwritten parameter.
 *
 * It is acceptable for this function to not write any data at all. In this case
 * it may return PAL_SUCCESS without altering any other parameters.
 *
 * \p mode controls the blocking behaviour of this function. If set to
 * PAL_NONBLOCKING a call to this function must return PAL_WOULD_BLOCK
 * immediately if it is unable to write any data to stderr. If it is able
 * to write at least 1 byte it should do so, updating the \p nwritten parameter
 * as required.
 *
 * When \p mode is set to PAL_BLOCKING this function may block the caller for up
 * to the microsecond duration specified in the \p timeout_us parameter. If at
 * any point before the timeout expires some data can be written to stderr
 * this function must do so, update \p nwritten, and return PAL_SUCCESS.
 * Should the timeout period expire without it being possible to write any data
 * this function must return PAL_TIMEOUT without altering any other parameters.
 *
 * The special timeout value of 0 means block indefinitely.
 *
 * @param buf Buffer from which to write data
 * @param n Maximum number of bytes to write to stderr
 * @param nwritten On success must be updated with the actual number of bytes
 * written to stderr
 * @param mode Blocking mode
 * @param timeout_us Maximum blocking duration, only valid when \p mode is
 * PAL_BLOCKING
 * @return PAL error code
 *
 */
typedef enum pal_error (*pal_stderr_write_t)(const uint8_t *buf, size_t n,
                                             size_t *nwritten,
                                             enum pal_blocking_mode mode,
                                             uint64_t timeout_us);

/**
 * Flush stderr
 *
 * The function must immediately flush any buffered unwritten data (if any) to
 * stderr, blocking for as long as is necessary.
 *
 * @return PAL error code
 */
typedef enum pal_error (*pal_stderr_flush_t)(void);

/**
 * Read from stdint
 *
 * This function will be called to read data from stdin.
 *
 * This function must read up to \p n bytes from stdin and write them in
 * to the buffer pointed at by \p buf. Should a call to this function succeed in
 * reading any amount of data at all the actual number of bytes read from the
 * device must be written in to the value pointed at by the \p nread parameter.
 *
 * It is acceptable for this function to not read any data at all. In this case
 * it may return PAL_SUCCESS without altering any other parameters providing
 * that the reason for not reading anything is not due to an error.
 *
 * \p mode controls the blocking behaviour of this function. If set to
 * PAL_NONBLOCKING a call to this function must return PAL_WOULD_BLOCK
 * immediately if it is unable to read any data from stdin. If it is
 * able to read at least 1 byte it should do so, updating the \p nread parameter
 * as required.
 *
 * When \p mode is set to PAL_BLOCKING this function may block the caller for up
 * to the microsecond duration specified in the \p timeout_us parameter. If at
 * any point before the timeout expires some data becomes available on stdin
 * this function must write it in to the \p buf, update \p nread, and
 * return PAL_SUCCESS. Should the timeout period expire without any data
 * becoming available this function must return PAL_TIMEOUT without altering any
 * other parameters.
 *
 * The special timeout value of 0 means block indefinitely.
 *
 * @param buf Buffer into which to write received data
 * @param n Maximum number of bytes to read from stdin
 * @param nread On success must be updated with the actual number of bytes
 * written in the \p buf
 * @param mode Blocking mode
 * @param timeout_us Maximum blocking duration, only valid when \p mode is
 * PAL_BLOCKING
 * @return PAL error code
 */
typedef enum pal_error (*pal_stdin_read_t)(uint8_t *buf, size_t n,
                                           size_t *nread,
                                           enum pal_blocking_mode mode,
                                           uint64_t timeout_us);

/**
 * Test stdin for EOF condition
 *
 * An EOF condition is one where stdin has become invalid and no further
 * read operations may be performed on it.
 *
 * This function must set the value pointer to by \p eof to true if stdin
 * is in an EOF state.
 *
 * @param eof On success this function must indicate the EOF state of the device
 * @return PAL error code
 */
typedef enum pal_error (*pal_stdin_eof_t)(bool *eof);

/**
 * PAL Standard stream IO implementation definition
 */
struct pal_impl_stdstream {
  pal_stdout_write_t stdout_write;
  pal_stdout_flush_t stdout_flush;
  pal_stderr_write_t stderr_write;
  pal_stderr_flush_t stderr_flush;
  pal_stdin_read_t stdin_read;
  pal_stdin_eof_t stdin_eof;
};

/**
 * Install PAL standard stream IO implementation in to API
 *
 * Call this function during pal_impl_init to register the implementation's
 * standard stream IO module with the libpal API
 *
 * @param impl Standard stream IO implementation definition
 * @return PAL error code
 */
enum pal_error pal_set_impl_stdstream(struct pal_impl_stdstream *impl);

#ifdef __cplusplus
}
#endif

#endif
