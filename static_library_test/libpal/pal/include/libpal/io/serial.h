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

#ifndef LIBPAL_IO_SERIAL_H
#define LIBPAL_IO_SERIAL_H

#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>

#include <libpal/impl/io/serial.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Check global PAL serial device implementation
 * @return true if global PAL serial device implementation has been set
 * otherwise false
 */
bool pal_has_impl_serial(void);

/**
 * Open a serial device with the given parameters
 *
 * This function will open and initialize the serial device and create an serial
 * handle which can be used for future operations. The handle will be returned
 * in the serial parameter, and the return code will indicate the function
 * success. The serial handle will be valid only if this function returns
 * PAL_SUCCESS
 *
 * @param serial New serial handle (out)
 * @param path Path to serial device in whatever format the platform expects
 * @param mode Access mode
 * @param brate Bit rate, must a standard serial/UART speed
 * @param bsize Byte size, must be 7 or 8.
 * @param parity Parity mode, must be 'O', 'N', or 'E'
 * @param stop Stop bit size, must be 1 or 2
 * @param rts RTS flow control on/off
 * @return PAL error code
 */
enum pal_error pal_serial_open(pal_serial_t *serial, const char *path,
                               enum pal_access_mode mode, int brate,
                               uint8_t bsize, char parity, uint8_t stop,
                               bool rts);

/**
 * Read data from a serial port
 *
 * This function will read up to \p n bytes from the specified serial handle and
 * write them into the provided buffer.
 *
 * Reading can be done either as a blocking or non blocking operation as
 * controlled by the \p mode parameter. When operating in non blocking mode this
 * function shall return immediately with a code of #PAL_WOULD_BLOCK if it is
 * not able to immediately read any data from the serial handle. In blocking
 * mode the call shall block until either data becomes available to read, an
 * error occurs on the serial handle, or the timeout period given in \p
 * timeout_us expires. The timeout must be specified in microseconds, the
 * special value of 0 means block indefinitely. Should the timeout expire the
 * function will return PAL_TIMEOUT.
 *
 *
 * If the caller sets the \p n parameter as zero, than the function will return
 * the current error status of the serial handle.
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
 * @param serial Serial handle
 * @param buf Buffer to read data in to
 * @param n Number of bytes to read
 * @param nread Number of bytes actually read
 * @param mode Blocking mode
 * @param timeout_us Timeout in microseconds
 * @return PAL error code
 */
enum pal_error pal_serial_read(pal_serial_t serial, uint8_t *buf, size_t n,
                               size_t *nread, enum pal_blocking_mode mode,
                               uint64_t timeout_us);

/**
 * Write data to a serial port
 *
 * This function will write up to \p n bytes from the specified buffer to the
 * serial handle
 *
 * Writing can be done either as a blocking or non blocking operation as
 * controlled by the \p mode parameter. When operating in non blocking mode this
 * function shall return immediately with a code of #PAL_WOULD_BLOCK if it is
 * not able to immediately read any data from the serial handle. In blocking
 * mode the call shall block until either some or all of the data has been
 * written, an error occurs on the serial handle, or the timeout period given in
 * \p timeout_us expires. The timeout must be specified in microseconds, the
 * special value of 0 means block indefinitely. Should the timeout expire the
 * function will return PAL_TIMEOUT.
 *
 * If the caller sets the \p n parameter as zero, then the function will return
 * the current error status of the serial handle.
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
 * @param serial Serial handle
 * @param buf Buffer from which to write data
 * @param n Number of bytes to write
 * @param nwritten Number of bytes actually written
 * @param mode Blocking mode
 * @param timeout_us Timeout in microseconds
 * @return PAL error code
 */
enum pal_error pal_serial_write(pal_serial_t serial, const uint8_t *buf,
                                size_t n, size_t *nwritten,
                                enum pal_blocking_mode mode,
                                uint64_t timeout_us);

/**
 * Close an open serial handle
 *
 * Calling this function will make the serial handle unavailable for more read
 * or writes. It may be opened again by calling #pal_serial_open with the same
 * path. After this function returns success the handle will be set to NULL must
 * not be reused.
 *
 * @param serial Pointer to open serial handle
 * @return PAL error code
 */
enum pal_error pal_serial_close(pal_serial_t *serial);

/**
 * Test whether a serial handle is in EOF condition
 *
 * The term EOF means that this serial handle is unavailable for further IO
 * activity but has not yet been closed. This is could be for any reason related
 * to the underlying platform. The only valid operation on a serial handle in
 * this state is to call #pal_serial_close to clean up any resources.
 *
 * @param serial Serial handle
 * @param eof On success will be set true if the serial handle is in EOF state,
 * false otherwise
 * @return PAL error code
 */
enum pal_error pal_serial_eof(pal_serial_t serial, bool *eof);

/**
 * Flush all pending data
 *
 * If any data written to this serial port has been buffered internally this
 * function causes it to be immediately flushed out to the underlying device.
 * This is not a valid call for files opened in read only mode.
 *
 * @param serial Serial handle
 * @return PAL error code
 */
enum pal_error pal_serial_flush(pal_serial_t serial);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // LIBPAL_IO_SERIAL_H
