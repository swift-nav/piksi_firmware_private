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

#ifndef LIBPAL_IMPL_IO_SERIAL_H
#define LIBPAL_IMPL_IO_SERIAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <libpal/error.h>
#include <libpal/impl/blocking_mode.h>
#include <libpal/impl/io/access_mode.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void *pal_serial_t;

/**
 * Open a serial device
 *
 * This function must initialize and make ready a serial/UART device. \p path
 * will be set to the unique system identifier for the UART device. This
 * function must create a serial handle and write it in to the value pointed at
 * by the \p io parameter. This handle will be used in any subsequent read/write
 * operations, and finally passed in to pal_serial_close.
 *
 * Should any error occur during initialization of the UART this function must
 * return some appropriate error code.
 *
 * @param serial On success this function must write an IO handle which can be
 * used in pal_serial_read/pal_serial_write
 * @param path System unique serial port identifier
 * @param mode Access mode, rdonly/wronly/rdwr
 * @param brate Baud rate
 * @param bsize Byte size
 * @param parity Parity mode
 * @param stop Stop bits
 * @param rts Flow control on/off
 */
typedef enum pal_error (*pal_serial_open_t)(
    pal_serial_t *serial, const char *path, enum pal_access_mode mode,
    int brate, uint8_t bsize, char parity, uint8_t stop, bool rts);

/**
 * Read from a serial port
 *
 * This function will be called to read data from a previously opened serial
 * handle. The handle passed in the \p io parameter will be that which was
 * returned from an earlier call to #pal_serial_open_t
 *
 * This function must read up to \p n bytes from the serial handle and write
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
 * immediately if it is unable to read any data from the serial handle. If it is
 * able to read at least 1 byte it should do so, updating the \p nread parameter
 * as required.
 *
 * When \p mode is set to PAL_BLOCKING this function may block the caller for up
 * to the microsecond duration specified in the \p timeout_us parameter. If at
 * any point before the timeout expires some data becomes available on the
 * serial handle this function must write it in to the \p buf, update \p nread,
 * and return PAL_SUCCESS. Should the timeout period expire without any data
 * becoming available this function must return PAL_TIMEOUT without altering any
 * other parameters.
 *
 * The special timeout value of 0 means block indefinitely.
 *
 * @param serial Serial handle
 * @param buf Buffer into which to write received data
 * @param n Maximum number of bytes to read from the serial handle
 * @param nread On success must be updated with the actual number of bytes
 * written in the \p buf
 * @param mode Blocking mode
 * @param timeout_us Maximum blocking duration, only valid when \p mode is
 * PAL_BLOCKING
 * @return PAL error code
 */
typedef enum pal_error (*pal_serial_read_t)(pal_serial_t serial, uint8_t *buf,
                                            size_t n, size_t *nread,
                                            enum pal_blocking_mode mode,
                                            uint64_t timeout_us);

/**
 * Write to a serial handle
 *
 * This function will be called to write data to a previously opened serial
 * handle. The handle passed in the \p io parameter will be that which was
 * returned from an earlier call to #pal_serial_open_t.
 *
 * This function must write up to \p n bytes to the serial handle taken from the
 * buffer pointed at by \p buf. Should a call to this function succeed in
 * writing any amount of data at all the actual number of bytes written to the
 * device must be saved in to the value pointed at by the \p nwritten parameter.
 *
 * It is acceptable for this function to not write any data at all. In this case
 * it may return PAL_SUCCESS without altering any other parameters.
 *
 * \p mode controls the blocking behaviour of this function. If set to
 * PAL_NONBLOCKING a call to this function must return PAL_WOULD_BLOCK
 * immediately if it is unable to write any data to the serial handle. If it is
 * able to write at least 1 byte it should do so, updating the \p nwritten
 * parameter as required.
 *
 * When \p mode is set to PAL_BLOCKING this function may block the caller for up
 * to the microsecond duration specified in the \p timeout_us parameter. If at
 * any point before the timeout expires some data can be written to the serial
 * handle this function must do so, update \p nwritten, and return PAL_SUCCESS.
 * Should the timeout period expire without it being possible to write any data
 * this function must return PAL_TIMEOUT without altering any other parameters.
 *
 * The special timeout value of 0 means block indefinitely.
 *
 * @param serial Serial handle
 * @param buf Buffer from which to write data
 * @param n Maximum number of bytes to write to the serial handle
 * @param nwritten On success must be updated with the actual number of bytes
 * written to the serial handle
 * @param mode Blocking mode
 * @param timeout_us Maximum blocking duration, only valid when \p mode is
 * PAL_BLOCKING
 * @return PAL error code
 *
 */
typedef enum pal_error (*pal_serial_write_t)(pal_serial_t serial,
                                             const uint8_t *buf, size_t n,
                                             size_t *nwritten,
                                             enum pal_blocking_mode mode,
                                             uint64_t timeout_us);

/**
 * Close an open serial port
 *
 * When the user of libpal no longer requires an opened serial port handle it
 * will call this function. The platform implementation can finalise any pending
 * write operations, discard any data in read buffers (if any), free associated
 * resources, and release the handle.
 *
 * @param serial Serial handle previously returned from pal_serial_open
 * @return PAL error code
 */
typedef enum pal_error (*pal_serial_close_t)(pal_serial_t serial);

/**
 * Test a serial handle for EOF condition
 *
 * An EOF condition is one where the serial handle has become invalid and no
 * further read or write operations may be performed on it. The cause of this
 * condition can be any terminal condition for the underlying platform handle.
 * When an serial handle is in the EOF state the only valid operation which can
 * be performed on it is close.
 *
 * This function must set the value pointer to by \p eof to true if the serial
 * handle is in an EOF state.
 *
 * @param serial Serial handle previously returned from pal_serial_open
 * @param eof On success this function must indicate the EOF state of the device
 * @return PAL error code
 */
typedef enum pal_error (*pal_serial_eof_t)(pal_serial_t serial, bool *eof);

/**
 * Flush a serial handle
 *
 * The function must immediately flush any buffered unwritten data (if any) to
 * the open serial device, blocking for as long as is necessary.
 *
 * This function is only valid for serial handles which were opened with write
 * access
 *
 * @param serial Serial handle as returned from #pal_serial_open_t
 * @return PAL error code
 */
typedef enum pal_error (*pal_serial_flush_t)(pal_serial_t serial);

/**
 * PAL Serial IO implementation definition
 */
struct pal_impl_serial {
  pal_serial_open_t open;
  pal_serial_read_t read;
  pal_serial_write_t write;
  pal_serial_close_t close;
  pal_serial_eof_t eof;
  pal_serial_flush_t flush;
};

/**
 * Install PAL serial IO implementation in to API
 *
 * Call this function during pal_impl_init to register the implementation's
 * serial IO module with the libpal API
 *
 * @param impl Serial IO implementation definition
 * @return PAL error code
 */
enum pal_error pal_set_impl_serial(struct pal_impl_serial *impl);

#ifdef __cplusplus
}
#endif

#endif
