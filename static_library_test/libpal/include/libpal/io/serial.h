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

#ifndef LIBPAL_IO_SERIAL_H
#define LIBPAL_IO_SERIAL_H

#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>

#include <libpal/io/io.h>

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
 * This function will open and initialize the serial device and create an IO
 * handle which can be used for future operations. The handle will be returned
 * in the io_out parameter, and the return code will indicate the function
 * success. The IO handle will be valid only if this function returns
 * PAL_SUCCESS
 *
 * @param path Path to serial device in whatever format the platform expects
 * @param mode Access mode
 * @param brate Bit rate, must a standard serial/UART speed
 * @param bsize Byte size, must be 7 or 8.
 * @param parity Parity mode, must be 'O', 'N', or 'E'
 * @param stop Stop bit size, must be 1 or 2
 * @param rts RTS flow control on/off
 * @param io_out New io handle (out)
 * @return PAL error code
 */
enum pal_error pal_serial_open(const char *path, enum pal_access_mode mode,
                               int brate, uint8_t bsize, char parity,
                               uint8_t stop, bool rts, pal_io_t *io);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // LIBPAL_IO_SERIAL_H
