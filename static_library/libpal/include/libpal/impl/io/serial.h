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

#ifndef LIBPAL_IMPL_IO_SERIAL_H
#define LIBPAL_IMPL_IO_SERIAL_H

#include <libpal/io/io.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * PAL serial IO Implementation
 *
 * This file defines the interface a PAL implementation must use to install its
 * own serial handling ability in to the libpal API
 *
 * The function pointer names and signatures in this file match those in
 * libpal/io/serial.h. The PAL implementation must provide a version of these
 * functions which meet the requirements stated in the documentation contained
 * in that file.
 */

/**
 * Open a serial device
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_serial_open() (see libpal/io/serial.h)
 *
 */
typedef enum pal_error (*pal_serial_open_t)(const char *path,
                                            enum pal_access_mode mode,
                                            int brate, uint8_t bsize,
                                            char parity, uint8_t stop, bool rts,
                                            pal_io_t *io);

/**
 * PAL Serial IO implementation definition
 */
struct pal_impl_serial {
  /// Implementation serial IO open routine
  pal_serial_open_t open;
};

/**
 * Install PAL serial IO implementation in to API
 *
 * Call this function during pal_init_impl to register the implementation's
 * serial IO module with the libpal API
 *
 * @param impl Serial IO implementation definition
 */
void pal_set_impl_serial(struct pal_impl_serial *impl);

#ifdef __cplusplus
}
#endif

#endif
