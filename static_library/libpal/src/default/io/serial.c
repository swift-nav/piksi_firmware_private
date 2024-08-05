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

#include <stddef.h>

#include <libpal/error.h>
#include <libpal/impl/io/serial.h>
#include <libpal/io/serial.h>
#include <pal_private.h>

#include "not_implemented.h"

/**
 * Default serial open routine
 * @param path Unused
 * @param mode Unused
 * @param brate Unused
 * @param bsize Unused
 * @param parity Unused
 * @param stop Unused
 * @param rts Unused
 * @param io Unused
 * @return PAL_INVALID
 */
static enum pal_error default_serial_open(const char *path,
                                          enum pal_access_mode mode, int brate,
                                          uint8_t bsize, char parity,
                                          uint8_t stop, bool rts,
                                          pal_io_t *io) {
  NOT_IMPLEMENTED();
  (void)path;
  (void)mode;
  (void)brate;
  (void)bsize;
  (void)parity;
  (void)stop;
  (void)rts;
  (void)io;
  return PAL_INVALID;
}

static struct pal_impl_serial serial_impl = {
    .open = default_serial_open,
};

void pal_reset_impl_serial(void) { serial_impl.open = default_serial_open; }

void pal_set_impl_serial(struct pal_impl_serial *impl) {
  assert(NULL != impl);
  assert(NULL != impl->open);

  serial_impl = *impl;
}

bool pal_has_impl_serial() { return serial_impl.open != default_serial_open; }

enum pal_error pal_serial_open(const char *path, enum pal_access_mode mode,
                               int brate, uint8_t bsize, char parity,
                               uint8_t stop, bool rts, pal_io_t *io) {
  return serial_impl.open(path, mode, brate, bsize, parity, stop, rts, io);
}
