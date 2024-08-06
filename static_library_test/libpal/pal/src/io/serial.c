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

#include <string.h>

#include <libpal/error.h>
#include <libpal/impl/io/serial.h>
#include <libpal/io/serial.h>
#include <libpal/require.h>
#include <pal_private.h>

static struct pal_impl_serial serial_impl = {
    NULL, NULL, NULL, NULL, NULL, NULL,
};

void pal_reset_impl_serial(void) {
  memset(&serial_impl, 0, sizeof(serial_impl));
}

enum pal_error pal_set_impl_serial(struct pal_impl_serial *impl) {
  enum pal_error ret = pal_require(NULL != impl && NULL != impl->open

  );
  if (ret == PAL_SUCCESS) {
    serial_impl = *impl;
  }
  return ret;
}

bool pal_has_impl_serial() { return serial_impl.open != NULL; }

enum pal_error pal_serial_open(pal_serial_t *serial, const char *path,
                               enum pal_access_mode mode, int brate,
                               uint8_t bsize, char parity, uint8_t stop,
                               bool rts) {
  enum pal_error ret =
      pal_require(pal_has_impl_serial() && serial != NULL && path != NULL &&
                  pal_validate_access_mode(mode) && brate > 0 && bsize > 0);
  if (ret == PAL_SUCCESS) {
    ret = serial_impl.open(serial, path, mode, brate, bsize, parity, stop, rts);
  }
  return ret;
}

enum pal_error pal_serial_read(pal_serial_t serial, uint8_t *buf, size_t n,
                               size_t *nread, enum pal_blocking_mode mode,
                               uint64_t timeout_us) {
  INIT_OUTPUT_PARAM(nread, 0);
  enum pal_error ret =
      pal_require(pal_has_impl_serial() && serial != NULL && buf != NULL &&
                  nread != NULL && pal_validate_blocking_mode(mode));
  if (ret == PAL_SUCCESS) {
    ret = serial_impl.read(serial, buf, n, nread, mode, timeout_us);
  }
  return ret;
}

enum pal_error pal_serial_write(pal_serial_t serial, const uint8_t *buf,
                                size_t n, size_t *nwritten,
                                enum pal_blocking_mode mode,
                                uint64_t timeout_us) {
  INIT_OUTPUT_PARAM(nwritten, 0);
  enum pal_error ret =
      pal_require(pal_has_impl_serial() && serial != NULL && buf != NULL &&
                  nwritten != NULL && pal_validate_blocking_mode(mode));
  if (ret == PAL_SUCCESS) {
    ret = serial_impl.write(serial, buf, n, nwritten, mode, timeout_us);
  }
  return ret;
}

enum pal_error pal_serial_close(pal_serial_t *serial) {
  enum pal_error ret = pal_require(serial != NULL);
  if (ret != PAL_SUCCESS) {
    return ret;
  }
  if (!*serial) {
    return PAL_SUCCESS;
  }
  ret = pal_require(pal_has_impl_serial());
  if (ret == PAL_SUCCESS) {
    ret = serial_impl.close(*serial);
    *serial = NULL;
  }
  return ret;
}

enum pal_error pal_serial_eof(pal_serial_t serial, bool *eof) {
  enum pal_error ret =
      pal_require(pal_has_impl_serial() && serial != NULL && eof != NULL);
  if (ret == PAL_SUCCESS) {
    ret = serial_impl.eof(serial, eof);
  }
  return ret;
}

enum pal_error pal_serial_flush(pal_serial_t serial) {
  enum pal_error ret = pal_require(pal_has_impl_serial() && serial != NULL);
  if (ret == PAL_SUCCESS) {
    ret = serial_impl.flush(serial);
  }
  return ret;
}
