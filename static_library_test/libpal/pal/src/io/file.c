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
#include <libpal/impl/io/file.h>
#include <libpal/io/file.h>
#include <libpal/require.h>

#include <pal_private.h>

static struct pal_impl_file file_impl = {
    NULL, NULL, NULL, NULL, NULL, NULL,
};

void pal_reset_impl_file(void) { memset(&file_impl, 0, sizeof(file_impl)); }

enum pal_error pal_set_impl_file(struct pal_impl_file *impl) {
  enum pal_error ret = pal_require(NULL != impl && NULL != impl->open &&
                                   NULL != impl->read && NULL != impl->write &&
                                   NULL != impl->close && NULL != impl->eof

  );
  if (ret == PAL_SUCCESS) {
    file_impl = *impl;
  }
  return ret;
}

bool pal_has_impl_file() { return file_impl.open != NULL; }

enum pal_error pal_file_open(pal_file_t *file, const char *path,
                             enum pal_access_mode mode) {
  enum pal_error ret =
      pal_require(pal_has_impl_file() && file != NULL && path != NULL &&
                  pal_validate_access_mode(mode));
  if (ret == PAL_SUCCESS) {
    ret = file_impl.open(file, path, mode);
  }
  return ret;
}

enum pal_error pal_file_flush(pal_file_t file) {
  enum pal_error ret = pal_require(pal_has_impl_file() && file != NULL);
  if (ret == PAL_SUCCESS) {
    ret = file_impl.flush(file);
  }
  return ret;
}

enum pal_error pal_file_read(pal_file_t file, uint8_t *buf, size_t n,
                             size_t *nread, enum pal_blocking_mode mode,
                             uint64_t timeout_us) {
  INIT_OUTPUT_PARAM(nread, 0);
  enum pal_error ret =
      pal_require(pal_has_impl_file() && file != NULL && buf != NULL &&
                  nread != NULL && pal_validate_blocking_mode(mode));
  if (ret == PAL_SUCCESS) {
    ret = file_impl.read(file, buf, n, nread, mode, timeout_us);
  }
  return ret;
}

enum pal_error pal_file_write(pal_file_t file, const uint8_t *buf, size_t n,
                              size_t *nwritten, enum pal_blocking_mode mode,
                              uint64_t timeout_us) {
  INIT_OUTPUT_PARAM(nwritten, 0);
  enum pal_error ret =
      pal_require(pal_has_impl_file() && file != NULL && buf != NULL &&
                  nwritten != NULL && pal_validate_blocking_mode(mode));
  if (ret == PAL_SUCCESS) {
    ret = file_impl.write(file, buf, n, nwritten, mode, timeout_us);
  }
  return ret;
}

enum pal_error pal_file_close(pal_file_t *file) {
  enum pal_error ret = pal_require(file != NULL);
  if (ret != PAL_SUCCESS) {
    return ret;
  }
  if (!*file) {
    return PAL_SUCCESS;
  }
  ret = pal_require(pal_has_impl_file());
  if (ret == PAL_SUCCESS) {
    ret = file_impl.close(*file);
    *file = NULL;
  }
  return ret;
}

enum pal_error pal_file_eof(pal_file_t file, bool *eof) {
  enum pal_error ret =
      pal_require(pal_has_impl_file() && file != NULL && eof != NULL);
  if (ret == PAL_SUCCESS) {
    ret = file_impl.eof(file, eof);
  }
  return ret;
}
