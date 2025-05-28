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
#include <libpal/impl/io/stdstream.h>
#include <libpal/io/stdstream.h>
#include <libpal/require.h>
#include <pal_private.h>

static struct pal_impl_stdstream stdstream_impl = {
    NULL, NULL, NULL, NULL, NULL, NULL,
};

void pal_reset_impl_stdstream(void) {
  memset(&stdstream_impl, 0, sizeof(stdstream_impl));
}

enum pal_error pal_set_impl_stdstream(struct pal_impl_stdstream *impl) {
  enum pal_error ret =
      pal_require(NULL != impl && NULL != impl->stdout_write &&
                  NULL != impl->stderr_write && NULL != impl->stdin_read &&
                  NULL != impl->stdout_flush && NULL != impl->stderr_flush &&
                  NULL != impl->stdin_eof

      );
  if (ret == PAL_SUCCESS) {
    stdstream_impl = *impl;
  }
  return ret;
}

bool pal_has_impl_stdstream(void) { return stdstream_impl.stdin_read != NULL; }

enum pal_error pal_stdin_read(uint8_t *buf, size_t n, size_t *nread,
                              enum pal_blocking_mode mode,
                              uint64_t timeout_us) {
  INIT_OUTPUT_PARAM(nread, 0);
  enum pal_error ret =
      pal_require(pal_has_impl_stdstream() && buf != NULL && nread != NULL &&
                  pal_validate_blocking_mode(mode));
  if (ret == PAL_SUCCESS) {
    ret = stdstream_impl.stdin_read(buf, n, nread, mode, timeout_us);
  }
  return ret;
}

enum pal_error pal_stdin_eof(bool *eof) {
  enum pal_error ret = pal_require(pal_has_impl_stdstream() && eof != NULL);
  if (ret == PAL_SUCCESS) {
    ret = stdstream_impl.stdin_eof(eof);
  }
  return ret;
}

enum pal_error pal_stdout_write(const uint8_t *buf, size_t n, size_t *nwritten,
                                enum pal_blocking_mode mode,
                                uint64_t timeout_us) {
  INIT_OUTPUT_PARAM(nwritten, 0);
  enum pal_error ret =
      pal_require(pal_has_impl_stdstream() && buf != NULL && nwritten != NULL &&
                  pal_validate_blocking_mode(mode));
  if (ret == PAL_SUCCESS) {
    ret = stdstream_impl.stdout_write(buf, n, nwritten, mode, timeout_us);
  }
  return ret;
}

enum pal_error pal_stdout_flush(void) {
  enum pal_error ret = pal_require(pal_has_impl_stdstream());
  if (ret == PAL_SUCCESS) {
    ret = stdstream_impl.stdout_flush();
  }
  return ret;
}

enum pal_error pal_stderr_write(const uint8_t *buf, size_t n, size_t *nwritten,
                                enum pal_blocking_mode mode,
                                uint64_t timeout_us) {
  INIT_OUTPUT_PARAM(nwritten, 0);
  enum pal_error ret =
      pal_require(pal_has_impl_stdstream() && buf != NULL && nwritten != NULL &&
                  pal_validate_blocking_mode(mode));
  if (ret == PAL_SUCCESS) {
    ret = stdstream_impl.stderr_write(buf, n, nwritten, mode, timeout_us);
  }
  return ret;
}

enum pal_error pal_stderr_flush(void) {
  enum pal_error ret = pal_require(pal_has_impl_stdstream());
  if (ret == PAL_SUCCESS) {
    ret = stdstream_impl.stderr_flush();
  }
  return ret;
}
