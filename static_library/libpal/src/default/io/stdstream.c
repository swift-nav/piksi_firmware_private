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

#include <stdio.h>

#include <libpal/error.h>
#include <libpal/impl/io/stdstream.h>
#include <libpal/io/stdstream.h>
#include <pal_private.h>

#include "not_implemented.h"

/**
 * Default get standard input stream routine
 * @param io Unused
 * @return PAL_INVALID
 */
static enum pal_error default_get_stdin(pal_io_t *io) {
  (void)io;
  NOT_IMPLEMENTED();
  return PAL_INVALID;
}

/**
 * Default get standard output stream routine
 * @param io Unused
 * @return PAL_INVALID
 */
static enum pal_error default_get_stdout(pal_io_t *io) {
  (void)io;
  NOT_IMPLEMENTED();
  return PAL_INVALID;
}

/**
 * Default get standard error output stream routine
 * @param io Unused
 * @return PAL_INVALID
 */
static enum pal_error default_get_stderr(pal_io_t *io) {
  (void)io;
  NOT_IMPLEMENTED();
  return PAL_INVALID;
}

static struct pal_impl_stdstream stdstream_impl = {
    .get_stdin = default_get_stdin,
    .get_stdout = default_get_stdout,
    .get_stderr = default_get_stderr,
};

void pal_reset_impl_stdstream(void) {
  stdstream_impl.get_stdin = default_get_stdin;
  stdstream_impl.get_stdout = default_get_stdout;
  stdstream_impl.get_stderr = default_get_stderr;
}

void pal_set_impl_stdstream(struct pal_impl_stdstream *impl) {
  assert(NULL != impl);
  assert(NULL != impl->get_stdin);
  assert(NULL != impl->get_stdout);
  assert(NULL != impl->get_stderr);

  stdstream_impl = *impl;
}

bool pal_has_impl_stdstream(void) {
  return stdstream_impl.get_stdin != default_get_stdin;
}

enum pal_error pal_get_stdin(pal_io_t *io) {
  return stdstream_impl.get_stdin(io);
}

enum pal_error pal_get_stdout(pal_io_t *io) {
  return stdstream_impl.get_stdout(io);
}

enum pal_error pal_get_stderr(pal_io_t *io) {
  return stdstream_impl.get_stderr(io);
}
