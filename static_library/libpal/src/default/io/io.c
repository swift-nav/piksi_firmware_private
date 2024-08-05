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

#include <libpal/error.h>
#include <libpal/impl/io/io.h>
#include <libpal/io/io.h>
#include <pal_private.h>
#include <stddef.h>

#include "not_implemented.h"

/**
 * Default IO read routine
 * @param io Unused
 * @param buf Unused
 * @param n Unused
 * @param timeout_us Unused
 * @return PAL_INVALID
 */
static enum pal_error default_io_read(pal_io_t io, uint8_t *buf, size_t n,
                                      size_t *nread, uint64_t timeout_us) {
  (void)io;
  (void)buf;
  (void)n;
  (void)nread;
  (void)timeout_us;
  NOT_IMPLEMENTED();
  return PAL_INVALID;
}

/**
 * Default IO write routine
 * @param io Unused
 * @param buf Unused
 * @param n Unused
 * @param timeout_us Unused
 * @return PAL_INVALID;
 */
static enum pal_error default_io_write(pal_io_t io, const uint8_t *buf,
                                       size_t n, size_t *nwritten,
                                       uint64_t timeout_us) {
  (void)io;
  (void)buf;
  (void)n;
  (void)nwritten;
  (void)timeout_us;
  NOT_IMPLEMENTED();
  return PAL_INVALID;
}

/**
 * Default IO close routine
 * @param io Unused
 * @return PAL_INVALID;
 */
static enum pal_error default_io_close(pal_io_t io) {
  (void)io;
  NOT_IMPLEMENTED();
  return PAL_INVALID;
}

/**
 * Default IO flush routine
 * @param io Unused
 * @return PAL_INVALID;
 */
static enum pal_error default_io_flush(pal_io_t io) {
  (void)io;
  NOT_IMPLEMENTED();
  return PAL_INVALID;
}

/**
 * Default IO watch routine
 * @param readset Unused
 * @param nread Unused
 * @param writeset Unused
 * @param nwrite Unused
 * @return PAL_INVALID;
 */
static enum pal_error default_io_watch(pal_io_t *readset, size_t *nread,
                                       pal_io_t *writeset, size_t *nwrite) {
  (void)readset;
  (void)nread;
  (void)writeset;
  (void)nwrite;
  NOT_IMPLEMENTED();
  return PAL_INVALID;
}

/**
 * Default IO test EOF routine
 * @param io Unused
 * @param eof Unused
 * @return PAL_INVALID
 */
static enum pal_error default_io_eof(pal_io_t io, bool *eof) {
  (void)io;
  (void)eof;
  NOT_IMPLEMENTED();
  return PAL_INVALID;
}

/**
 * Default get IO type routine
 * @param io Unused
 * @param type Unused
 * @return PAL_INVALID
 */
static enum pal_error default_io_get_type(pal_io_t io, enum pal_io_type *type) {
  (void)io;
  (void)type;
  NOT_IMPLEMENTED();
  return PAL_INVALID;
}

static struct pal_impl_io io_impl = {
    .read = default_io_read,
    .write = default_io_write,
    .close = default_io_close,
    .flush = default_io_flush,
    .watch = default_io_watch,
    .eof = default_io_eof,
    .get_type = default_io_get_type,
};

void pal_reset_impl_io(void) {
  io_impl.read = default_io_read;
  io_impl.write = default_io_write;
  io_impl.close = default_io_close;
  io_impl.flush = default_io_flush;
  io_impl.watch = default_io_watch;
  io_impl.eof = default_io_eof;
  io_impl.get_type = default_io_get_type;
}
void pal_set_impl_io(struct pal_impl_io *impl) {
  assert(NULL != impl);
  assert(NULL != impl->read);
  assert(NULL != impl->write);
  assert(NULL != impl->close);
  assert(NULL != impl->flush);
  assert(NULL != impl->watch);
  assert(NULL != impl->eof);

  io_impl = *impl;
}

bool pal_has_impl_io() { return io_impl.read != default_io_read; }

enum pal_error pal_io_read(pal_io_t io, uint8_t *buf, size_t n, size_t *nread,
                           uint64_t timeout_us) {
  return io_impl.read(io, buf, n, nread, timeout_us);
}

enum pal_error pal_io_write(pal_io_t io, const uint8_t *buf, size_t n,
                            size_t *nwritten, uint64_t timeout_us) {
  return io_impl.write(io, buf, n, nwritten, timeout_us);
}

enum pal_error pal_io_close(pal_io_t *io) {
  if (!io) {
    return PAL_INVALID;
  }
  enum pal_error ret = io_impl.close(*io);
  if (ret == PAL_SUCCESS) {
    *io = NULL;
  }
  return ret;
}

enum pal_error pal_io_flush(pal_io_t io) { return io_impl.flush(io); }

enum pal_error pal_io_watch(pal_io_t *readset, size_t *nread,
                            pal_io_t *writeset, size_t *nwrite) {
  return io_impl.watch(readset, nread, writeset, nwrite);
}

enum pal_error pal_io_eof(pal_io_t io, bool *eof) {
  return io_impl.eof(io, eof);
}

enum pal_error pal_io_get_type(pal_io_t io, enum pal_io_type *type) {
  return io_impl.get_type(io, type);
}
