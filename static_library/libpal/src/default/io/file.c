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

#include <assert.h>
#include <stddef.h>

#include <libpal/error.h>
#include <libpal/impl/io/file.h>
#include <libpal/io/file.h>

#include <pal_private.h>

#include "not_implemented.h"

/**
 * Default file open routine
 * @param path Unused
 * @param mode Unused
 * @param io Unused
 * @return PAL_INVALID
 */
static enum pal_error default_file_open(const char *name,
                                        enum pal_access_mode mode,
                                        pal_io_t *io) {
  NOT_IMPLEMENTED();
  (void)name;
  (void)mode;
  (void)io;
  return PAL_INVALID;
}

static struct pal_impl_file file_impl = {
    .open = default_file_open,
};

void pal_reset_impl_file(void) { file_impl.open = default_file_open; }

void pal_set_impl_file(struct pal_impl_file *impl) {
  assert(NULL != impl);
  assert(NULL != impl->open);

  file_impl = *impl;
}

bool pal_has_impl_file() { return file_impl.open != default_file_open; }

enum pal_error pal_file_open(const char *name, enum pal_access_mode mode,
                             pal_io_t *io) {
  return file_impl.open(name, mode, io);
}
