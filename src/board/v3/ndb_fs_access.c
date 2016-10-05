/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Roman Gezikov <rgezikov@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <stdio.h>
#include <stdbool.h>

#include "ndb/ndb_fs_access.h"
#include "sbp_fileio.h"

bool ndb_fs_is_real()
{
  return true;
}

int ndb_fs_remove(const char *name)
{
  sbp_fileio_remove(name);
  return 0;
}

ssize_t ndb_fs_read(const char *fn, off_t offset, void *buf, size_t size)
{
  return sbp_fileio_read(fn, offset, buf, size);
}

ssize_t ndb_fs_write(const char *fn, off_t offset, const void *buf, size_t size)
{
  return sbp_fileio_write(fn, offset, buf, size);
}

int ndb_fs_reserve(const char *name, size_t size)
{
  (void)name;
  return size;
}
