/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Valeri Atamaniouk <valeri.atamaniouk@exafore.com>
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

#include <libswiftnav/common.h>

/**
 * Checks if NDB persistence is available.
 *
 * \retval true  NDB persistence is in use
 * \retval false NDB persistence is not available
 */
bool ndb_fs_is_real(void)
{
  return true;
}

/**
 * Removes file from persistent storage
 *
 * \param[in] name File name.
 *
 * \return 0
 */
int ndb_fs_remove(const char *name)
{
  sbp_fileio_remove(name);
  return 0;
}

/**
 * Block read from a persistent file.
 *
 * \param[in]  fn      File name.
 * \param[in]  offset  Data offset in file in bytes.
 * \param[out] buf     Destination data buffer.
 * \param[in]  size    Size of destination data buffer in bytes.
 *
 * \return Error code is <0 or number of bytes read.
 */
ssize_t ndb_fs_read(const char *fn, off_t offset, void *buf, size_t size)
{
  return sbp_fileio_read(fn, offset, buf, size);
}

/**
 * Block read from a persistent file.
 *
 * \param[in] fn      File name.
 * \param[in] offset  Data offset in file in bytes.
 * \param[in] buf     Source data buffer.
 * \param[in] size    Size of source data buffer in bytes.
 *
 * \return Error code is <0 or number of bytes written.
 */
ssize_t ndb_fs_write(const char *fn, off_t offset, const void *buf, size_t size)
{
  return sbp_fileio_write(fn, offset, buf, size);
}

/**
 * Creates empty file with a given size.
 *
 * The method creates a file of a given size. The file is filled with zeros.
 *
 * \param[in] name File name
 * \param[in] size File size in bytes
 *
 * \return Error code is <0 or number of bytes written.
 */
ssize_t ndb_fs_reserve(const char *name, size_t size)
{
  static const u8 zero[256];
  off_t offset = 0;
  ssize_t res = 0;
  while (size > 0) {
    size_t to_write = MIN(size, sizeof(zero));
    if (sbp_fileio_write(name, offset, zero, to_write) < 0) {
      res = -1;
      break;
    }
    size -= to_write;
    res += to_write;
  }

  return res;
}
