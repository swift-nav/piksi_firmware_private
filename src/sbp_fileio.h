/*
 * Copyright (C) 2012-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_SBP_FILEIO_H
#define SWIFTNAV_SBP_FILEIO_H

#include <libswiftnav/common.h>

void sbp_fileio_remove(const char *filename);
ssize_t sbp_fileio_write(const char *filename, off_t offset, const u8 *buf, size_t size);
ssize_t sbp_fileio_read(const char *filename, off_t offset, u8 *buf, size_t size);

#endif
