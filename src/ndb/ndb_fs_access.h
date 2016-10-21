/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Gareth McMullin <gareth@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#ifndef NDB_FS_ACCESS_H
#define NDB_FS_ACCESS_H

#include <stdlib.h>
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

bool ndb_fs_is_real(void);
int ndb_fs_remove(const char *name);
ssize_t ndb_fs_read(const char *fn, off_t offset, void *buf, size_t size);
ssize_t ndb_fs_write(const char *fn, off_t offset, const void *buf, size_t size);
ssize_t ndb_fs_reserve(const char *name, size_t size);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif

