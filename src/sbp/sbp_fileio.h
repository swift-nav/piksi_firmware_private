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

#include <stdio.h>
#include <swiftnav/common.h>

void sbp_fileio_remove(const char *fn);

/**
 * @brief   Write to non-volatile memory
 *
 * @note    Function is neither reentrant nor thread-safe
 *
 * @param[in] filename      Name of the file to write to
 * @param[in] offset        Offset into the file [bytes]
 * @param[in] buf           Buffer to write from
 * @param[in] size          Size of the buffer
 *
 * @return                  Number of bytes written
 * @retval -1               Error
 */
ssize_t sbp_fileio_write(const char *filename,
                         u32 offset,
                         const u8 *buf,
                         size_t size);

/**
 * @brief   Read from non-volatile memory
 *
 * @note    Function is neither reentrant nor thread-safe
 *
 * @param[in] filename      Name of the file to read from
 * @param[in] offset        Offset into the file [bytes]
 * @param[in] buf           Buffer to write into
 * @param[in] size          Size of the buffer
 *
 * @return                  Number of bytes read
 * @retval -1               Error
 */
ssize_t sbp_fileio_read(const char *filename, u32 offset, u8 *buf, size_t size);

#endif
