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

#ifndef LIBPAL_IO_FILE_H
#define LIBPAL_IO_FILE_H

#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>

#include <libpal/io/io.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Check Global PAL File Implementation
 * @return true if global file implementation has be set, otherfile
 * false
 */
bool pal_has_impl_file(void);

/**
 * Linux PAL file open routine.
 *
 * Opens a file and make ready for future access according to the given mode. On
 * success this function will provide a file descriptor in the io_out parameter
 * which can be used for future read and/or write operations. When the file is
 * no longer needed it should be closed by passing the file descriptor back to
 * pal_file_close(). The new IO handle will only be valid if this function
 * returns PAL_SUCCESS
 *
 * When opened the IO descriptor must be set to reference the first byte of the
 * file, that is to say the first read or write operation on the descriptor will
 * operate on the first byte of the file. When opened in write only mode
 * (PAL_WRONLY) the file will be truncated, when opened in read/write mode
 * (PAL_RDWR) it will not be truncated but the first write operation will still
 * occur at the first byte of the file. When opened with write access
 * (PAL_WRONLY or PAL_RDWR) and the file does not already exist it must be
 * created.
 *
 * @param path File name/path
 * @param mode Access mode, PAL_RDONLY, PAL_WRONLY, PAL_RDWR
 * @param io PAL file descriptor to be used for later access
 * @return PAL Exit Code
 */
enum pal_error pal_file_open(const char *name, enum pal_access_mode mode,
                             pal_io_t *io);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // LIBPAL_IO_FILE_H
