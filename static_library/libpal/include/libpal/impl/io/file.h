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

#ifndef LIBPAL_IMPL_IO_FILE_H
#define LIBPAL_IMPL_IO_FILE_H

#include <libpal/io/io.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * PAL File IO Implementation
 *
 * This file defines the interface a PAL implementation must use to install its
 * own file handling ability in to the libpal API
 *
 * The function pointer names and signatures in this file match those in
 * libpal/io/file.h. The PAL implementation must provide a version of these
 * functions which meet the requirements stated in the documentation contained
 * in that file.
 */

/**
 * Open a file
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_file_open() (see libpal/io/file.h)
 *
 */
typedef enum pal_error (*pal_file_open_t)(const char *path,
                                          enum pal_access_mode mode,
                                          pal_io_t *io);

/**
 * PAL File IO implementation definition
 */
struct pal_impl_file {
  // Implementation File IO open routine
  pal_file_open_t open;
};

/**
 * Install PAL file IO implementation in to API
 *
 * Call this function during pal_impl_init to register the implementation's file
 * IO module with the libpal API
 *
 * @param impl File IO implementation definition
 */
void pal_set_impl_file(struct pal_impl_file *impl);

#ifdef __cplusplus
}
#endif

#endif
