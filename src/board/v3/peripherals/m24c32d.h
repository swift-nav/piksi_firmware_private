/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_M24C32D_H
#define SWIFTNAV_M24C32D_H

#include <stdlib.h>
#include <libswiftnav/common.h>

bool m24c32d_read(u16 addr, u8 *data, size_t length);
bool m24c32d_write(u16 addr, const u8 *data, size_t length);

bool m24c32d_id_read(u8 addr, u8 *data, size_t length);
bool m24c32d_id_write(u8 addr, const u8 *data, size_t length);
bool m24c32d_id_lock(void);

#endif /* SWIFTNAV_M24C32D_H */
