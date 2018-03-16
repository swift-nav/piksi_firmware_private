/*
 * Copyright (C) 2011-2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_NAV_BIT_FIFO_H
#define SWIFTNAV_NAV_BIT_FIFO_H

#include <libswiftnav/common.h>
#include <libswiftnav/memcpy_s.h>

#define NAV_BIT_FIFO_SIZE 64 /* Size of nav bit FIFO. Must be a power of 2 */

#define NAV_BIT_FIFO_INDEX_DIFF(write_index, read_index) \
  ((u8)((write_index) - (read_index)))

typedef s8 nav_bit_t;

typedef struct {
  u8 read_index;
  u8 write_index;
  nav_bit_t elements[NAV_BIT_FIFO_SIZE];
} nav_bit_fifo_t;

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void nav_bit_fifo_init(nav_bit_fifo_t *fifo);
bool nav_bit_fifo_full(nav_bit_fifo_t *fifo);
bool nav_bit_fifo_write(nav_bit_fifo_t *fifo, const nav_bit_t *element);
bool nav_bit_fifo_read(nav_bit_fifo_t *fifo, nav_bit_t *element);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* SWIFTNAV_NAV_BIT_FIFO_H */
