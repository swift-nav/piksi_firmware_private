/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_FIFO_H
#define SWIFTNAV_FIFO_H

#include <libswiftnav/common.h>

typedef enum { FIFO_MODE_STANDARD, FIFO_MODE_RECORD } fifo_mode_t;

typedef u32 fifo_size_t;

typedef struct {
  fifo_size_t read_index;
  fifo_size_t write_index;
  fifo_size_t buffer_size;
  fifo_mode_t mode;
  u8 *buffer;
} fifo_t;

void fifo_init(fifo_t *fifo,
               fifo_mode_t mode,
               u8 *buffer,
               fifo_size_t buffer_size);

fifo_size_t fifo_length(fifo_t *fifo);
fifo_size_t fifo_space(fifo_t *fifo);

fifo_size_t fifo_read(fifo_t *fifo, u8 *buffer, fifo_size_t length);
fifo_size_t fifo_peek(fifo_t *fifo, u8 *buffer, fifo_size_t length);
fifo_size_t fifo_remove(fifo_t *fifo, fifo_size_t length);

fifo_size_t fifo_write(fifo_t *fifo, const u8 *buffer, fifo_size_t length);
fifo_size_t fifo_poke(fifo_t *fifo, const u8 *buffer, fifo_size_t length);
fifo_size_t fifo_add(fifo_t *fifo, fifo_size_t length);

#endif /* SWIFTNAV_FIFO_H */
