/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Pasi Miettinen <pasi.miettinen@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_CNAV_MSG_STORAGE_H
#define LIBSWIFTNAV_CNAV_MSG_STORAGE_H

#include <libswiftnav/cnav_msg.h>
#include <libswiftnav/signal.h>

typedef enum {
  CNAV_MSG_TYPE_IDX_10,
  CNAV_MSG_TYPE_IDX_11,
  CNAV_MSG_TYPE_IDX_30,
  CNAV_MSG_TYPE_IDX_32,
  CNAV_MSG_TYPE_IDX_33,
  CNAV_MSG_TYPE_NUM
} cnav_msg_idx_t;

typedef struct {
  bool msg_set;
  cnav_msg_t msg;
} cnav_msg_storage_t;

void cnav_msg_put(const cnav_msg_t *msg);
bool cnav_msg_get(gnss_signal_t sid, cnav_msg_type_t type, cnav_msg_t *msg);

#endif /* LIBSWIFTNAV_CNAV_MSG_STORAGE_H */
