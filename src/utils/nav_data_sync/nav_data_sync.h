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

#ifndef SWIFTNAV_NAV_DATA_SYNC_H
#define SWIFTNAV_NAV_DATA_SYNC_H

#include <stdbool.h>
#include "main/main.h"
#include "nav_bit_fifo/nav_bit_fifo.h"
#include "shm/shm.h"

typedef enum {
  SYNC_NONE = 0,             /**< Nothing to sync */
  SYNC_POL = (1 << 0),       /**< Sync data polarity */
  SYNC_TOW = (1 << 1),       /**< Sync TOW */
  SYNC_EPH = (1 << 2),       /**< Sync ephemeris parameters */
  SYNC_GLO_STRING = (1 << 3) /**< Sync GLO string */
} decode_sync_flags_t;

typedef struct {
  s32 TOW_ms;
  s32 TOW_residual_ns; /**< Residual to TOW_ms [ns] */
  s8 bit_polarity;
  u16 glo_orbit_slot;
  u8 read_index;
  health_t health;
  bool valid;
  decode_sync_flags_t sync_flags;
} nav_data_sync_t;

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void nav_data_sync_init(nav_data_sync_t *sync);
bool nav_data_sync_set(nav_data_sync_t *to_tracker,
                       const nav_data_sync_t *from_decoder);
bool nav_data_sync_get(nav_data_sync_t *to_tracker,
                       nav_data_sync_t *from_decoder);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* SWIFTNAV_NAV_DATA_SYNC_H */
