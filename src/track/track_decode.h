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

#ifndef SWIFTNAV_TRACK_DECODE_H
#define SWIFTNAV_TRACK_DECODE_H

#include "tracker.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Decoder interface */
bool tracker_nav_bit_received(u8 id, nav_bit_t *nav_bit);
bool tracker_health_sync(u8 id, u8 health);
void tracker_data_sync_init(nav_data_sync_t *data_sync);
void tracker_data_sync(u8 id, nav_data_sync_t *from_decoder);

#ifdef __cplusplus
}
#endif

#endif /* SWIFTNAV_TRACK_DECODE_H */
