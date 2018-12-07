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
#ifndef SWIFTNAV_TRACK_GPS_L1CA_H
#define SWIFTNAV_TRACK_GPS_L1CA_H

#include "track/track_common.h"

#ifdef __cplusplus
extern "C" {
#endif

void track_gps_l1ca_register(void);

void tracker_gps_l1ca_update_shared(tracker_t *tracker, u32 cflags);

#ifdef __cplusplus
}
#endif

#endif
