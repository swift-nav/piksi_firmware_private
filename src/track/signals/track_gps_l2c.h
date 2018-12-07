/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#ifndef SWIFTNAV_TRACK_GPS_L2C_H
#define SWIFTNAV_TRACK_GPS_L2C_H

#include <swiftnav/common.h>
#include "track/track_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* not weak as it is used in L2C builds only */
void track_gps_l2c_register(void);

void tracker_gps_l2c_update_shared(tracker_t *tracker, u32 cflags);

void do_l1ca_to_l2c_handover(u32 sample_count,
                             u16 sat,
                             double code_phase,
                             double carrier_freq,
                             float cn0_init,
                             s32 TOW_ms);

#ifdef __cplusplus
}
#endif

#endif /* SWIFTNAV_TRACK_GPS_L2C_H */
