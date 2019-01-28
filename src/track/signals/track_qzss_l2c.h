/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#ifndef TRACK_QZSS_L2C_H
#define TRACK_QZSS_L2C_H

#include <swiftnav/common.h>

#ifdef __cplusplus
extern "C" {
#endif

void track_qzss_l2c_register(void);

void qzss_l1ca_to_l2c_handover(u32 sample_count,
                               u16 sat,
                               double code_phase,
                               double doppler_hz,
                               float cn0_init,
                               s32 TOW_ms);

#ifdef __cplusplus
}
#endif

#endif /* TRACK_QZSS_L2C_H */
