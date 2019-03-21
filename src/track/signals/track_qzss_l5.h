/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Michele Bavaro <michele@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#ifndef SWIFTNAV_TRACK_QZSS_L5_H
#define SWIFTNAV_TRACK_QZSS_L5_H

#include <swiftnav/common.h>

#ifdef __cplusplus
extern "C" {
#endif

void track_qzss_l5_register(void);

void do_qzss_l1_to_l5_handover(u32 sample_count,
                               u16 sat,
                               double code_phase,
                               double carrier_freq,
                               float cn0_init);

#ifdef __cplusplus
}
#endif

#endif /* SWIFTNAV_TRACK_QZSS_L5_H */
