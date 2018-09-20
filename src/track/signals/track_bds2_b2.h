/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Michele Bavaro <michele@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#ifndef TRACK_BDS2_B2_H
#define TRACK_BDS2_B2_H

#include <swiftnav/common.h>

#ifdef __cplusplus
extern "C" {
#endif

void track_bds2_b2_register(void);

void bds_b11_to_b2_handover(u32 sample_count,
                            u16 sat,
                            double code_phase,
                            double carrier_freq,
                            float cn0_init);

#ifdef __cplusplus
}
#endif

#endif /* TRACK_BDS2_B2_H */
