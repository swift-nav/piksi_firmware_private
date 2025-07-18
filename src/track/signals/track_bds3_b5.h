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
#ifndef TRACK_BDS3_B5_H
#define TRACK_BDS3_B5_H

#include <swiftnav/common.h>

#ifdef __cplusplus
extern "C" {
#endif

void track_bds3_b5_register(void);

void bds_b1_to_b5_handover(u32 sample_count,
                           u16 sat,
                           double code_phase,
                           double carrier_freq,
                           float cn0_init);

#ifdef __cplusplus
}
#endif

#endif /* TRACK_BDS3_B5_H */
