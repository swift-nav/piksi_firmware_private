/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Michele Bavaro <michele@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#ifndef TRACK_GAL_E1_H
#define TRACK_GAL_E1_H

#include <swiftnav/common.h>

#ifdef __cplusplus
extern "C" {
#endif

void track_gal_e1_register(void);

void gal_e7_to_e1_handover(u32 sample_count,
                           u16 sat,
                           double code_phase,
                           double doppler_hz,
                           float cn0_init);

#ifdef __cplusplus
}
#endif

#endif /* TRACK_GAL_E1_H */
