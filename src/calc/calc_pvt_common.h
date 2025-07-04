/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Michele Bavaro <michele@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_CALC_PVT_COMMON_H
#define SWIFTNAV_CALC_PVT_COMMON_H

#include <calc/starling_integration.h>

#ifdef __cplusplus
extern "C" {
#endif

void send_observations(const obs_array_t *obs_array, u32 msg_obs_max_size);
bool get_max_sats(double soln_freq_hz,
                  pvt_driver_solution_mode_t soln_mode,
                  s32 *max_sats);

#ifdef __cplusplus
}
#endif

#endif /* COMMON_CALC_PVT_H */
