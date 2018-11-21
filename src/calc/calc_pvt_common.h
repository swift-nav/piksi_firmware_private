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

#include <starling/starling.h>

#ifdef __cplusplus
extern "C" {
#endif

void send_observations(const obs_array_t *obs_array, u32 msg_obs_max_size);

#ifdef __cplusplus
}
#endif

#endif /* COMMON_CALC_PVT_H */
