/*
 * Copyright (C) 2020 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_PVT_ENGINE_PVT_MAX_SATS_H
#define STARLING_PVT_ENGINE_PVT_MAX_SATS_H
#include "pvt_common/containers/map.h"
#include "pvt_types.h"
#include "solution_frequency.h"

pvt_common::containers::Map<observation_rate_t, s32, NUM_OBSERVATION_RATES>
get_frequency_rates_to_max_sats_time_matched();

pvt_common::containers::Map<observation_rate_t, s32, NUM_OBSERVATION_RATES>
get_frequency_rates_to_max_sats_low_latency();

/* Get the max sats for the given requested frequency. */
bool get_max_sats_from_requested_freq_and_solution_mode(
    double requested_frequency_hz, pvt_engine::PROCESSING_MODE mode,
    s32 *max_sats);

/* Get the max sats for the given observation rate. */
s32 get_max_sats_from_freq_enum_and_solution_mode(
    observation_rate_t rate, pvt_engine::PROCESSING_MODE mode);

#endif  // STARLING_PVT_ENGINE_PVT_MAX_SATS_H
