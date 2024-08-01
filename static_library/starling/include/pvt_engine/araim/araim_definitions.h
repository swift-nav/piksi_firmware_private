/*
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_ARAIM_DEFINITIONS_H
#define STARLING_ARAIM_DEFINITIONS_H

#include <pvt_common/containers/set.h>
#include <pvt_engine/eigen_types.h>
#include <swiftnav/signal.h>

namespace pvt_engine {
namespace araim {

// Constants determined by problem definition
static constexpr s32 cMaxSatelliteFailureEvents = NUM_SATS;
static constexpr s32 cMaxConstellationFailureEvents = CONSTELLATION_COUNT;
static constexpr s32 cMaxFailureEvents =
    cMaxSatelliteFailureEvents + cMaxConstellationFailureEvents;

// Constants governed by computational limits of monitoring platform
static constexpr s32 cMaxAllowableSimultaneousFailures = 3;
static constexpr s32 cMaxAllowableFaultModes = 60;

// A set to store indices associated with an observation vector
//  Ex. Set of observation indices associated with a given FailureEvent
using ObsIndexSubset = pvt_common::containers::Set<s32, cMaxFilterObservations>;

}  // namespace araim
}  // namespace pvt_engine

#endif  // STARLING_ARAIM_DEFINITIONS_H
