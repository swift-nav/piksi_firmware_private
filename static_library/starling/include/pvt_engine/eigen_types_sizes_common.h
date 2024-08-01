/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_PVT_ENGINE_EIGEN_TYPES_SIZES_COMMON_H
#define LIBSWIFTNAV_PVT_ENGINE_EIGEN_TYPES_SIZES_COMMON_H

#include <swiftnav/signal.h>

namespace pvt_engine {

constexpr s32 cMaxSingleClockStates = 1;
constexpr s32 cMaxSingleClockPerConstellationStates =
    static_cast<s32>(CONSTELLATION_COUNT);
constexpr s32 cMaxDecoupledClockStates = 16;

constexpr s32 cMaxPhaseWindupStates = 1;
constexpr s32 cMaxHardwareBiasStates = 8;

// Define the number of spatial dimensions (x, y, z).
constexpr s32 NUM_DIMENSIONS = 3;
constexpr s32 NUM_G_COLS = NUM_DIMENSIONS + 1;

/** Max number of baseline states = position, velocity, accleration for x, y, z
 *
 */
constexpr s32 cMaxBaselineStates = NUM_DIMENSIONS * 3;

constexpr s32 cMaxTroposphereStates = 1;

/** Max number of integer solution sets
 *
 */
constexpr s32 cMaxIntegerAmbiguitySets = 3;

constexpr s32 cMaxNumberProcessedObservationTypes = 2;

// TODO(https://github.com/swift-nav/firmware_team_planning/issues/334)
constexpr s32 cMaxObsBufferSize = 20;

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_EIGEN_TYPES_SIZES_COMMON_H
