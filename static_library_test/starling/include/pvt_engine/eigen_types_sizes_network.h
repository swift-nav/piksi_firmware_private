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

#ifndef LIBSWIFTNAV_PVT_ENGINE_EIGEN_TYPES_SIZES_NETWORK_H
#define LIBSWIFTNAV_PVT_ENGINE_EIGEN_TYPES_SIZES_NETWORK_H

#include <pvt_engine/eigen_types_sizes_common.h>

namespace pvt_engine {

constexpr s32 cMaxInputTrackingChannels = MAX_CHANNELS;
constexpr s32 cMaxFilterTrackingChannels = STARLING_MAX_CHANNEL_COUNT;

/** Max number of iono states = one state for each satellite
 *  note that the number of iono states can never be more than
 *  half of the number of tracking channels because each state
 *  will require two channels (dual frequency is required for
 *  iono estimation). The +1 is required due to integer division
 *  potentially discarding a fractional part.
 *
 */
constexpr s32 cMaxIonoStatesFromDualFreqPhase =
    ((cMaxFilterTrackingChannels + 1) / 2);

// On the network, we retain iono states so need more space for them
constexpr s32 cMaxIonoStatesFromAllObs = cMaxFilterTrackingChannels;

/** Max number of ambiguities = one ambiguity for each signal
 *
 */
constexpr s32 cMaxAmbiguities = cMaxFilterTrackingChannels;

/** Max number of observations = pseudorange, carrier_phase xor computed
 * Doppler and baseline magnitude
 */
constexpr s32 cMaxFilterObservations =
    cMaxFilterTrackingChannels * cMaxNumberProcessedObservationTypes + 1;

/** Max number of observations = pseudorange, carrier_phase xor computed
 * Doppler and baseline magnitude
 */
constexpr s32 cMaxInputObservations =
    cMaxInputTrackingChannels * cMaxNumberProcessedObservationTypes + 1;

constexpr s32 cMaxStateDim =
    cMaxBaselineStates + cMaxSingleClockPerConstellationStates +
    cMaxPhaseWindupStates + cMaxHardwareBiasStates + cMaxAmbiguities +
    cMaxIonoStatesFromAllObs + cMaxTroposphereStates;

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_EIGEN_TYPES_SIZES_NETWORK_H
