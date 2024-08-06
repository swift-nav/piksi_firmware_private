/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_PVT_ENGINE_INTERNAL_SNAPSHOT_VALIDATE_H
#define LIBSWIFTNAV_PVT_ENGINE_INTERNAL_SNAPSHOT_VALIDATE_H

#include <pvt_engine/ambiguity_types.h>
#include <starling/build/config.h>

namespace pvt_engine {

namespace ambiguities {

using SignalCounterMap =
    pvt_common::containers::Map<SatIdentifier, u8, cNumSat>;
using SignalCounterVector = pvt_common::containers::StaticVector<u8, cNumSat>;

u8 count_multi_freq_dd_ambs(
    const ambiguities::FloatAmbiguityStates &float_ambiguities);

bool are_there_enough_multi_freqs_carrier_phase(
    const ambiguities::FloatAmbiguityStates &float_ambiguities,
    const u16 &min_dual_frequency_double_differences);
}  // namespace ambiguities

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_INTERNAL_SNAPSHOT_VALIDATE_H
