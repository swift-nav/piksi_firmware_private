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

#ifndef LIBSWIFTNAV_PVT_ENGINE_AMBIGUITY_MAP_H
#define LIBSWIFTNAV_PVT_ENGINE_AMBIGUITY_MAP_H

#include <pvt_common/containers/map.h>
#include <pvt_engine/observation_handler.h>
#include <pvt_engine/pvt_types.h>
#include <swiftnav/signal.h>

namespace pvt_engine {

namespace ambiguities {

using IndexSet = pvt_common::containers::Set<s32, cMaxAmbiguities>;

using AmbiguityMap =
    pvt_common::containers::Map<gnss_signal_t, s32, cMaxAmbiguities>;
using AmbiguityIndexElem =
    pvt_common::containers::MapElement<gnss_signal_t, s32>;
using AmbiguityIndexMap =
    pvt_common::containers::Map<gnss_signal_t, s32, cMaxAmbiguities>;

inline bool match_frequency(const ambiguities::AmbiguityIndexElem &elem,
                            FREQUENCY freq) {
  ObservationIdentifier obs_id(elem.key, CARRIER_PHASE);
  return obs_filters::match_frequency(obs_id, freq);
}
inline bool match_code(const ambiguities::AmbiguityIndexElem &elem,
                       code_t code) {
  ObservationIdentifier obs_id(elem.key, CARRIER_PHASE);
  return obs_filters::match_code(obs_id, code);
}

}  // namespace ambiguities

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_AMBIGUITY_MAP_H
