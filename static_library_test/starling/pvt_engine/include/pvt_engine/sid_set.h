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

#ifndef LIBSWIFTNAV_PVT_ENGINE_SID_SET_H
#define LIBSWIFTNAV_PVT_ENGINE_SID_SET_H

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdlib>

#include <swiftnav/signal.h>

#include <pvt_common/containers/set.h>
#include <pvt_common/containers/static_vector.h>
#include <pvt_engine/observation.h>
#include <pvt_engine/pvt_types.h>
#include <pvt_engine/sat_identifier.h>

namespace pvt_engine {

namespace ambiguities {

using SidVector =
    pvt_common::containers::StaticVector<gnss_signal_t, cMaxAmbiguities>;

using SidSet = pvt_common::containers::Set<gnss_signal_t, cMaxAmbiguities>;

s32 num_unique_codes(const SidSet &sids);

bool select_constellation(const gnss_signal_t &sid,
                          const constellation_t &constellation);

}  // namespace ambiguities

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_SID_SET_H
