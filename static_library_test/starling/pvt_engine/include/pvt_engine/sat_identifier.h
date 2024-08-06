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

#ifndef LIBSWIFTNAV_PVT_ENGINE_SAT_IDENTIFIER_H
#define LIBSWIFTNAV_PVT_ENGINE_SAT_IDENTIFIER_H

#include <swiftnav/signal.h>

#include <pvt_common/containers/map.h>
#include <pvt_common/containers/set.h>
#include <pvt_engine/observation.h>
#include <pvt_engine/pvt_types.h>
#include <starling/build/config.h>

namespace pvt_engine {

struct SatIdentifier {
  SatIdentifier();

  SatIdentifier(const u16 sat, const constellation_t constellation);

  explicit SatIdentifier(const gnss_signal_t &sid);

  explicit SatIdentifier(const ObservationIdentifier &obs_id);

  explicit SatIdentifier(const std::string &state_label);

  bool operator==(const SatIdentifier &other) const;

  bool operator!=(const SatIdentifier &other) const;

  bool operator<(const SatIdentifier &other) const;

  std::string to_string() const;

  bool is_valid() const;

  gnss_signal_t get_l1_sid() const;

  u16 sat_;
  constellation_t constellation_;
};

using SatIdSet = pvt_common::containers::Set<SatIdentifier, cNumSat>;

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_SAT_IDENTIFIER_H
