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

#ifndef STARLING_INTEGRITY_SUPPORT_MESSAGE_H
#define STARLING_INTEGRITY_SUPPORT_MESSAGE_H

#include <pvt_common/containers/map.h>
#include <pvt_engine/sat_identifier.h>
#include <swiftnav/signal.h>

namespace pvt_engine {

using SatDoubleMap =
    pvt_common::containers::Map<SatIdentifier, double, NUM_SATS>;
using ConstellationDoubleMap =
    pvt_common::containers::Map<constellation_t, double, CONSTELLATION_COUNT>;

struct IntegritySupportMessageConfiguration {
  // Default values to assign for ISM parameters
  optional<double> default_satellite_fault_probability;
  optional<double> default_constellation_fault_probability;
  optional<double> default_sigma_ura;
  optional<double> default_sigma_ure;
  optional<double> default_nominal_bias;

  IntegritySupportMessageConfiguration();

  bool valid_values() const;
};

struct IntegritySupportMessage {
  // Standard deviation of user range accuracy (URA) [m]
  SatDoubleMap sigma_ura;
  // Standard deviation of user range error (URE) [m]
  SatDoubleMap sigma_ure;
  // Maximum nominal code bias [m]
  SatDoubleMap nominal_bias;
  // Prior probability of a satellite fault
  SatDoubleMap P_sat;
  // Prior probability of a constellation fault
  ConstellationDoubleMap P_constellation;

  IntegritySupportMessage(const IntegritySupportMessageConfiguration &config);

  bool valid_values() const;
};

}  // namespace pvt_engine

#endif  // STARLING_INTEGRITY_SUPPORT_MESSAGE_H
