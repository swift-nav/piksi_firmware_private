/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_PVT_ENGINE_SBAS_H
#define LIBSWIFTNAV_PVT_ENGINE_SBAS_H

#include <pvt_engine/optional.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/sbas_raw_data.h>
#include <swiftnav/signal.h>

namespace pvt_engine {

// maximum number of satellites SBAS can transfer corrections to
constexpr u16 cMaxSbasAugmentedSatellites = 51;

struct SBASCorrections {
  gnss_signal_t sid;
  gps_time_t epoch_time;
  optional<double> iono_correction;
  optional<double> range_correction;
  optional<double> range_rate_correction;
};

using SBASRawData = sbas_raw_data_t;

};  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_SBAS_H
