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

#ifndef STARLING_PVT_ENGINE_SSR_UTILS_H
#define STARLING_PVT_ENGINE_SSR_UTILS_H

#include <swiftnav/gnss_time.h>

#include <pvt_engine/eigen_types.h>
#include <pvt_engine/sat_identifier.h>
#include <pvt_engine/satellite_states.h>

namespace pvt_engine {
namespace ssr {

struct TimeOfTransmissionInfo {
  Eigen::Vector3d sat_pos;
  gps_time_t time_of_transmission;
  double sat_range;
  EphemerisKey eph_key;

  TimeOfTransmissionInfo()
      : sat_pos(), time_of_transmission(), sat_range(), eph_key(){};
};

optional<TimeOfTransmissionInfo> get_time_of_transmission_info(
    const SatelliteStatesHandler &satellite_states_handler,
    const CorrectionKey &correction_key, const SatIdentifier &sat,
    const Eigen::Vector3d &reception_location,
    const gps_time_t &time_of_reception);

bool sid_available_and_visible(
    const SatelliteStatesHandler &satellite_states_handler,
    const CorrectionKey &correction_key, const gnss_signal_t &sid,
    const Eigen::Vector3d &reception_location,
    const gps_time_t &time_of_reception, const double elevation_mask_rad = 0.0);

bool sid_visible(const SatelliteStatesHandler &satellite_states_handler,
                 const gnss_signal_t &sid,
                 const Eigen::Vector3d &reception_location,
                 const gps_time_t &time_of_reception);

SatIdSet get_visible_sats(
    const SatelliteStatesHandler &satellite_states_handler,
    const Eigen::Vector3d &reception_location, const gps_time_t &gps_time,
    const obs_filters::ConstellationSet &constellations = {});

}  // namespace ssr
}  // namespace pvt_engine

#endif  // STARLING_PVT_ENGINE_SSR_UTILS_H
