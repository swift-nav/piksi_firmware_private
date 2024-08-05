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

#ifndef LIBSWIFTNAV_BASE_POSITION_HANDLER_H
#define LIBSWIFTNAV_BASE_POSITION_HANDLER_H

#include <pvt_engine/configuration.h>
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/optional.h>
#include <pvt_engine/pvt_types.h>

namespace pvt_engine {

struct BasePosition {
  BasePosition();
  bool operator==(const BasePosition &rhs) const;

  Eigen::Vector3d reference_station_position_;
  REFERENCE_STATION_POSITION_SOURCE reference_station_position_source_;
};

class BasePositionHandler {
 public:
  enum class ResetFilter : bool { ResetNeeded, NoResetNeeded };

  explicit BasePositionHandler(const BasePositionConfiguration &config);
  bool operator==(const BasePositionHandler &rhs) const;

  void initialize();
  void update_config(const BasePositionConfiguration &config);

  ResetFilter set_updated_reference_position(
      const gps_time_t &obs_time,
      const Eigen::Vector3d &spp_reference_station_position);

  optional<BasePosition> get_current_reference_position() const;

  bool is_reset_needed(const gps_time_t &obs_time,
                       const BasePosition &reference_station_position) const;

  void set_known_ref_pos(const Eigen::Vector3d &known_reference_position);

 private:
  bool has_valid_surveyed_position(
      const gps_time_t &obs_time,
      const Eigen::Vector3d &spp_reference_station_position) const;

  bool is_valid_surveyed_position(
      const gps_time_t &obs_time,
      const Eigen::Vector3d &spp_reference_station_position,
      const Eigen::Vector3d &reference_station_position) const;

  bool has_base_position_moved_too_far(
      const gps_time_t &obs_time,
      const BasePosition &reference_station_position) const;

  BasePositionConfiguration config_;
  optional<BasePosition> current_base_pos_;
  optional<Eigen::Vector3d> known_reference_position_;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_BASE_POSITION_HANDLER_H
