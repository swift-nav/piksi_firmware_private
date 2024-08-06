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

#ifndef GROUND_TRUTH_EPOCH_H
#define GROUND_TRUTH_EPOCH_H

#include <swiftnav/gnss_time.h>
#include <swiftnav/logging.h>

#include <pvt_common/eigen_custom.h>
#include <pvt_common/optional.h>

#include "truth_file_and_format.h"

namespace ground_truth {

struct TruthEpoch {
  gps_time_t gps_time = GPS_TIME_UNKNOWN;
  optional<s32> pos_mode;
  optional<Eigen::Vector3d> baseline = {};
  optional<Eigen::Vector3d> reference_station_ecef = {};
  optional<Eigen::Vector3d> position_ecef = {};
  optional<Eigen::Vector3d> average_velocity_ecef = {};
  optional<Eigen::Vector3d> instantaneous_velocity_ecef = {};
  // noted EHPE in post 08-2019 novatel span csv files. In [m]
  optional<double> estimated_horizontal_error = {};
  optional<double> cog_degrees = {};
  optional<double> hdg_degrees = {};
  optional<bool> is_zupt = {};
};

class Truth {
 public:
  virtual ~Truth() = default;

  virtual optional<TruthEpoch> get_truth_at(const gps_time_t &time) = 0;
  virtual optional<TruthEpoch> get_truth_at(const gps_time_t &time_now,
                                            const gps_time_t &time_prev) = 0;
};

}  // namespace ground_truth

#endif  // GROUND_TRUTH_EPOCH_H
