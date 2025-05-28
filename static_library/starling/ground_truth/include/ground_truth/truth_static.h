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

#ifndef GROUND_TRUTH_STATIC_H
#define GROUND_TRUTH_STATIC_H

#include <swiftnav/constants.h>
#include <swiftnav/coord_system.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/logging.h>

#include <pvt_common/eigen_custom.h>
#include <pvt_common/optional.h>

#include "ground_truth/truth_epoch.h"

namespace ground_truth {

class TruthStatic : public Truth {
 public:
  TruthStatic(const optional<Eigen::Vector3d> &truth_position_ecef,
              const optional<Eigen::Vector3d> &truth_reference_ecef,
              const optional<Eigen::Vector3d> &truth_baseline);

  optional<TruthEpoch> get_truth_at(const gps_time_t &time) override;
  optional<TruthEpoch> get_truth_at(const gps_time_t &time_now,
                                    const gps_time_t &time_prev) override;

 private:
  optional<Eigen::Vector3d> truth_position_ecef_;
  optional<Eigen::Vector3d> truth_reference_ecef_;
  optional<Eigen::Vector3d> truth_baseline_;
};

}  // namespace ground_truth

#endif  // GROUND_TRUTH_STATIC_H
