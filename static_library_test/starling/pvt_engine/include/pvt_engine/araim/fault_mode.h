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

#ifndef STARLING_FAULT_MODE_H
#define STARLING_FAULT_MODE_H

#include <pvt_common/containers/set.h>
#include <pvt_engine/araim/araim_definitions.h>
#include <pvt_engine/araim/failure_event.h>
#include <pvt_engine/eigen_types.h>
#include "pvt_engine/sat_identifier.h"

namespace pvt_engine {
namespace araim {

// ----------------------------------------------------------------------------
// About:
//
// Fault modes refer to unique combinations of simultaneous failure events. Each
// subset of events is treated as a fault hypothesis and the ARAIM algorithm
// offers a set of statistical tests that evaluate whether that fault has
// manifested. The prior probability of a fault mode's occurrence is given by
// the simultaneous probability of all its constituent failure events.
//  Examples: - Simultaneous failures of satellites GPS-01 and GPS-02
//            - Simultaneous failures of GAL constellation and satellite GPS-03
// ----------------------------------------------------------------------------

class FaultMode {
 public:
  FaultMode();

  FaultMode(const ObsIndexSubset &excluded_measurement_indices,
            const double &fault_probability);

  void set_state_estimate(const Eigen::Vector3d &mean,
                          const Eigen::Matrix3d &covariance);
  void set_fit_matrix(const MatrixNumDimsByMaxObsd_t &fit_matrix);
  void set_solution_separation_results(const Eigen::Vector3d &separation,
                                       const Eigen::Vector3d &threshold);
  void set_position_bias(const Eigen::Vector3d &bias);

  ObsIndexSubset get_excluded_measurement_indices() const;
  int number_of_excluded_measurements() const;
  double get_fault_probability() const;
  Eigen::Vector3d get_estimate_mean() const;
  Eigen::Matrix3d get_estimate_covariance() const;
  Eigen::Vector3d get_estimate_std_dev() const;
  MatrixNumDimsByMaxObsd_t get_fit_matrix() const;
  Eigen::Vector3d get_solution_separation() const;
  Eigen::Vector3d get_solution_separation_threshold() const;
  Eigen::Vector3d get_position_bias() const;

  void map_solution_from_ecef_to_ned(const Eigen::Vector3d &x0_ecef,
                                     const Eigen::Matrix3d &R_ecef2ned);

 private:
  // Comments next to each member indicate their variable names according to
  // the ARAIM Report.
  ObsIndexSubset excluded_measurement_indices_;
  double fault_probability_;  // P_fault_(k)

  Eigen::Vector3d state_mean_;        // x_(k)
  Eigen::Matrix3d state_covariance_;  // sigma_(k)

  MatrixNumDimsByMaxObsd_t fit_matrix_;  // S_(k)

  Eigen::Vector3d solution_separation_;            // x - x_(k)
  Eigen::Vector3d solution_separation_threshold_;  // T_(k)

  Eigen::Vector3d position_bias_;  // b_(k)
};

// A large vector to store a list of FaultModes
//  Ex. List of FaultModes currently being monitored by ARAIM algorithm
using FaultModeVector =
    pvt_common::containers::StaticVector<FaultMode, cMaxAllowableFaultModes>;

}  // namespace araim
}  // namespace pvt_engine

#endif  // STARLING_FAULT_MODE_H
