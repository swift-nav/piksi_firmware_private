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

#ifndef LIBSWIFTNAV_PVT_ENGINE_ROBUST_ESTIMATOR_H
#define LIBSWIFTNAV_PVT_ENGINE_ROBUST_ESTIMATOR_H

#include <optional.hpp>

#include <pvt_engine/estimator_interface.h>
#include <pvt_engine/filter_data_interface.h>
#include <pvt_engine/filter_update_insight.h>
#include <pvt_engine/gaussian.h>
#include <pvt_engine/outlier_detection_insight.h>
#include <pvt_engine/tagged_filter_data.h>

constexpr u32 FOUR_SIGNIFICANT_DIGITS = 4;

namespace pvt_engine {

struct FilterInsights {
  FilterInsights() : filter_update_(), outlier_detection_() {}

  FilterUpdateInsights filter_update_;
  OutlierDetectionInsight outlier_detection_;
};

class RobustEstimator {
 public:
  explicit RobustEstimator(const RobustEstimatorConfiguration &config)
      : config_(config){};

  PRC do_outlier_detection(
      const gps_time_t &current_epoch_time,
      const optional<Eigen::Vector3i> &position_indices,
      const optional<Eigen::Vector3i> &velocity_indices,
      TaggedFilterData *accepted_filter_data,
      TaggedFilterData *rejected_filter_data, EstimatorInterface *estimator,
      FilterInsights *filter_insight,
      SumSquaredNormalizedResiduals *epoch_sum_squared_normalized_residuals,
      optional<double> *variance_factor) const;

 private:
  optional<s32> find_outlier(
      const FilterDataInterface &processed_filter_data,
      const EstimatorInterface &estimator, const VectorMaxObsd_t &HPHt_diag,
      const double &DoF, OutlierInfo *outlier_insight,
      pvt_common::containers::StaticVector<Gaussian, cMaxFilterObservations>
          *residual_distributions) const;

  bool is_outlier(double residual, u64 scaledup_ratio) const;

  s32 get_max_num_outliers(const VectorMaxObsd_t &HPHt_diag,
                           const FilterDataInterface &filter_data) const;

  RobustEstimatorConfiguration config_;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_ROBUST_ESTIMATOR_H
