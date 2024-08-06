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

#ifndef LIBSWIFTNAV_PVT_ENGINE_KALMAN_FILTER_H
#define LIBSWIFTNAV_PVT_ENGINE_KALMAN_FILTER_H

#include <array>

#include <swiftnav/common.h>

#include <pvt_engine/covariance_copy.h>
#include <pvt_engine/covariance_interface.h>
#include <pvt_engine/covariance_ref.h>
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/estimator_interface.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-overflow"
#include <pvt_engine/utils.h>
#pragma GCC diagnostic pop

namespace pvt_engine {

class KalmanFilter : public EstimatorInterface {
 public:
  KalmanFilter();

  explicit KalmanFilter(const VectorMaxStateDimd_t &x0,
                        const MatrixMaxStateDimd_t &P0);

  PRC initialize(const VectorMaxStateDimd_t &x0,
                 const MatrixMaxStateDimd_t &P0) final;

  PRC predict(const MatrixMaxStateDimd_t &F,
              const MatrixMaxStateDimd_t &Q) override;

  PRC update(
      const FilterDataInterface &filter_data,
      pvt_common::containers::StaticVector<UpdateInfo, cMaxFilterObservations>
          *update_insight) override;

  PRC add_state(s32 idx, double estimate, double variance) override;

  void update_scalar(const SingleObsUntaggedFilterData &obs_data,
                     UpdateInfo *update_insight) override;

  void revert_scalar(const SingleObsUntaggedFilterData &obs_data,
                     UpdateInfo *update_insight) override;

  PRC drop_states(const pvt_common::containers::Set<s32, cMaxStateDim>
                      &indices_to_drop) override;

  PRC set_covariance(const CovarianceInterface &cov) override;

  PRC reinitialize_states(const EstimatorInterface::IndexStateAndVarianceSet
                              &indices_states_and_variances) override;

  VectorMaxStateDimd_t get_state() const override;

  void set_state(const VectorMaxStateDimd_t &state) override;

  CovarianceRef get_covariance() const override;
  CovarianceCopy copy_covariance() const override;

  VectorMaxStateDimd_t get_state_subset(
      const VectorMaxStateDims32_t &idx) const override;

  bool initialized() const override;

 private:
  PRC validate_states() const override;
  void update_scalar_internal(double R,
                              const SingleObsUntaggedFilterData &obs_data,
                              UpdateInfo *update_insight);

  bool initialized_;
  VectorMaxStateDimd_t x_;
  MatrixMaxStateDimd_t P_;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_KALMAN_FILTER_H
