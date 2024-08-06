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

#ifndef LIBSWIFTNAV_PVT_ENGINE_BIERMAN_FILTER_H
#define LIBSWIFTNAV_PVT_ENGINE_BIERMAN_FILTER_H

#include <array>

#include <swiftnav/common.h>

#include <pvt_engine/covariance_interface.h>
#include <pvt_engine/covariance_ref.h>
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/estimator_interface.h>
#include <pvt_engine/observation_model_position.h>
#include <pvt_engine/triangular_matrix_as_vector.h>
#include <pvt_engine/utils.h>

namespace pvt_engine {

class BiermanFilter : public EstimatorInterface {
 public:
  BiermanFilter();

  BiermanFilter(const VectorMaxStateDimd_t &x0, const MatrixMaxStateDimd_t &P0);

  PRC initialize(const VectorMaxStateDimd_t &x0,
                 const MatrixMaxStateDimd_t &P0) override;

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

  void update_scalar_internal(double r,
                              const SingleObsUntaggedFilterData &obs_data,
                              UpdateInfo *update_insight);
  PRC update_decorrelated(
      const FilterDataInterface &filter_data,
      std::array<UpdateInfo, cMaxFilterObservations> *update_insight);
  PRC initialize_ud(s32 n, TriangularMatrixAsVector *U,
                    VectorMaxStateDimd_t *D) const;
  PRC matrix_udu(const MatrixMaxStateDimd_t &P, TriangularMatrixAsVector *U,
                 VectorMaxStateDimd_t *D) const;
  PRC propagate_ud(const MatrixMaxStateDimd_t &F, TriangularMatrixAsVector *U,
                   VectorMaxStateDimd_t *D) const;
  PRC propagate_ud(const MatrixMaxStateDimd_t &F, const MatrixMaxStateDimd_t &Q,
                   TriangularMatrixAsVector *U, VectorMaxStateDimd_t *D) const;
  void update_udu_single_measurement(const VectorMaxStateDimd_t &h, double r,
                                     TriangularMatrixAsVector *U,
                                     VectorMaxStateDimd_t *D,
                                     VectorMaxStateDimd_t *k,
                                     double *alpha_out) const;

  bool initialized_;
  VectorMaxStateDimd_t x_;
  TriangularMatrixAsVector U_;
  VectorMaxStateDimd_t D_;
};

MatrixMaxStateDimd_t calc_full_covariance(const BiermanFilter &bf);

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_BIERMAN_FILTER_H
