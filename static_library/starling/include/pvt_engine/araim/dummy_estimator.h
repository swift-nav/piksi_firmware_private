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

#ifndef STARLING_DUMMY_ESTIMATOR_H
#define STARLING_DUMMY_ESTIMATOR_H

#include <pvt_engine/estimator_interface.h>

namespace pvt_engine {
namespace araim {

// TODO(STAR-925) Refactor the obs process model to not require
//  an estimator in the update function
class DummyEstimator : public EstimatorInterface {
 public:
  DummyEstimator() : P(){};
  ~DummyEstimator() final{};

  PRC initialize(const VectorMaxStateDimd_t &a,
                 const MatrixMaxStateDimd_t &b) final {
    (void)a;
    (void)b;
    return RC_S_OK;
  }

  PRC predict(const MatrixMaxStateDimd_t &a,
              const MatrixMaxStateDimd_t &b) final {
    (void)a;
    (void)b;
    return RC_S_OK;
  }

  PRC update(
      const FilterDataInterface &a,
      pvt_common::containers::StaticVector<UpdateInfo, cMaxFilterObservations>
          *b) final {
    (void)a;
    (void)b;
    return RC_S_OK;
  }

  PRC add_state(s32 a, double b, double c) final {
    (void)a;
    (void)b;
    (void)c;
    return RC_S_OK;
  }

  void update_scalar(const SingleObsUntaggedFilterData &a,
                     UpdateInfo *b) final {
    (void)a;
    (void)b;
  }

  void revert_scalar(const SingleObsUntaggedFilterData &a,
                     UpdateInfo *b) final {
    (void)a;
    (void)b;
  }

  PRC drop_state(s32 a) final {
    (void)a;
    return RC_S_OK;
  }

  PRC set_covariance(const CovarianceInterface &a) final {
    (void)a;
    return RC_S_OK;
  }

  PRC reinitialize_state(s32 a, double b) final {
    (void)a;
    (void)b;
    return RC_S_OK;
  }

  PRC reinitialize_states(const IndexAndVarianceVector &a) final {
    (void)a;
    return RC_S_OK;
  }

  VectorMaxStateDimd_t get_state() const final { return {}; }

  void set_state(const VectorMaxStateDimd_t &a) final { (void)a; }

  CovarianceRef get_covariance() const final { return CovarianceRef{P}; }

  CovarianceCopy copy_covariance() const final { return CovarianceCopy({}); }

  VectorMaxStateDimd_t get_state_subset(
      const VectorMaxStateDims32_t &a) const final {
    (void)a;
    return {};
  }

  optional<CovarianceCopy> get_covariance_subset(
      const VectorMaxStateDims32_t &a) const final {
    (void)a;
    return {};
  }

  bool initialized() const final { return true; }

  PRC validate_states() const final { return RC_S_OK; }

  const MatrixMaxStateDimd_t P;
};

}  // namespace araim
}  // namespace pvt_engine

#endif  // STARLING_DUMMY_ESTIMATOR_H
