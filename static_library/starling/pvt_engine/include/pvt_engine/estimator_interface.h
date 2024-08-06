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

#ifndef LIBSWIFTNAV_PVT_ENGINE_ESTIMATOR_H
#define LIBSWIFTNAV_PVT_ENGINE_ESTIMATOR_H

#include <pvt_common/optional.h>

#include <swiftnav/common.h>
#include <swiftnav/logging.h>

#include <pvt_common/containers/set.h>
#include <pvt_engine/covariance_copy.h>
#include <pvt_engine/covariance_interface.h>
#include <pvt_engine/covariance_ref.h>
#include <pvt_engine/degrees_of_freedom.h>
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/filter_data_interface.h>
#include <pvt_engine/finity.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/pvt_types.h>
#include <pvt_engine/single_observation_filter_data.h>
#include <pvt_engine/update_info.h>

namespace pvt_engine {

/** \defgroup estimator_interface EstimatorInterface
 * Defines the abstract interface for an estimator which can be implemented
 * by least squares, (Extended) Kalman Filter, UD Kalman Filter, Unscented
 * Kalman Filter, Particle Filter, etc.
 * \{ */

class EstimatorInterface {
 public:
  EstimatorInterface() = default;
  virtual ~EstimatorInterface() = default;

  virtual PRC initialize(const VectorMaxStateDimd_t &x0,
                         const MatrixMaxStateDimd_t &P0) = 0;

  /**
   *
   * @param F state transition matrix
   * @param Q process noise matrix
   * @return code indicating success or failure type
   */
  virtual PRC predict(const MatrixMaxStateDimd_t &F,
                      const MatrixMaxStateDimd_t &Q) = 0;

  /**
   * Perform a filter update using the observation model function to compute
   * the innovations. Log insight information.
   * @param filter_data measurements for the filter update
   * @param update_insight if not `nullptr`, this is populated with
   *                       diagnostic information about each UD update
   *                       step.
   * @return code indicating success or failure
   */
  virtual PRC update(
      const FilterDataInterface &filter_data,
      pvt_common::containers::StaticVector<UpdateInfo, cMaxFilterObservations>
          *update_insight) = 0;

  virtual PRC add_state(s32 idx, double estimate, double variance) = 0;

  /**
   * Perform a filter update using the observation model function to compute
   * the innovations. Log insight information.
   * @param filter_data measurement for the filter update
   * @param update_insight if not `nullptr`, this is populated with
   *                       diagnostic information about each UD update
   *                       step.
   * @return code indicating success or failure
   */
  virtual void update_scalar(const SingleObsUntaggedFilterData &filter_data,
                             UpdateInfo *update_insight) = 0;

  /**
   * Invert (undo) na filter update using the observation model
   * function to compute the innovations. Log insight information.
   * @param filter_data measurement for the filter update
   * @param update_insight if not `nullptr`, this is populated with
   *                       diagnostic information about each UD update
   *                       step.
   * @return code indicating success or failure
   */
  virtual void revert_scalar(const SingleObsUntaggedFilterData &filter_data,
                             UpdateInfo *update_insight) = 0;

  /**
   *
   * @param idx index of state to drop
   * @return code indicating success or failure type
   */
  virtual PRC drop_states(const pvt_common::containers::Set<s32, cMaxStateDim>
                              &indices_to_drop) = 0;

  /**
   *
   * @param cov full or decomposed covariance
   * @return code indicating success or failure type
   */
  virtual PRC set_covariance(const CovarianceInterface &cov) = 0;

  struct IndexStateAndVariance {
    s32 index;
    double state;
    double variance;

    bool operator<(const IndexStateAndVariance &rhs) const {
      return index < rhs.index;
    }
    bool operator==(const IndexStateAndVariance &rhs) const {
      return index == rhs.index;
    }
  };

  using IndexStateAndVarianceSet =
      pvt_common::containers::Set<IndexStateAndVariance,
                                  cMaxFilterObservations>;

  virtual PRC reinitialize_states(
      const IndexStateAndVarianceSet &indices_states_and_variances) = 0;

  /**
   *
   * @return current state estimate vector
   */
  virtual VectorMaxStateDimd_t get_state() const = 0;

  virtual void set_state(const VectorMaxStateDimd_t &state) = 0;
  /**
   *
   * @return current state covariance
   */
  virtual CovarianceRef get_covariance() const = 0;

  virtual CovarianceCopy copy_covariance() const = 0;

  /**
   *
   * @param idx list of states for which covariance is required
   * @return current subset of states
   */
  virtual VectorMaxStateDimd_t get_state_subset(
      const VectorMaxStateDims32_t &idx) const = 0;

  virtual bool initialized() const = 0;

 private:
  virtual PRC validate_states() const = 0;
};

// \}
}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_ESTIMATOR_H
