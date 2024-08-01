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

#ifndef LIBSWIFTNAV_PVT_ENGINE_FILTER_H
#define LIBSWIFTNAV_PVT_ENGINE_FILTER_H

#include <pvt_engine/optional.h>

#include <starling/observation.h>

#include <pvt_engine/ambiguity_types.h>
#include <pvt_engine/apriori_model_handler.h>
#include <pvt_engine/bierman_filter.h>
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/estimator_interface.h>
#include <pvt_engine/filter_update_insight.h>
#include <pvt_engine/kalman_filter.h>
#include <pvt_engine/observation_handler.h>
#include <pvt_engine/observation_model_interface.h>
#include <pvt_engine/observation_process_model.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/pvt_types.h>
#include <pvt_engine/robust_estimator.h>

namespace pvt_engine {

// Class that contains the filter state, inclusive of state labels, estimates
// and covariances. Provides functions to obtain access to various subsections
// of the states and covariances, together with their labels.
class FilterState {
 public:
  FilterState();
  FilterState(
      const VectorMaxStateDimd_t &state_vector_,
      const MatrixMaxStateDimd_t &covariance_matrix_,
      const pvt_common::containers::StaticVector<StateAndModel, cMaxStateDim>
          &state_labels_);

  Eigen::Vector3d get_baseline() const;

  // This function returns a vector of position states in the format X, Y, Z,
  // vX, vY, vZ, aX, aY, aZ. If the size is smaller than nine, the ordering is
  // retained and the last states aren't present. The covariance matrix follows
  // the same principle.
  void get_baseline_states_covariances(
      VectorMaxBaselineStatesd_t *position,
      MatrixMaxBaselineStatesd_t *position_covariance) const;

  const VectorMaxStateDimd_t &get_state_estimates() const;
  const MatrixMaxStateDimd_t &get_covariances() const;
  const pvt_common::containers::StaticVector<StateAndModel, cMaxStateDim>
      &get_state_labels() const;

  ambiguities::FloatAmbiguityStates get_ambiguities() const;

  void partition_states(VectorMaxStateDimd_t *non_amb_states,
                        VectorMaxStateDimd_t *amb_states,
                        MatrixMaxStateDimd_t *Qaa, MatrixMaxStateDimd_t *Qba,
                        MatrixMaxStateDimd_t *Qbb) const;

  ambiguities::AmbiguityIndexMap get_amb_index_map() const;

  pvt_common::containers::StaticVector<StateAndModel, cMaxStateDim>
  get_non_amb_labels() const;

 private:
  void get_amb_offset_and_number_of_ambs(s32 *amb_model_offset,
                                         s32 *num_amb_states) const;

  VectorMaxStateDimd_t state_vector;
  MatrixMaxStateDimd_t covariance_matrix;
  pvt_common::containers::StaticVector<StateAndModel, cMaxStateDim>
      state_labels;
};

class Filter {
 public:
  enum class ResetFilter : bool { ResetNeeded, NoResetNeeded };

  Filter(const FilterConfiguration &config, const CommonData &common_data);

  Filter(const Filter &other);

  Filter &operator=(const Filter &other);

  void initialize(const FilterConfiguration &config);

  void partial_reinitialization();

  const FilterConfiguration &get_config() const;

  bool update_config(const FilterConfiguration &config);

  PRC update(const FilterObservationHandler &observation_handler,
             const ambiguities::CodeSet &codes_to_fix,
             FilterObservationHandler *filtered_observation_handler_out,
             FilterObservationIdSet *ids_to_reset,
             ResetFilter *is_reset_needed);

  s32 get_num_signals() const;

  s32 get_num_satellites() const;

  PRC get_position_estimate(VectorMaxBaselineStatesd_t *position,
                            MatrixMaxBaselineStatesd_t *covariance) const;

  PRC get_baseline_estimate(VectorMaxBaselineStatesd_t *baseline,
                            MatrixMaxBaselineStatesd_t *covariance) const;

  PRC get_ambiguity_state(
      s32 *ambiguity_model_offset,
      ambiguities::AmbiguityIndexMap *ambiguity_index_map) const;

  optional<FilterState> get_state_estimate_covariance() const;

  optional<FilterState> process_widelane_fixed_epoch(
      const ambiguities::TransformedIntegerAmbiguities
          &validated_widelane_ambiguities) const;

  PRC get_ambiguity_estimates(
      ambiguities::AmbiguitiesAndCovariances *ambiguity_estimates) const;

  bool initialized() const;
  optional<const FilterObservationHandler &> get_obs_handler() const;

  optional<gps_time_t> get_last_update_time() const;

  optional<const double> get_update_delta_time() const;

  DOPS get_last_dops() const;

  const DoF_container &get_dof() const;
  SumSquaredNormalizedResiduals get_sum_squared_normalized_residuals() const;

  FilterUpdateInsights get_insight() const;

 private:
  PRC get_state_estimate_covariance(VectorMaxStateDimd_t *state_estimate,
                                    MatrixMaxStateDimd_t *covariance) const;

  double get_delta_time(const gps_time_t &obs_time) const;

  void assign_estimator();

  PRC check_dop_limits(const DOPS &dop_values);

  PRC reinitialize_amb_state(const ObservationIdentifier &obs_id,
                             DoF_container *new_DoF);

  void reinitialize_amb_states(const TaggedFilterData &reinit,
                               DoF_container *new_DoF);

  PRC measurement_update(const FilterObservationHandler &observations,
                         TaggedFilterData *accepted_observations,
                         FilterObservationIdSet *ids_to_reset,
                         ResetFilter *is_reset_needed, DoF_container *new_DoF);

  PRC dop_update(const FilterObservationHandler &obs_handler);

  optional<TaggedFilterData> get_obs_and_model() const;
  VectorMaxStateDimd_t get_float_state_with_zeroed_ambs() const;
  void remove_apriori_effects(
      const TaggedFilterData &observations,
      const VectorMaxStateDimd_t &float_state_with_zeroed_ambs,
      const ambiguities::TransformedIntegerAmbiguities &validated_ambiguities,
      const MatrixMaxStateDimd_t &obs_to_amb_transformation_matrix,
      const pvt_common::containers::StaticVector<double, cMaxAmbiguities>
          &widelane_wavelengths,
      VectorMaxObsd_t *phase_y) const;
  void get_widelane_phase_obs(
      const TaggedFilterData &observations,
      const VectorMaxStateDimd_t &float_state_with_zeroed_ambs,
      const ambiguities::TransformedIntegerAmbiguities
          &validated_widelane_ambiguities,
      VectorMaxObsd_t *phase_y, MatrixMaxObsByStatesd_t *phase_H,
      MatrixMaxObsd_t *phase_R) const;
  void remove_unobserved_states(
      VectorMaxStateDimd_t *float_state_with_zeroed_ambs,
      pvt_common::containers::StaticVector<StateAndModel, cMaxStateDim>
          *state_labels,
      MatrixMaxObsByStatesd_t *H) const;

  double get_position_variance_scale_factor() const;

  double get_velocity_variance_scale_factor() const;

  FilterConfiguration config_;
  KalmanFilter kf_;
  BiermanFilter bf_;
  ESTIMATOR_TYPE current_estimator_;
  EstimatorInterface *estimator_;
  RobustEstimator robust_estimator_;
  ObservationProcessModel observation_process_model_;

  // This is `optional` because if it was a fixed field, we'd need to
  // take an `ObservationHandlerConfiguration` in our constructor to
  // populate an empty observation handler.  `optional` more
  // accurately represents the fact that we may not have anything here
  // until we receive some observations, and thereby also allows us to
  // avoid the configuration issue.
  optional<FilterObservationHandler> obs_handler_;

  FilterUpdateInsights update_insight_;

  s32 num_iterations_;
  optional<gps_time_t> last_updated_time_;
  optional<double> delta_time_;
  DOPS dop_values_;
  DoF_container DoF_;
  SumSquaredNormalizedResiduals sum_squared_normalized_residuals_;
  s32 consecutive_epochs_with_excessive_outliers_;
  optional<double> variance_factor_;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_FILTER_H
