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

#include <pvt_common/optional.h>

#include <pvt_engine/ambiguity_types.h>
#include <pvt_engine/apriori_model_handler.h>
#include <pvt_engine/bierman_filter.h>
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/estimator_interface.h>
#include <pvt_engine/kalman_filter.h>
#include <pvt_engine/observation_handler.h>
#include <pvt_engine/observation_model_interface.h>
#include <pvt_engine/observation_process_model.h>
#include <pvt_engine/position_prior.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/pvt_types.h>
#include <pvt_engine/robust_estimator.h>
#include <swiftnav/macros.h>

namespace pvt_engine {

enum class RequestedStates : bool { OnlyPositionStates, AllStates };

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

  Eigen::Vector3d get_position_states() const;

  // This function returns a vector of position states in the format X, Y, Z,
  // vX, vY, vZ, aX, aY, aZ. If the size is smaller than nine, the ordering is
  // retained and the last states aren't present. The covariance matrix follows
  // the same principle.
  void get_kinematic_states_and_covariances(
      VectorMaxKinematicStatesd_t *kinematic_state,
      MatrixMaxKinematicStatesd_t *kinematic_state_covariance) const;

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

  Filter(const Filter &other) = default;

  Filter &operator=(const Filter &other) = default;

  virtual ~Filter() = default;

  void initialize(const FilterConfiguration &config);

  void partial_reinitialization();

  const FilterConfiguration &get_config() const;

  bool update_config(const FilterConfiguration &config);

  PRC update(const obs_filters::CodeSet &codes_to_fix,
             FilterObservationIdSet *ids_to_reset,
             ResetFilter *is_reset_needed);

  PRC predict(const double delta_time, DoF_container *DoF_lost);

  s32 get_num_signals() const;

  s32 get_num_satellites() const;

  PRC get_position_estimate(VectorMaxKinematicStatesd_t *position,
                            MatrixMaxKinematicStatesd_t *covariance) const;

  PRC get_baseline_estimate(VectorMaxKinematicStatesd_t *baseline,
                            MatrixMaxKinematicStatesd_t *covariance) const;

  PRC get_ambiguity_state(
      s32 *ambiguity_model_offset,
      ambiguities::AmbiguityIndexMap *ambiguity_index_map) const;

  optional<FilterState> get_state_estimate_covariance(
      RequestedStates requested_states) const;

  PRC process_widelane_fixed_epoch(
      const ambiguities::TransformedIntegerAmbiguities
          &validated_widelane_ambiguities,
      const FixedSolutionType &fixed_solution_type,
      FilterState *filter_state) const;

  PRC get_ambiguity_estimates(
      ambiguities::AmbiguitiesAndCovariances *ambiguity_estimates) const;

  bool initialized() const;

  optional<gps_time_t> get_last_update_time() const;

  optional<const double> get_update_delta_time() const;

  DOPS get_last_dops() const;

  const DoF_container &get_dof() const;
  SumSquaredNormalizedResiduals get_sum_squared_normalized_residuals() const;

  FilterUpdateInsights get_filter_update_insight() const;

  OutlierDetectionInsight get_outlier_detection_insight() const;

  void reset_outlier_detection_insight();

  FilterObservationHandler obs_handler_;

  double get_instantaneous_velocity_variance_scale_factor() const;

 protected:
  virtual PRC update_filter(TaggedFilterData *accepted_observations,
                            FilterObservationIdSet *ids_to_reset,
                            ResetFilter *is_reset_needed,
                            DoF_container *new_DoF);

  PRC constrain_position(const double position_ecef[3],
                         const double position_var[3],
                         EstimatorInterface *estimator);

  PRC measurement_update(const VectorMaxStateDimd_t &prior_state,
                         const CovarianceCopy &prior_covariance,
                         bool get_insight, EstimatorInterface *estimator,
                         TaggedFilterData *accepted_observations);

  PRC run_outlier_detection(const gps_time_t &current_epoch_time,
                            const VectorMaxStateDimd_t &prior_state,
                            const CovarianceCopy &prior_covariance,
                            bool get_insight, EstimatorInterface *estimator,
                            TaggedFilterData *accepted_observations,
                            TaggedFilterData *rejected_observations,
                            ResetFilter *is_reset_needed,
                            FilterObservationIdSet *ids_to_reset,
                            DoF_container *new_DoF);

  PRC get_state_estimate_covariance(RequestedStates requested_states,
                                    VectorMaxStateDimd_t *state_estimate,
                                    MatrixMaxStateDimd_t *covariance) const;

  double get_delta_time(const gps_time_t &obs_time) const;

  PRC check_dop_limits(const DOPS &dop_values);

  void reinitialize_amb_states(const TaggedFilterData &reinit,
                               DoF_container *new_DoF);

  PRC dop_update();

  PRC get_obs_and_model(TaggedFilterData *observations) const;
  VectorMaxStateDimd_t get_float_state_with_zeroed_ambs() const;
  void remove_apriori_effects(
      const TaggedFilterData &observations,
      const VectorMaxStateDimd_t &float_state_with_zeroed_ambs,
      const ambiguities::FixedAmbiguityVector &fixed_amb_values,
      const MatrixMaxStateDimd_t &obs_to_amb_transformation_matrix,
      const pvt_common::containers::StaticVector<double, cMaxAmbiguities>
          &widelane_wavelengths,
      VectorMaxObsd_t *phase_y) const;
  bool get_widelane_phase_obs(
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

  double get_average_velocity_variance_scale_factor() const;

  optional<Eigen::Vector3i> get_position_model_indices(
      const POSITION_MODEL_COMPONENT &model_component) const;

  virtual PRC get_linearization_point(
      VectorMaxKinematicStatesd_t *linearization_point) const = 0;
  virtual bool is_converged(const VectorMaxKinematicStatesd_t
                                &previous_linearization_point) const = 0;
  virtual bool consider_dop() const = 0;

  FilterConfiguration config_;
  BiermanFilter bf_;
  RobustEstimator robust_estimator_;
  ObservationProcessModel observation_process_model_;

  s32 num_iterations_;
  optional<gps_time_t> last_updated_time_;
  optional<double> delta_time_;
  DOPS dop_values_;
  DoF_container DoF_;
  SumSquaredNormalizedResiduals sum_squared_normalized_residuals_;
  s32 consecutive_epochs_with_excessive_outliers_;
  optional<gps_time_t> start_of_mde_failure_;
  optional<double> variance_factor_;
  FilterInsights filter_insights_;
};

class FilterWithPosition : public Filter {
 public:
  FilterWithPosition(const FilterConfiguration &config,
                     const CommonData &common_data);

  ~FilterWithPosition() override = default;

  void set_position_prior(const PositionPrior &new_prior);
  void clear_position_prior();

 protected:
  PRC update_filter(TaggedFilterData *accepted_observations,
                    FilterObservationIdSet *ids_to_reset,
                    ResetFilter *is_reset_needed,
                    DoF_container *new_DoF) override;

  PRC get_linearization_point(
      VectorMaxKinematicStatesd_t *linearization_point) const final {
    return get_position_estimate(linearization_point, nullptr);
  }

  bool is_converged(const VectorMaxKinematicStatesd_t
                        &previous_linearization_point) const final {
    VectorMaxKinematicStatesd_t current_linearization_point;
    PRC rc SWIFT_ATTR_UNUSED =
        get_position_estimate(&current_linearization_point, nullptr);
    assert(prc::success(rc));

    // If the position is not moving after the update, no need to
    // iterate as the non-linear errors will already be negligible.
    return (current_linearization_point - previous_linearization_point).norm() <
           config_.max_tolerance_to_iterate;
  }

  bool consider_dop() const final { return true; }

  optional<PositionPrior> position_prior_;
};

class FilterWithoutPosition : public Filter {
 public:
  FilterWithoutPosition(const FilterConfiguration &config,
                        const CommonData &common_data);

  ~FilterWithoutPosition() override = default;
  void set_linearization_point(
      const VectorMaxKinematicStatesd_t &linearization_point);

 protected:
  PRC get_linearization_point(
      VectorMaxKinematicStatesd_t *linearization_point) const final;
  bool is_converged(const VectorMaxKinematicStatesd_t
                        &previous_linearization_point) const final;
  bool consider_dop() const final;

  optional<VectorMaxKinematicStatesd_t> linearization_point_;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_FILTER_H
