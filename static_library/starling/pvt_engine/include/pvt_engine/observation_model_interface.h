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

#ifndef LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_INTERFACE_H
#define LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_INTERFACE_H

#include <string>
#include <vector>

#include <swiftnav/common.h>

#include <pvt_engine/configuration.h>
#include <pvt_engine/covariance_ref.h>
#include <pvt_engine/degrees_of_freedom.h>
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/observation_handler.h>
#include <pvt_engine/observation_model_function.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/pvt_types.h>

namespace pvt_engine {

// Forward declaration to avoid circular dependency and make
// estimator_interface.h more separated from this file.
class EstimatorInterface;

typedef ObservationIdentifier StateIdentifier;

// Class the defines a state uniquely by coupling the model it relates to with
// the state identifier from within that model.
struct StateAndModel {
  StateAndModel() : state_id_(), model_() {}
  StateAndModel(const StateIdentifier &state_id, const MODEL_TYPE &model)
      : state_id_(state_id), model_(model) {}
  StateIdentifier state_id_;
  MODEL_TYPE model_;
};

inline bool operator==(const StateAndModel &lhs, const StateAndModel &rhs) {
  return (lhs.state_id_ == rhs.state_id_) && (lhs.model_ == rhs.model_);
}

using ObsID2IndexMap =
    pvt_common::containers::Map<StateIdentifier, s32, cMaxStateDim>;

/** \defgroup observation_model_interface ObservationModelInterface
 * Defines the abstract interface for a Observation model.
 * \{ */

class ObservationModelInterface {
 public:
  explicit ObservationModelInterface(
      const ObservationModelConfiguration &config, const MODEL_TYPE model_type)
      : config_(config),
        model_type_(model_type),
        index_map_(),
        DoF_from_states_added_or_reinit_(){};

  virtual ~ObservationModelInterface() = default;

  virtual s32 get_max_size() const = 0;

  /**
   * Initialization of the Observation model
   *
   * @param config configuration for the observation models being initialized
   * @return PRC return code indicating success or failure
   */
  virtual PRC initialize(const ObservationModelConfiguration &config) = 0;

  virtual bool update_config(const ObservationModelConfiguration &config);

  void get_partial_reinitialization(MatrixMaxStateDimd_t *F,
                                    MatrixMaxStateDimd_t *Q,
                                    DoF_container *dof_loss) const;

  PRC get_observation_model_vector(
      const VectorMaxKinematicStatesd_t &linearization_point,
      const Observation &obs, VectorMaxStateDimd_t *obs_vector,
      s32 model_offset) const {
    return get_observation_model_vector(linearization_point, obs, obs_vector,
                                        nullptr, model_offset);
  }

  virtual PRC get_observation_model_vector(
      const VectorMaxKinematicStatesd_t &linearization_point,
      const Observation &obs, VectorMaxStateDimd_t *obs_vector,
      ObservationModelFunction *obs_model_function, s32 model_offset) const = 0;

  /**
   *  Iterate through the observation handler and adjust the estimator for any
   * added or deleted states
   * @param observation_handler set of observations to be processed
   * @param state_offset - state_offset, indicating the number of states from
   * observation models processed already
   * @param estimator - reference to the estimator that holds the
   * state/covariance information
   * @return PRC return code indicating success or failure
   */
  virtual PRC update_states(const FilterObservationHandler &observation_handler,
                            s32 state_offset,
                            EstimatorInterface *estimator) = 0;

  /**
   * Get the total number of states of this observation model in the estimator
   * @return Number of states currently owned by this observation model
   */
  s32 get_number_of_states() const;

  /**
   *
   * @return model type of underlying observation model
   */
  MODEL_TYPE get_model_type() const { return model_type_; };

  const ObsID2IndexMap &get_index_map() const;

  virtual bool is_state_observed(const ObservationIdentifier &obs_id,
                                 const s32 state_offset,
                                 const EstimatorInterface &estimator) const = 0;

  // This check makes sure that a state has been observed - if it hasn't and
  // we reset it, we shouldn't remove a degree of freedom
  bool is_state_observed(const StateIdentifier &state_id,
                         const s32 state_offset,
                         const EstimatorInterface &estimator,
                         const double &initial_variance) const;

  void reset_DoF_from_states_added_or_reinit() {
    DoF_from_states_added_or_reinit_.clear();
  }
  const DoF_container &get_DoF_from_states_added_or_reinit() const {
    return DoF_from_states_added_or_reinit_;
  }

  virtual double get_initial_variance(
      const StateIdentifier &state_id) const = 0;

  pvt_common::containers::StaticVector<StateAndModel, cMaxStateDim>
  get_state_labels() const;

 protected:
  PRC drop_signals(const FilterObservationIdSet &signals_to_drop,
                   const s32 state_offset, EstimatorInterface *estimator);

  PRC add_signal(const StateIdentifier &state_id,
                 const double &initial_state_estimate,
                 const double &initial_variance, const s32 &state_offset,
                 EstimatorInterface *estimator);

  PRC check_dropped_signal(const FilterObservationIdSet &signals_this_epoch,
                           const s32 &state_offset,
                           EstimatorInterface *estimator);

  void check_state_completeness(
      const FilterObservationIdSet &signals_this_epoch);
  /**
   *
   * @param obs_model_offset offset to the absolute start of this observation
   * models state
   * @param relative_offset relative offset into this observation model
   * @return
   */
  static s32 get_estimator_index(s32 obs_model_offset, s32 relative_offset) {
    return (obs_model_offset + relative_offset);
  };

  // All models must use the convention that the sid.code == CODE_COUNT in the
  // return value indicates that the state isn't constellation dependent when
  // implementing this function
  virtual StateIdentifier get_state_id(
      const ObservationIdentifier &obs_id) const = 0;

  virtual double get_dof_loss(const StateIdentifier &state_id) const = 0;

  // See comments in source file
  double get_dof_loss_scale_factor(
      const double &inverse_var_sum,
      const double &obs_state_mapping_factor) const;

  // See comments in source file
  double get_dof_loss(const double &initial_variance,
                      const double &dof_loss_scale_factor) const;

  double get_obs_var_meters(
      const OBSERVATION_TYPE &obs_type,
      const VarianceHandlerConfiguration &variance_config) const;

  // Initialize the inverse variance sum of the observations that observe each
  // state (to be used in get_dof_loss_scale_factor)
  virtual void init_inverse_var_sum(
      const VarianceHandlerConfiguration &variance_config) = 0;

  ObservationModelConfiguration config_;
  MODEL_TYPE model_type_;
  ObsID2IndexMap index_map_;
  DoF_container DoF_from_states_added_or_reinit_;
};
// \}

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_INTERFACE_H
