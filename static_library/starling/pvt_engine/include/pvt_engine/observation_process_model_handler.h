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

#ifndef LIBSWIFTNAV_OBSERVATION_PROCESS_MODEL_HANDLER_H
#define LIBSWIFTNAV_OBSERVATION_PROCESS_MODEL_HANDLER_H

#include <pvt_engine/observation_model_ambiguities.h>
#include <pvt_engine/observation_model_decoupled_clock.h>
#include <pvt_engine/observation_model_hardware_biases.h>
#include <pvt_engine/observation_model_hardware_biases_on_pseudorange_only.h>
#include <pvt_engine/observation_model_ionosphere.h>
#include <pvt_engine/observation_model_phase_windup.h>
#include <pvt_engine/observation_model_position_single_differenced.h>
#include <pvt_engine/observation_model_position_undifferenced.h>
#include <pvt_engine/observation_model_single_clock.h>
#include <pvt_engine/observation_model_single_clock_per_constellation.h>
#include <pvt_engine/observation_model_troposphere.h>
#include <pvt_engine/process_model_ambiguity.h>
#include <pvt_engine/process_model_diagonal.h>
#include <pvt_engine/process_model_ionosphere.h>
#include <pvt_engine/process_model_position.h>

namespace pvt_engine {

class ObservationProcessModelHandler {
 public:
  struct ModelPair {
    ModelPair() : is_used(false), obs_model(nullptr), process_model(nullptr) {}

    bool is_used;
    ObservationModelInterface *obs_model;
    ProcessModelInterface *process_model;
  };

  explicit ObservationProcessModelHandler(
      const CommonData &common_data, const ObsProcModelConfiguration &config);

  ObservationProcessModelHandler &operator=(
      const ObservationProcessModelHandler &other);

  PRC initialize(const ObsProcModelTypes models,
                 const ObsProcModelConfiguration &config);

  bool update_config(const ObsProcModelConfiguration &config);

  PRC initialize_model(const MODEL_TYPE model,
                       const ObsProcModelConfiguration &config);

  void update_models(const FilterObservationHandler &observation_handler,
                     EstimatorInterface *estimator,
                     DoF_container *DoF_change_from_states_added);

  void get_process_model(const double &delta_time, MatrixMaxStateDimd_t *F,
                         MatrixMaxStateDimd_t *Q,
                         DoF_container *DoF_lost) const;

  void get_partial_reinitialization(MatrixMaxStateDimd_t *F,
                                    MatrixMaxStateDimd_t *Q,
                                    DoF_container *DoF_lost) const;

  pvt_common::containers::StaticVector<StateAndModel, cMaxStateDim>
  get_state_labels() const;

  optional<const ModelPair *> get_model(MODEL_TYPE model) const;

  optional<s32> get_model_offset(MODEL_TYPE model) const;

  optional<s32> get_position_model_offset() const;

  s32 get_number_of_states(MODEL_TYPE model) const;

  s32 get_total_number_of_states() const;

  PRC get_observation_model(
      const Eigen::Vector3d &linearization_point, const Observation &obs,
      VectorMaxStateDimd_t *H_row, ObservationModelFunctionRow *h_function,
      pvt_common::containers::Map<ObservationIdentifier, LabeledObservation,
                                  cMaxFilterObservations> *obs_insight) const;

  s32 get_num_signals() const;

  PRC get_position_estimate(const EstimatorInterface &estimator,
                            VectorMaxKinematicStatesd_t *position,
                            MatrixMaxKinematicStatesd_t *covariance) const;

  PRC get_ambiguity_indices(
      s32 *ambiguity_model_offset,
      ambiguities::AmbiguityIndexMap *ambiguity_index_map) const;

  ObsProcModelTypes get_current_models() const;

  PRC reinitialize_amb_states(const FilterObservationIdSet &obs_ids,
                              EstimatorInterface *estimator,
                              DoF_container *DoF);

 private:
  PRC setup_models(MODEL_TYPE model_enum);

  pvt_common::containers::StaticVector<ModelPair, MAX_MODEL_NUM>
      obs_proc_models_;

  ObservationModelPositionSingleDifferenced
      position_obs_model_single_differenced_;
  ProcessModelPosition position_proc_model_single_differenced_;

  ObservationModelPositionUndifferenced position_obs_model_undifferenced_;
  ProcessModelPosition position_proc_model_undifferenced_;

  ObservationModelDecoupledClock decoupled_clock_obs_model_;
  ProcessModelDiagonal decoupled_clock_proc_model_;

  ObservationModelSingleClock single_clock_obs_model_;
  ProcessModelDiagonal single_clock_proc_model_;

  ObservationModelSingleClockPerConstellation
      single_clock_per_constellation_obs_model_;
  ProcessModelDiagonal single_clock_per_constellation_proc_model_;

  ObservationModelHardwareBiases hardware_biases_obs_model_;
  ProcessModelDiagonal hardware_biases_proc_model_;

  ObservationModelHardwareBiasesOnPseudorangeOnly
      hardware_biases_on_pseudorange_only_obs_model_;
  ProcessModelDiagonal hardware_biases_on_pseudorange_only_proc_model_;

  ObservationModelPhaseWindup phase_windup_obs_model_;
  ProcessModelDiagonal phase_windup_proc_model_;

  ObservationModelTroposphere troposphere_obs_model_;
  ProcessModelDiagonal troposphere_proc_model_;

  ObservationModelIonosphere ionosphere_obs_model_;
  ProcessModelIonosphere ionosphere_proc_model_;

  ObservationModelAmbiguities ambiguities_obs_model_;
  ProcessModelAmbiguity ambiguities_proc_model_;

  ObsProcModelTypes models_not_to_reset_in_partial_reinit_;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_OBSERVATION_PROCESS_MODEL_HANDLER_H
