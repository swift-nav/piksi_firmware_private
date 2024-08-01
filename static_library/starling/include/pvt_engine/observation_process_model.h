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

#ifndef LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_PROCESS_MODEL_H
#define LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_PROCESS_MODEL_H

#include <pvt_engine/common_data.h>
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/observation_handler.h>
#include <pvt_engine/observation_model_ambiguities.h>
#include <pvt_engine/observation_model_decoupled_clock.h>
#include <pvt_engine/observation_model_function_row.h>
#include <pvt_engine/observation_model_interface.h>
#include <pvt_engine/observation_model_phase_windup.h>
#include <pvt_engine/observation_model_position.h>
#include <pvt_engine/observation_model_troposphere.h>
#include <pvt_engine/observation_process_model_handler.h>
#include <pvt_engine/process_model_diagonal.h>
#include <pvt_engine/process_model_position.h>
#include <pvt_engine/pvt_types.h>
#include <pvt_engine/tagged_filter_data.h>
#include <pvt_engine/variance_handler.h>

namespace pvt_engine {

class ObservationProcessModel {
 public:
  explicit ObservationProcessModel(const CommonData &common_data,
                                   const ObsProcModelConfiguration &config);

  ObservationProcessModel &operator=(const ObservationProcessModel &other);

  PRC initialize(const ObsProcModelTypes &models,
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

  PRC get_observation_model(
      const Eigen::Vector3d &linearization_point,
      const FilterObservationHandler &observation_handler,
      TaggedFilterData *filter_data,
      pvt_common::containers::Map<ObservationIdentifier, LabeledObservation,
                                  cMaxFilterObservations> *obs_insight) const;

  s32 get_num_signals() const;

  PRC get_position_estimate(const EstimatorInterface &estimator,
                            VectorMaxBaselineStatesd_t *position,
                            MatrixMaxBaselineStatesd_t *covariance) const;

  PRC get_baseline_estimate(const EstimatorInterface &estimator,
                            VectorMaxBaselineStatesd_t *baseline,
                            MatrixMaxBaselineStatesd_t *covariance) const;

  PRC get_ambiguity_indices(
      s32 *ambiguity_model_offset,
      ambiguities::AmbiguityIndexMap *ambiguity_index_map) const;

  pvt_common::containers::StaticVector<StateAndModel, cMaxStateDim>
  get_state_labels() const;

  DOPS compute_DOPS(const FilterObservationHandler &obs_handler,
                    const Eigen::Vector3d &position_estimate,
                    const EstimatorInterface &estimator) const;

  PRC reinitialize_state(const MODEL_TYPE &model,
                         const ObservationIdentifier &obs_id,
                         EstimatorInterface *estimator, DoF_container *DoF);

 private:
  ObservationProcessModelHandler models_;
  VectorMaxAmbsd_t extract_amb_covariance_diagonal(
      const EstimatorInterface &estimator) const;
  MatrixMaxStateDimd_t extract_amb_covariance(
      const EstimatorInterface &estimator) const;

  const CommonData &common_data_;

  VarianceHandler variance_handler_;

  double amb_constraint_variance_;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_PROCESS_MODEL_H
