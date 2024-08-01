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

#ifndef LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_FUNCTION_ROW_H
#define LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_FUNCTION_ROW_H

#include <pvt_engine/configuration.h>
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/observation_model_function.h>
#include <pvt_engine/observation_model_function_linear.h>
#include <pvt_engine/observation_model_function_position_single_differenced.h>
#include <pvt_engine/observation_model_function_position_undifferenced.h>
#include <pvt_engine/optional.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/pvt_types.h>

namespace pvt_engine {
/**
 * This class combines multiple ObservationModelFunctions. It basically
 * acts as one row of the H matrix. This function `h()` maps the entire state
 * vector to an observation.
 */
class ObservationModelFunctionRow {
 public:
  ObservationModelFunctionRow();

  ObservationModelFunctionRow(const ObservationModelFunctionRow &other);

  ObservationModelFunctionRow &operator=(
      const ObservationModelFunctionRow &rhs);

  void initialize(const ObsProcModelTypes &model_types);

  /*
   * For a given model type we set the number of states in that observation
   * model and get a pointer to the underlying ObservationModelFunction
   * so we can set its data appropriately.
   */
  ObservationModelFunction *get_and_set_model_function(
      const MODEL_TYPE &model_type, const s32 &model_length);

  /*
   * The only function which is relevant to the user is the operator() which
   * acts like `h(x)` on the state x.
   */
  double operator()(const VectorMaxStateDimd_t &state) const;

 private:
  struct FilterModel {
    FilterModel() : model_(nullptr), model_type_(), model_length_(0){};

    ObservationModelFunction *model_;
    MODEL_TYPE model_type_;
    s32 model_length_;
  };

  using FilterModels =
      pvt_common::containers::StaticVector<FilterModel, MAX_MODEL_NUM>;

  void setup_models(const ObsProcModelTypes &model_types);

  void setup_models(const FilterModels &models);

  void clear_model_list();

  FilterModel setup_model(const MODEL_TYPE model_type, const s32 model_length);

  optional<FilterModel *> find_model(const MODEL_TYPE &model_type);

  FilterModels models_;

  ObservationModelFunctionPositionSingleDifferenced
      position_model_function_single_differenced_;
  ObservationModelFunctionPositionUndifferenced
      position_model_function_undifferenced_;
  LinearObservationModelFunction decoupled_clock_model_function_;
  LinearObservationModelFunction single_clock_model_function_;
  LinearObservationModelFunction single_clock_per_constellation_model_function_;
  LinearObservationModelFunction hardware_biases_model_function_;
  LinearObservationModelFunction ambiguities_model_function_;
  LinearObservationModelFunction troposphere_model_function_;
  LinearObservationModelFunction ionosphere_model_function_;
  LinearObservationModelFunction phase_windup_model_function_;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_FUNCTION_ROW_H
