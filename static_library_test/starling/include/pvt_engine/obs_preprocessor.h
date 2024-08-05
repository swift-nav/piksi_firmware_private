/**
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING__PVT_ENGINE_OBS_PREPROCESSOR_H
#define STARLING__PVT_ENGINE_OBS_PREPROCESSOR_H

#include <pvt_common/containers/circular_buffer.h>
#include <pvt_engine/apriori_model_handler.h>
#include <pvt_engine/common_data.h>
#include <pvt_engine/configuration.h>
#include <pvt_engine/precise_satellite_states.h>
#include <pvt_engine/preprocessing_insight.h>
#include <starling/starling.h>

namespace pvt_engine {

class ObsPreprocessor {
 public:
  ObsPreprocessor(CommonData &common_data, const MasterConfiguration &config,
                  const PROCESSING_MODE processing_mode,
                  const FILTER_MANAGER_TYPE &filter_type);

  PRC initialize(const MasterConfiguration &config);
  void reinitialize(const MasterConfiguration &config);

  bool update_config(const MasterConfiguration &config);

  void copy_handlers(const ObsPreprocessor &other);

  void overwrite_ephemerides(s16 num_ephs, const ephemeris_t *stored_ephs[]);
  void fail_if_no_otl_params() const;

  PRC add_rover_observations(
      InputObservationHandler &rover_obs_handler_in,
      const optional<Eigen::Vector3d> &estimated_position,
      FilterObservationHandler *obs_handler_out);

  PRC set_reference_observations(
      InputObservationHandler &new_reference_obs_handler,
      const optional<Eigen::Vector3d> &estimated_rover_position,
      const BasePosition &updated_base_position,
      FilterObservationHandler *obs_handler_out);

  bool is_differencing() const { return differencing_; }

 private:
  PRC make_new_low_latency_sdiffs(
      const optional<Eigen::Vector3d> &estimated_position,
      FilterObservationHandler *obs_handler_out);
  PRC make_new_time_matched_sdiffs(
      const optional<Eigen::Vector3d> &estimated_position,
      FilterObservationHandler *obs_handler_out);
  PRC apply_sdiff_apriori_models(
      InputObservationHandler *sdiff_obs_handler,
      const optional<Eigen::Vector3d> &estimated_position);

  CommonData &common_data_;
  double max_reference_observation_age_;
  const bool differencing_;
  PROCESSING_MODE processing_mode_;

  EphemerisHandler ephemeris_handler_;
  AprioriModelHandler rover_apriori_model_handler_;
  optional<InputObservationHandler> latest_reference_obs_;
  pvt_common::containers::CircularBuffer<InputObservationHandler,
                                         cMaxObsBufferSize>
      rover_observations_;
  AprioriModelHandler reference_apriori_model_handler_;
  AprioriModelHandler sdiff_apriori_model_handler_;
};

}  // namespace pvt_engine

#endif  // STARLING__PVT_ENGINE_OBS_PREPROCESSOR_H
