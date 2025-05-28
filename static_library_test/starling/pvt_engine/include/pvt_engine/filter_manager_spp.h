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

#ifndef LIBSWIFTNAV_PVT_ENGINE_FILTER_MANAGER_SPP_H
#define LIBSWIFTNAV_PVT_ENGINE_FILTER_MANAGER_SPP_H

#include <pvt_common/enable_variable.h>

#include <pvt_engine/filter_manager.h>
#include <pvt_engine/obs_preprocessor.h>

#include <starling/build/config.h>

namespace pvt_engine {

class InstantaneousVelocityFilterManager {
 public:
  InstantaneousVelocityFilterManager(
      const FilterManagerConfiguration &configuration,
      const CommonData &common_data,
      const CommonAprioriModelConfiguration &common_apriori_model_config,
      CommonAprioriModels &common_apriori_models);
  void initialize(
      const CommonAprioriModelConfiguration &common_apriori_model_config);
  void partial_reinitialization();
  bool update_config(const FilterManagerConfiguration &config);
  PRC get_position_estimate(VectorMaxKinematicStatesd_t *position,
                            MatrixMaxKinematicStatesd_t *covariance) const;
  optional<gps_time_t> get_last_update_time() const;
  PRC process_observations(const gps_time_t obs_time, const u8 num_obs,
                           const navigation_measurement_t *nav_meas,
                           const optional<Eigen::Vector3d> &apriori_position,
                           const obs_filters::CodeSet &codes_to_fix);
  double get_instantaneous_velocity_variance_scale_factor() const;

 private:
  FilterManagerConfiguration instantaneous_velocity_configuration_;
  FilterWithoutPosition instantaneous_velocity_filter_;
  FilterManagerAprioriModels instantaneous_velocity_apriori_models_;
  AprioriModelHandler instantaneous_velocity_apriori_model_handler_;
};

// FilterManagerSPP is final in order to prevent a derived class from being in a
// different mode (i.e. not SPP_MODE) but having the constructor set up the
// configuration parameters as SPP parameters.
class FilterManagerSPP final : public FilterManager {
 public:
  // Note - common_apriori_model_config must be persistent as a reference to it
  // is stored
  explicit FilterManagerSPP(
      const FilterManagerConfiguration &filter_manager_config,
      const CommonAprioriModelConfiguration &common_apriori_model_config,
      EphemerisHandlerInterface &eph_handler,
      CommonAprioriModels &common_apriori_models);

  // This is also final for the reason that the class is
  FILTER_MANAGER_TYPE get_filter_mode() const final { return SPP_MODE; }

  FilterManagerSPP &operator=(const FilterManagerSPP &other);

  ambiguities::TransformedIntegerAmbiguities get_fixed_signals() const override;

  Insight get_insight() const override;

  bool update_config(
      const FilterManagerConfiguration &new_configuration) override;

  PRC update(obss_t *obs_sol, FilterObservationIdSet *obs_to_drop_out,
             bool *reset_downstream_filter, FilterResult *filter_result);

 private:
  PRC initialize_internal() override;
  void partial_reinitialization_internal() final;

  void get_instantaneous_velocity(FilterResult *filter_result) const;
  void get_position_average_velocity(FilterResult *filter_result) const;

  PRC process_observations(const gps_time_t obs_time, const u8 num_obs,
                           const navigation_measurement_t *nav_meas,
                           bool *reset_downstream_filter);

  PRC get_filter_result(bool main_filter_update_succeeded,
                        bool instantaneous_velocity_filter_update_succeeded,
                        FilterResult *filter_result) const;

  ObsPreprocessor obs_preprocessor_;

  pvt_common::EnableVariable<InstantaneousVelocityFilterManager,
                             BUILD_CONFIG_INSTANTANEOUS_VELOCITY>
      instantaneous_velocity_filter_manager_;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_FILTER_MANAGER_SPP_H
