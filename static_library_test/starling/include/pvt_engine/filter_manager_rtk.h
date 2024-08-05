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

#ifndef LIBSWIFTNAV_PVT_ENGINE_FILTER_MANAGER_RTK_H
#define LIBSWIFTNAV_PVT_ENGINE_FILTER_MANAGER_RTK_H

#include <swiftnav/glonass_phase_biases.h>

#include <pvt_common/containers/circular_buffer.h>
#include <pvt_engine/base_glonass_biases_handler.h>
#include <pvt_engine/base_position_handler.h>
#include <pvt_engine/filter_manager_ppp.h>

namespace pvt_engine {

// FilterManagerRTK is final in order to prevent a derived class from being in a
// different mode (i.e. not RTK_MODE) but having the constructor set up the
// configuration parameters as RTK parameters.
class FilterManagerRTK final : public FilterManagerPPP {
 public:
  explicit FilterManagerRTK(ConfigurationType config_type,
                            const PROCESSING_MODE processing_mode);
  explicit FilterManagerRTK(const MasterConfiguration &config,
                            const PROCESSING_MODE processing_mode);

  // This is also final for the reason that the class is
  FILTER_MANAGER_TYPE get_filter_mode() const final { return RTK_MODE; }

  FilterManagerRTK &operator=(const FilterManagerRTK &other);

  PRC update(FilterObservationIdSet *obs_to_drop_out,
             bool *reset_downstream_filter) override;

  PRC get_filter_result(FilterResult *filter_result,
                        const PROCESSING_MODE &mode) const override;

  PRC set_reference_observations(
      const gps_time_t &obs_time, const u8 num_obs,
      const navigation_measurement_t nav_meas[],
      const pvt_common::containers::Map<gnss_signal_t, measurement_std_t,
                                        pvt_engine::cMaxInputTrackingChannels>
          &observations_std,
      const Eigen::Vector3d &spp_reference_station_position);

  void set_known_ref_pos(const Eigen::Vector3d &known_reference_position);

  void set_known_glonass_biases(const glo_biases_t &biases);

  optional<double> get_percentage_of_fixed_ambiguities() const override;

  bool is_relative_position() const override { return true; }

  void set_config_for_correction_source(const CORRECTION_SOURCE &source);

 private:
  PRC initialize_internal() override;

  void partial_reinitialization_internal() final;

  PRC update_low_latency(FilterObservationIdSet *obs_to_drop_out,
                         bool *reset_downstream_filter);

  const Filter *get_output_filter() const override;

  bool update_config() override;

  void apply_reference_station_state(
      FilterResult *filter_result) const override;

  using FilterManagerPPP::get_filter_result;

  Filter low_latency_filter_;
};

};  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_FILTER_MANAGER_RTK_H
