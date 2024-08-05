/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_PVT_ENGINE_OBSERVATION_GENERATOR_H
#define STARLING_PVT_ENGINE_OBSERVATION_GENERATOR_H

#include <swiftnav/gnss_time.h>

#include <starling/starling.h>

#include <pvt_common/eigen_custom.h>
#include <pvt_engine/apriori_model_handler.h>
#include <pvt_engine/configuration.h>
#include <pvt_engine/line_of_sight.h>
#include <pvt_engine/mean_and_variance.h>
#include <pvt_engine/optional.h>
#include <pvt_engine/precise_satellite_states.h>

namespace pvt_engine {

using MeanAndVarianceMap =
    pvt_common::containers::Map<ObservationIdentifier, MeanAndVariance,
                                cMaxInputObservations>;

class ObservationGenerator {
 public:
  explicit ObservationGenerator(
      const ObservationGeneratorConfiguration &config);
  virtual ~ObservationGenerator() = default;

  virtual void initialize_internal() = 0;

  PRC initialize();

  void update_otl_params(const std::vector<double> &params);

  bool otl_params_loaded() const;

  void update_precise_corrections(
      const SatelliteCorrectionsHandler &new_satellite_corrections_handler);

  PRC add_clock_orbit_apriori_errors(InputObservationHandler &obs_handler);

  optional<obss_t> generate_virtual_base_obs(
      const Eigen::Vector3d &rover_location, const gps_time_t &gps_time);

  optional<obs_array_t> generate_virtual_base_obs_as_obs_array_t(
      const Eigen::Vector3d &rover_location, const gps_time_t &gps_time);

  void update_virtual_base_location(
      const Eigen::Vector3d &new_virtual_base_location);

  void update_virtual_base_location_with_rover_location(
      const Eigen::Vector3d &rover_location);

  bool has_virtual_base_location() const;

  optional<Eigen::Vector3d> get_virtual_base_location() const;

 private:
  ObservationGeneratorConfiguration config_;

 protected:
  optional<gps_time_t> iono_model_time_;
  SatelliteStatesHandler satellite_states_handler_;
  optional<Eigen::Vector3d> virtual_base_location_;

  bool update_config(const ObservationGeneratorConfiguration &config);

  SatIdSet get_sats_to_create_virtual_obs_for(
      const CORRECTION_TYPE &correction_type,
      const pvt_common::containers::Map<constellation_t, gps_time_t,
                                        CONSTELLATION_COUNT>
          &best_correction_times,
      const Eigen::Vector3d &rover_location, const gps_time_t &gps_time) const;

 private:
  u8 obs_handler_to_nav_meas(const InputObservationHandler &obs_handler,
                             navigation_measurement_t *nav_meas,
                             measurement_std_t *meas_std) const;

  optional<CorrectionKey> get_correction_key(
      const CORRECTION_TYPE &correction_type, const constellation_t &constel,
      const pvt_common::containers::Map<constellation_t, gps_time_t,
                                        CONSTELLATION_COUNT>
          &best_correction_times) const;

  InputObservationHandler get_delay_free_observation_handler(
      const CORRECTION_TYPE &correction_type,
      const pvt_common::containers::Map<constellation_t, gps_time_t,
                                        CONSTELLATION_COUNT>
          &best_correction_times,
      const gps_time_t &gps_time, const SatIdSet sats) const;

  virtual std::pair<MeanAndVarianceMap, bool> compute_iono_delays(
      const Eigen::Vector3d &rover_ecef, const gps_time_t &gps_time,
      const InputObservationHandler &obs_handler) = 0;

  virtual std::pair<MeanAndVarianceMap, bool> compute_tropo_delays(
      const Eigen::Vector3d &rover_ecef, const gps_time_t &gps_time,
      const InputObservationHandler &obs_handler) = 0;

  optional<obss_t> add_delays_to_observations(
      const Eigen::Vector3d &rover_ecef, const gps_time_t &gps_time,
      InputObservationHandler &obs_handler);

  pvt_common::containers::Map<constellation_t, gps_time_t, CONSTELLATION_COUNT>
  get_best_correction_times(const Eigen::Vector3d &rover_location,
                            const gps_time_t &gps_time) const;

  void get_median_correction_deltas(
      const InputObservationHandler &obs_handler,
      const ObsId2DoubleMap &curr_epoch_corrections,
      const ObsId2DoubleMap &prev_epoch_corrections,
      pvt_common::containers::Map<code_t, double, CODE_COUNT> *median_deltas);

  void adjust_correction_offsets(
      const bool &apply_current_epoch_offset,
      const pvt_common::containers::Map<code_t, double, CODE_COUNT> &deltas,
      const double &max_change,
      pvt_common::containers::Map<code_t, double, CODE_COUNT>
          *correction_offsets);

  void obss_t_to_obs_array_t(const obss_t &obs_in, obs_array_t *obs_out) const;

  optional<gps_time_t> previous_iono_model_time_;
  CommonData common_data_;
  AprioriModelHandler apriori_model_handler_;
  ObsId2DoubleMap prev_epoch_iono_corrections_;
  ObsId2DoubleMap prev_epoch_phase_bias_corrections_;
  pvt_common::containers::Map<code_t, double, CODE_COUNT>
      median_iono_correction_offsets_;
  pvt_common::containers::Map<code_t, double, CODE_COUNT>
      phase_bias_correction_offsets_;
  ObsId2DoubleMap curr_epoch_iono_corrections_;
};

}  // namespace pvt_engine

#endif  // STARLING_PVT_ENGINE_OBSERVATION_GENERATOR_H
