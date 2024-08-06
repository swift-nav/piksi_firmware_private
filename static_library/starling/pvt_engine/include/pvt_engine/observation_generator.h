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

#include <pvt_common/eigen_custom.h>
#include <pvt_common/optional.h>
#include <pvt_engine/apriori_model_handler.h>
#include <pvt_engine/configuration.h>
#include <pvt_engine/line_of_sight.h>
#include <pvt_engine/mean_and_variance.h>
#include <pvt_engine/obss.h>
#include <pvt_engine/satellite_states.h>

namespace pvt_engine {

struct ObsGenFailureInfo {
  s32 num_obs_no_atmo_corrections;
  s32 num_obs_over_variance_threshold;
  PRC apriori_ret_code;
};

using MeanAndVarianceMap =
    pvt_common::containers::Map<ObservationIdentifier, MeanAndVariance,
                                cMaxInputObservations>;

/******************************************************************
 *                  ObservationGenerator Interface                *
 ******************************************************************/

class ObservationGeneratorInterface {
 public:
  virtual ~ObservationGeneratorInterface() = default;

  virtual PRC initialize(bool apriori_model_init) = 0;

  virtual void update_otl_params(const std::vector<double> &params) = 0;

  virtual bool otl_params_loaded() const = 0;

  virtual void update_precise_corrections(
      const SatelliteCorrectionsHandler &new_satellite_corrections_handler) = 0;

  virtual void update_atmospheric_corrections(
      const AtmosphericCorrectionsHandler &new_atmospheric_corrections) = 0;

  virtual PRC add_clock_orbit_apriori_errors(
      InputObservationHandler *obs_handler) = 0;

  virtual optional<obss_t> generate_virtual_base_obs(
      const Eigen::Vector3d &rover_location, const gps_time_t &gps_time,
      ObsGenFailureInfo *failure_info) = 0;

  virtual void update_virtual_base_location(
      const Eigen::Vector3d &new_virtual_base_location) = 0;

  virtual void update_virtual_base_location_with_rover_location(
      const Eigen::Vector3d &rover_location) = 0;

  virtual bool has_virtual_base_location() const = 0;

  virtual optional<Eigen::Vector3d> get_virtual_base_location() const = 0;

  virtual ObservationGeneratorConfiguration get_config() const = 0;

  virtual void add_satellite_apc_data(
      const pvt_engine::SatellitePCVMap &satellite_apc) = 0;

  virtual bool update_config(
      const ObservationGeneratorConfiguration &config) = 0;
};

/******************************************************************
 *               ObservationGenerator Implementation              *
 ******************************************************************/

class ObservationGenerator : public ObservationGeneratorInterface {
 public:
  explicit ObservationGenerator(const ObservationGeneratorConfiguration &config,
                                const SatelliteInformationHandler &sat_info);

  ~ObservationGenerator() override = default;

  PRC initialize(bool apriori_model_init) override;

  void update_otl_params(const std::vector<double> &params) override;

  bool otl_params_loaded() const override;

  void update_precise_corrections(
      const SatelliteCorrectionsHandler &new_satellite_corrections_handler)
      override;

  void update_atmospheric_corrections(const AtmosphericCorrectionsHandler &
                                          new_atmospheric_corrections) override;

  PRC add_clock_orbit_apriori_errors(
      InputObservationHandler *obs_handler) override;

  optional<obss_t> generate_virtual_base_obs(
      const Eigen::Vector3d &rover_location, const gps_time_t &gps_time,
      ObsGenFailureInfo *failure_info) override;

  void update_virtual_base_location(
      const Eigen::Vector3d &new_virtual_base_location) override;

  void update_virtual_base_location_with_rover_location(
      const Eigen::Vector3d &rover_location) override;

  bool has_virtual_base_location() const override;

  optional<Eigen::Vector3d> get_virtual_base_location() const override;

  ObservationGeneratorConfiguration get_config() const override;

  void add_satellite_apc_data(
      const pvt_engine::SatellitePCVMap &satellite_apc) override;

  bool update_config(const ObservationGeneratorConfiguration &config) override;

 private:
  ObservationGeneratorConfiguration config_;

 protected:
  optional<gps_time_t> iono_model_time_;
  SatelliteStatesHandler satellite_states_handler_;
  optional<Eigen::Vector3d> virtual_base_location_;

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
      InputObservationHandler &obs_handler, ObsGenFailureInfo *failure_info);

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

  optional<gps_time_t> previous_iono_model_time_;
  CommonData common_data_;
  CommonAprioriModels common_models_;
  FilterManagerAprioriModels filter_manager_models_;
  AprioriModelHandler apriori_model_handler_;
  ObsId2DoubleMap prev_epoch_iono_corrections_;
  ObsId2DoubleMap prev_epoch_phase_bias_corrections_;
  pvt_common::containers::Map<code_t, double, CODE_COUNT>
      median_iono_correction_offsets_;
  pvt_common::containers::Map<code_t, double, CODE_COUNT>
      phase_bias_correction_offsets_;
  ObsId2DoubleMap curr_epoch_iono_corrections_;
  const SatelliteInformationHandler &satellite_information_handler_;
};

}  // namespace pvt_engine

#endif  // STARLING_PVT_ENGINE_OBSERVATION_GENERATOR_H
