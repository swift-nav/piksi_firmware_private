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

#ifndef LIBSWIFTNAV_PVT_ENGINE_AMBIGUITY_MANAGER_H
#define LIBSWIFTNAV_PVT_ENGINE_AMBIGUITY_MANAGER_H

#include <pvt_engine/ambiguity_map.h>
#include <pvt_engine/ambiguity_set.h>
#include <pvt_engine/cross_validate.h>
#include <pvt_engine/ephemeris_handler.h>
#include <pvt_engine/filter_manager_ppp.h>
#include <pvt_engine/obs_preprocessor.h>
#include <pvt_engine/observation_handler.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/pvt_types.h>
#include <pvt_engine/snapshot_filter.h>

namespace pvt_engine {

class AmbiguityManager final : public AmbiguitySet {
 public:
  // Note - common_apriori_model_config must be persistent as a reference to it
  // is stored
  AmbiguityManager(
      const FilterManagerConfiguration &filter_manager_config,
      const CommonAprioriModelConfiguration &common_apriori_model_config,
      FILTER_MANAGER_TYPE upstream_filter_type,
      EphemerisHandlerInterface &eph_handler,
      CommonAprioriModels &common_apriori_models);

  AmbiguityManager &operator=(const AmbiguityManager &other);

  PRC initialize(const CommonData::ResetBasePosition &reset_base_position =
                     CommonData::ResetBasePosition::ResetNeeded);

  PRC set_configuration(const FilterManagerConfiguration &config);
  FilterManagerConfiguration get_configuration() const;

  // Updates the ambiguity manager with the most recent observations
  // and attempts to validate new integer ambiguities
  //
  // There are two stages to validating a set of integers using the
  // snapshot algorithm.
  //
  // The first validation stage (in snapshot_validate.cc) is passing
  // per-filter tests (r-ratio, drop-1 LAMBDA).  If this "per-filter
  // validation" (or "snapshot validation") succeeds, the snapshot
  // filter is considered to have fixed; the resulting set of integers
  // is stored in a finite history of result sets (the "snapshot
  // history"), and the snapshot filter is reinitialized from scratch.
  // If a maximum number of epochs passes without a fix, the snapshot
  // filter is reinitialized anyway, and the results from this epoch
  // are the empty set.
  //
  // The second stage compares all the historical windows in the
  // snapshot history to each other (in cross_validate.cc) and runs
  // another series of tests (minimum count, float baseline
  // comparison).  If this "cross-validation" succeeds, the integers
  // are ready for use in a fixed solution, and you can get them by
  // calling this function.
  PRC update(const optional<FilterObservationIdSet &> obs_to_drop,
             const ambiguities::IntegerPositionComparison &compare_float_f);

  void set_known_glonass_biases(const glo_biases_t &biases);

  PRC add_rover_observations(obss_t *obs_sol);
  PRC set_reference_observations(
      const gps_time_t &obs_time, const u8 num_obs,
      const navigation_measurement_t nav_meas[],
      const pvt_common::containers::Map<gnss_signal_t, measurement_std_t,
                                        pvt_engine::cMaxInputTrackingChannels>
          &observations_std,
      const Eigen::Vector3d &spp_reference_station_position,
      const optional<Eigen::Vector3d> &approximate_velocity);

  void overwrite_precise_corrections(
      const SatelliteCorrectionsHandler &ssr_corrections) {
    satellite_states_handler_.set_sat_corrections_handler(ssr_corrections);
  };

  void update_iono_params(const ionosphere_t &new_iono_params,
                          const bool disable_klobuchar) {
    common_data_.set_iono_parameters(new_iono_params, disable_klobuchar);
  }

  void set_known_ref_pos(const Eigen::Vector3d &known_reference_position) {
    common_data_.set_known_ref_pos(known_reference_position);
  }

  void update_approximate_position(const Eigen::Vector3d &pos) {
    common_data_.set_apriori_position(pos);
  }

  void update_tm_pos_estimate(const optional<Eigen::Vector3d> &new_estimate) {
    tm_pos_estimate_ = new_estimate;
  }

  s32 refine_ambiguities(const FilterObservationHandler &obs_handler) override;

  // Remove the ambiguities corresponding to the given signal IDs.
  s32 remove_ambiguities(const gps_time_t &time,
                         const ambiguities::SidSet &ambs_to_drop) override;

  pvt_common::containers::Map<constellation_t, u8, CONSTELLATION_COUNT>
  get_number_of_searchable_dual_freq_phase_satellites() const;

  FilterManagerPPP::FMIntPosition get_fixed_position_comparer() const {
    return FilterManagerPPP::FMIntPosition(common_data_.get_glonass_biases(),
                                           snapshot_filter_);
  }

  const FilterObservationHandler &get_obs_handler() const {
    return snapshot_filter_.obs_handler_;
  }
  void fail_if_no_otl_params() const {
    obs_preprocessor_.fail_if_no_otl_params();
  }

  static void remove_non_widelane_consistent_obs(
      InputObservationHandler *new_reference_obs_handler);
  void set_config_for_correction_source(const CORRECTION_SOURCE &source);

  bool is_multipath_temporally_decorrelated(
      const optional<Eigen::Vector3d> &approximate_velocity) const;

  CombinedAmbiguityVector get_transformed_float_ambiguities() const;

  void set_min_modelled_baseline_len_km(double value);

 private:
  bool require_reinit_on_successful_validation() const;

  PRC check_update_delta_time(const gps_time_t &obs_time);

  PRC validate_new_ambiguities(
      const ambiguities::IntegerPositionComparison &compare_to_float_f,
      const ambiguities::TransformedIntegerAmbiguities
          &new_validated_ambiguities,
      ambiguities::AmbiguityCrossValidationResult *insight);

  optional<Eigen::Vector3d> get_latest_position_estimate() const;

  void partial_reinitialization();
  PRC compute_ssr_corrections(InputObservationHandler *obs_handler);
  void set_apriori_position(const Eigen::Vector3d &apriori_position) {
    common_data_.set_apriori_position(apriori_position);
  }
  PRC retrieve_apriori_position(obss_t *obs_sol);

  bool initialized_;
  FilterManagerConfiguration configuration_;
  const CommonAprioriModelConfiguration &common_apriori_model_configuration_;
  CommonData common_data_;
  optional<gps_time_t> last_update_time_;
  pvt_common::containers::Set<gps_time_t, NUMBER_OF_PARTIAL_RESET_TIMES_STORED>
      last_partial_resets_;
  SnapshotFilter snapshot_filter_;
  ambiguities::CrossValidator cross_validator_;
  GlonassBiases base_glonass_biases_handler_;
  ObsPreprocessor obs_preprocessor_;
  optional<Eigen::Vector3d> tm_pos_estimate_;
  SatelliteStatesHandler satellite_states_handler_;
  pvt_common::EnableVariable<ambiguities::AmbiguitiesAndCovariances,
                             BUILD_CONFIG_ENABLE_INSIGHTS>
      previous_float_ambs_;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_AMBIGUITY_MANAGER_H
