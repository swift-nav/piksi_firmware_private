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

#ifndef LIBSWIFTNAV_PVT_ENGINE_FILTER_MANAGER_H
#define LIBSWIFTNAV_PVT_ENGINE_FILTER_MANAGER_H

#include <swiftnav/ephemeris.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/macros.h>

#include <cmath>

#include <pvt_common/observation_utils.h>
#include <pvt_common/optional.h>
#include <pvt_engine/ambiguity_lambda_interface.h>
#include <pvt_engine/common_data.h>
#include <pvt_engine/configuration.h>
#include <pvt_engine/filter.h>
#include <pvt_engine/filter_manager_interface.h>
#include <pvt_engine/finity.h>
#include <pvt_engine/observation.h>
#include <pvt_engine/obss.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/pvt_types.h>
#include <pvt_engine/validate.h>

constexpr s32 NUMBER_OF_PARTIAL_RESET_TIMES_STORED = 3;

namespace pvt_engine {

struct ProtectionLevelResult {
  Eigen::Vector3d protection_levels_ned;
  Eigen::Vector3d position_ecef;
  TIR tir;

  ProtectionLevelResult()
      : protection_levels_ned({0.0, 0.0, 0.0}),
        position_ecef({0.0, 0.0, 0.0}),
        tir(TIR::SAFE_STATE) {}
};

struct FilterResult {
  optional<Eigen::Vector3d> position_ecef_;
  optional<Eigen::Matrix3d> position_ecef_covariance_;
  optional<Eigen::Vector3d> baseline_;
  optional<Eigen::Matrix3d> baseline_covariance_;
  optional<Eigen::Vector3d> average_velocity_;
  optional<Eigen::Matrix3d> average_velocity_covariance_;
  optional<Eigen::Vector3d> instantaneous_velocity_;
  optional<Eigen::Matrix3d> instantaneous_velocity_covariance_;
  bool is_fixed_;
  s32 num_signals_;
  s32 num_satellites_;
  optional<Eigen::Vector3d> known_reference_location_;
  double propagation_time_;
  gps_time_t time;
  optional<ProtectionLevelResult> protection_level_;

  FilterResult()
      : position_ecef_(),
        position_ecef_covariance_(),
        baseline_(),
        baseline_covariance_(),
        average_velocity_(),
        average_velocity_covariance_(),
        instantaneous_velocity_(),
        instantaneous_velocity_covariance_(),
        is_fixed_(),
        num_signals_(),
        num_satellites_(),
        known_reference_location_(),
        propagation_time_(),
        time(),
        protection_level_(){};
};

class FilterManager : public FilterManagerInterface {
 public:
  ~FilterManager() override = default;

  FilterManager &operator=(const FilterManager &other);

  PRC initialize(
      const CommonData::ResetBasePosition &reset_base_position) override;

  void partial_reinitialization() override;

  s32 get_num_signals() const override;

  FilterObservationIdSet get_signals() const override;

  DOPS get_dop_values() const override;

  optional<FilterState> get_continuous_state() const override;

  optional<FilterState> get_fixed_continuous_state(
      ambiguities::SidSet *unexpected_codes SWIFT_ATTR_UNUSED) const override {
    return {};
  };

  void update_iono_params(const ionosphere_t &new_iono_params,
                          const bool disable_klobuchar) override;

  PRC set_configuration(const FilterManagerConfiguration &config) override;

  FilterManagerConfiguration get_configuration() const override;

  std::string get_stn_code() const override;

  bool is_initialized() const override;

  void overwrite_sbas_manager(
      const SBASCorrectionsManager *sbas_corrections_manager) override;

  bool is_sbas_used() const override;

  bool has_sbas_corrections(const gnss_signal_t &sid,
                            const gps_time_t &epoch_time,
                            const u16 &IODE) const override;

  void overwrite_precise_corrections(
      const SatelliteCorrectionsHandler &ssr_corrections) override;

  void set_apriori_position(const Eigen::Vector3d &apriori_position) override {
    common_data_.set_apriori_position(apriori_position);
  }

  PRC retrieve_apriori_position(obss_t *obs_sol) override;

  pvt_common::containers::Map<constellation_t, double, CONSTELLATION_COUNT>
  get_proportion_of_fixed_ambiguities() const override;

  CombinedAmbiguityVector get_transformed_float_ambiguities() const override;

  optional<Eigen::Vector3d> get_apriori_position() const override {
    return common_data_.get_apriori_position();
  }

  bool time_matched_klobuchar_disabled() const override;

  optional<gps_time_t> get_last_update_time() const override {
    return last_update_time_;
  }

  void set_position_prior(const PositionPrior &new_prior) override {
    filter_.set_position_prior(new_prior);
  }

  void clear_position_prior() override { filter_.clear_position_prior(); }

  void set_min_modelled_baseline_len_km(double value);

 protected:
  // Note - common_apriori_model_config must be persistent as a reference to it
  // is stored
  explicit FilterManager(
      const BasePositionConfiguration &base_pos_config,
      const FilterManagerConfiguration &filter_manager_config,
      const CommonAprioriModelConfiguration &common_apriori_model_config,
      const PROCESSING_MODE processing_mode,
      EphemerisHandlerInterface &eph_handler);

  virtual PRC initialize_internal() = 0;

  virtual void partial_reinitialization_internal() = 0;

  bool average_velocity_initialized(
      const MatrixMaxStateDimd_t &kinematic_state_covariance) const;

  virtual bool update_config(
      const FilterManagerConfiguration &new_configuration);

  PRC check_update_delta_time(const gps_time_t &obs_time);

  bool is_average_velocity_update_time_acceptable(
      const Filter &output_filter) const;

  void incorporate_position_biases(const Eigen::Vector3d &pos_ecef,
                                   Eigen::Matrix3d *covariance) const;

  FilterManagerConfiguration configuration_;
  EphemerisHandlerInterface &ephemeris_handler_;
  CommonData common_data_;
  optional<gps_time_t> last_update_time_;
  bool initialized_;
  pvt_common::containers::Set<gps_time_t, NUMBER_OF_PARTIAL_RESET_TIMES_STORED>
      last_partial_resets_;
  const PROCESSING_MODE processing_mode_;
  FilterWithPosition filter_;

  const CommonAprioriModelConfiguration &common_apriori_model_configuration_;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_FILTER_MANAGER_H
