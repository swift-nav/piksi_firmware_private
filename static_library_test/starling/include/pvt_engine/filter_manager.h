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

#include <cmath>

#include <starling/observation.h>
#include <starling/starling.h>

#include <pvt_engine/ambiguity_lambda_interface.h>
#include <pvt_engine/common_data.h>
#include <pvt_engine/configuration.h>
#include <pvt_engine/ephemeris_handler.h>
#include <pvt_engine/filter.h>
#include <pvt_engine/finity.h>
#include <pvt_engine/observation_utils.h>
#include <pvt_engine/optional.h>
#include <pvt_engine/preprocessing_insight.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/pvt_types.h>
#include <pvt_engine/ssr_corrections.h>
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
  Eigen::Vector3d baseline_;
  Eigen::Matrix3d baseline_covariance_;
  optional<Eigen::Vector3d> velocity_;
  optional<Eigen::Matrix3d> velocity_covariance_;
  bool is_fixed_;
  s32 num_signals_;
  s32 num_satellites_;
  optional<Eigen::Vector3d> known_reference_location_;
  double propagation_time_;
  gps_time_t time;
  optional<ProtectionLevelResult> protection_level_;

  FilterResult()
      : position_ecef_(),
        baseline_(),
        baseline_covariance_(),
        velocity_(),
        velocity_covariance_(),
        is_fixed_(),
        num_signals_(),
        num_satellites_(),
        known_reference_location_(),
        propagation_time_(),
        time(),
        protection_level_(){};
};

class FilterManager {
 public:
  virtual FILTER_MANAGER_TYPE get_filter_mode() const = 0;

  virtual ~FilterManager() = default;

  FilterManager &operator=(const FilterManager &other);

  PRC initialize(const CommonData::ResetBasePosition &reset_base_position =
                     CommonData::ResetBasePosition::ResetNeeded);

  void partial_reinitialization();

  virtual PRC update(FilterObservationIdSet *obs_to_drop,
                     bool *reset_downstream_filter) = 0;

  virtual PRC get_filter_result(FilterResult *filter_result,
                                const PROCESSING_MODE &mode) const = 0;

  virtual ambiguities::TransformedIntegerAmbiguities get_fixed_signals()
      const = 0;

  s32 get_num_signals() const;

  FilterObservationIdSet get_signals() const;

  struct Insight {
    FilterUpdateInsights float_filter_update;
    ambiguities::ValidationInsight ambiguity_validation;
    PreprocessingInsights preprocessing_insights;
    Insight(const FilterUpdateInsights &float_filter,
            const ambiguities::ValidationInsight &amb,
            const PreprocessingInsights &preprocessing_insights_)
        : float_filter_update(float_filter),
          ambiguity_validation(amb),
          preprocessing_insights(preprocessing_insights_) {}
  };
  virtual Insight get_insight() const = 0;

  DOPS get_dop_values() const;

  optional<FilterState> get_continuous_state() const;

  virtual optional<FilterState> get_fixed_continuous_state() const {
    return {};
  };

  virtual void update_iono_params(const ionosphere_t &new_iono_params,
                                  const bool disable_klobuchar);

  PRC set_configuration(const MasterConfiguration &config);

  MasterConfiguration get_configuration() const;

  bool is_initialized() const;

  virtual void overwrite_ephemerides(s16 num_ephs,
                                     const ephemeris_t *stored_ephs[]) = 0;

  PRC process_sbas_message(const SBASRawData &message);

  virtual void overwrite_sbas_corrections_manager(
      const SBASCorrectionsManager &other);

  bool is_sbas_used() const;

  bool has_sbas_corrections(const gnss_signal_t &sid,
                            const gps_time_t &epoch_time, const u8 &IODE) const;

  virtual PRC add_rover_observations(obss_t *obs_sol) = 0;

  virtual void overwrite_precise_corrections(
      const SatelliteCorrectionsHandler &ssr_corrections);

  void set_apriori_position(const Eigen::Vector3d &apriori_position) {
    common_data_.set_apriori_position(apriori_position);
  }

  PRC retrieve_apriori_position(obss_t *obs_sol);

  virtual optional<double> get_percentage_of_fixed_ambiguities() const;

  optional<Eigen::Vector3d> get_apriori_position() const {
    return common_data_.get_apriori_position();
  }

 protected:
  explicit FilterManager(const MasterConfiguration &config);

  virtual PRC initialize_internal() = 0;

  virtual void partial_reinitialization_internal() = 0;

  bool velocity_initialized(
      const MatrixMaxStateDimd_t &baseline_covariance) const;

  virtual const Filter *get_output_filter() const = 0;

  virtual bool update_config() = 0;

  PRC check_update_delta_time(const gps_time_t &obs_time);

  bool is_velocity_update_time_acceptable(const Filter &output_filter) const;

  MasterConfiguration configuration_;
  CommonData common_data_;
  optional<gps_time_t> last_update_time_;
  bool initialized_;
  pvt_common::containers::Set<gps_time_t, NUMBER_OF_PARTIAL_RESET_TIMES_STORED>
      last_partial_resets_;
};

};  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_FILTER_MANAGER_H
