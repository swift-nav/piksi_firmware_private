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

#ifndef LIBSWIFTNAV_PVT_ENGINE_FILTER_MANAGER_PPP_H
#define LIBSWIFTNAV_PVT_ENGINE_FILTER_MANAGER_PPP_H

#include <pvt_engine/ambiguity_set.h>
#include <pvt_engine/base_position_handler.h>
#include <pvt_engine/filter_manager.h>
#include <pvt_engine/obs_preprocessor.h>
#include <pvt_engine/preprocessing_insight.h>
#include <pvt_engine/protection_level_manager.h>
#include <pvt_engine/satellite_states.h>
#include <pvt_engine/ssr_corrections.h>
#include <string>

namespace pvt_engine {

class FilterManagerPPP : public FilterManager {
 public:
  // Note - common_apriori_model_config must be persistent as a reference to it
  // is stored
  explicit FilterManagerPPP(
      const FilterManagerConfiguration &filter_manager_config,
      const CommonAprioriModelConfiguration &common_apriori_model_config,
      const PROCESSING_MODE processing_mode,
      const FILTER_MANAGER_TYPE &filter_manager_type,
      EphemerisHandlerInterface &eph_handler,
      CommonAprioriModels &common_apriori_models);

  // This can't be final because it may or may not be derived
  FILTER_MANAGER_TYPE get_filter_mode() const override { return PPP_MODE; }

  FilterManagerPPP &operator=(const FilterManagerPPP &other);

  virtual PRC update(FilterObservationIdSet *obs_to_drop,
                     bool *reset_downstream_filter);

  PRC get_filter_result(FilterResult *filter_result) const;

  optional<FilterState> get_fixed_continuous_state(
      ambiguities::SidSet *unexpected_codes) const override;

  ambiguities::TransformedIntegerAmbiguities get_fixed_signals() const override;

  PRC add_rover_observations(obss_t *obs_sol);

  Insight get_insight() const override;

  void overwrite_precise_corrections(
      const SatelliteCorrectionsHandler &ssr_corrections) override;

  void fail_if_no_otl_params() const;

  pvt_common::containers::Map<constellation_t, double, CONSTELLATION_COUNT>
  get_proportion_of_fixed_ambiguities() const final;

  CombinedAmbiguityVector get_transformed_float_ambiguities() const final;

  PRC load_new_ambiguities(const AmbiguitySet &new_set);
  void update_iono_params(const ionosphere_t &new_iono_params,
                          const bool disable_klobuchar) override;

  optional<Eigen::Vector3d> get_latest_position_estimate() const;

  optional<Eigen::Vector3d> get_latest_average_velocity_estimate() const;

  class FMIntPosition : public ambiguities::IntegerPositionComparison {
   public:
    explicit FMIntPosition(const optional<glo_biases_t> &glo_biases,
                           const Filter &filter)
        : filter_(filter), glo_biases_(glo_biases) {}

   private:
    PRC do_function(const ambiguities::TransformedIntegerAmbiguities &ambs,
                    double *distance) const override;

    const Filter &filter_;
    const optional<glo_biases_t> &glo_biases_;
  };

  FMIntPosition get_fixed_position_comparer() const {
    return FMIntPosition(common_data_.get_glonass_biases(), filter_);
  }

  virtual bool is_relative_position() const { return false; }

 protected:
  Eigen::Vector3d get_estimated_position() const;

  PRC initialize_internal() override;

  void partial_reinitialization_internal() override;

  PRC update_filter(FilterObservationIdSet *obs_to_drop_out,
                    bool *reset_downstream_filter);

  void update_approximate_position();

  bool update_config(
      const FilterManagerConfiguration &new_configuration) override;

  virtual void apply_reference_station_state(FilterResult *filter_result) const;

  PRC compute_ssr_corrections(InputObservationHandler *obs_handler);

  ambiguities::SidSet get_ambiguities_from_obs(
      const FilterObservationIdSet &obs) const;

  void ensure_klobuchar_correctness() const;

  AmbiguitySet ambiguity_set_;
  ObsPreprocessor obs_preprocessor_;

 private:
  FilterState drop_unsuccessfully_fixed_iono_states(
      const FilterState &filter_state,
      const ambiguities::TransformedIntegerAmbiguities &validated_ambiguities,
      ambiguities::SidSet *unexpected_codes) const;

  SatelliteStatesHandler satellite_states_handler_;
  PreprocessingInsights preprocessing_insights_;
  OutlierDetectionInsight outlier_detection_insights_;
  mutable ProtectionLevelManager plm_;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_FILTER_MANAGER_RTK_H
