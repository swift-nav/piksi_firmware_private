/*
 * Copyright (C) 2021 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_FILTER_MANAGER_INTERFACE_H
#define STARLING_FILTER_MANAGER_INTERFACE_H

#include <pvt_engine/ambiguity_set.h>
#include <pvt_engine/preprocessing_insight.h>
#include <pvt_engine/ssr_corrections.h>

namespace pvt_engine {

class FilterManagerInterface {
 public:
  virtual ~FilterManagerInterface() = default;

  virtual FILTER_MANAGER_TYPE get_filter_mode() const = 0;

  virtual PRC initialize(
      const CommonData::ResetBasePosition &reset_base_position) = 0;
  virtual void partial_reinitialization() = 0;
  virtual ambiguities::TransformedIntegerAmbiguities get_fixed_signals()
      const = 0;
  virtual s32 get_num_signals() const = 0;
  virtual FilterObservationIdSet get_signals() const = 0;
  struct Insight {
    FilterUpdateInsights float_filter_update;
    ambiguities::ValidationInsight ambiguity_validation;
    PreprocessingInsights preprocessing_insights;
    OutlierDetectionInsight outlier_detection_insights;
    Insight(const FilterUpdateInsights &float_filter,
            const ambiguities::ValidationInsight &amb,
            const PreprocessingInsights &preprocessing_insights_,
            const OutlierDetectionInsight &outlier_detection_insights_)
        : float_filter_update(float_filter),
          ambiguity_validation(amb),
          preprocessing_insights(preprocessing_insights_),
          outlier_detection_insights(outlier_detection_insights_) {}
  };
  virtual Insight get_insight() const = 0;
  virtual DOPS get_dop_values() const = 0;
  virtual optional<FilterState> get_continuous_state() const = 0;
  virtual optional<FilterState> get_fixed_continuous_state(
      ambiguities::SidSet *unexpected_codes SWIFT_ATTR_UNUSED) const = 0;
  virtual void update_iono_params(const ionosphere_t &new_iono_params,
                                  const bool disable_klobuchar) = 0;
  virtual PRC set_configuration(const FilterManagerConfiguration &config) = 0;
  virtual FilterManagerConfiguration get_configuration() const = 0;
  virtual std::string get_stn_code() const = 0;
  virtual bool is_initialized() const = 0;
  virtual void overwrite_sbas_manager(
      const SBASCorrectionsManager *sbas_corrections_manager) = 0;
  virtual bool is_sbas_used() const = 0;
  virtual bool has_sbas_corrections(const gnss_signal_t &sid,
                                    const gps_time_t &epoch_time,
                                    const u16 &IODE) const = 0;
  virtual void overwrite_precise_corrections(
      const SatelliteCorrectionsHandler &ssr_corrections) = 0;
  virtual void set_apriori_position(
      const Eigen::Vector3d &apriori_position) = 0;
  virtual PRC retrieve_apriori_position(obss_t *obs_sol) = 0;
  virtual pvt_common::containers::Map<constellation_t, double,
                                      CONSTELLATION_COUNT>
  get_proportion_of_fixed_ambiguities() const = 0;
  virtual CombinedAmbiguityVector get_transformed_float_ambiguities() const = 0;
  virtual optional<Eigen::Vector3d> get_apriori_position() const = 0;
  virtual bool time_matched_klobuchar_disabled() const = 0;
  virtual optional<gps_time_t> get_last_update_time() const = 0;
  virtual void set_position_prior(const PositionPrior &new_prior) = 0;
  virtual void clear_position_prior() = 0;
};

}  // namespace pvt_engine

#endif  // STARLING_FILTER_MANAGER_INTERFACE_H
