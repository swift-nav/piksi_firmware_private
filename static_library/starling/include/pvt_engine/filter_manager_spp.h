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

#include <pvt_engine/filter_manager.h>
#include <pvt_engine/obs_preprocessor.h>

namespace pvt_engine {

// FilterManagerSPP is final in order to prevent a derived class from being in a
// different mode (i.e. not SPP_MODE) but having the constructor set up the
// configuration parameters as SPP parameters.
class FilterManagerSPP final : public FilterManager {
 public:
  explicit FilterManagerSPP(ConfigurationType config_type);
  explicit FilterManagerSPP(const MasterConfiguration &config);

  // This is also final for the reason that the class is
  FILTER_MANAGER_TYPE get_filter_mode() const final { return SPP_MODE; }

  FilterManagerSPP &operator=(const FilterManagerSPP &other);

  void overwrite_ephemerides(s16 num_ephs,
                             const ephemeris_t *stored_ephs[]) override {
    (void)num_ephs;
    (void)stored_ephs;
  }

  PRC update(FilterObservationIdSet *obs_to_drop_out,
             bool *reset_downstream_filter) override;

  PRC get_filter_result(FilterResult *filter_result,
                        __attribute__((unused))
                        const PROCESSING_MODE &mode) const override;

  ambiguities::TransformedIntegerAmbiguities get_fixed_signals() const override;

  PRC add_rover_observations(obss_t *obs_sol) override;

  Insight get_insight() const override;

 private:
  PRC initialize_internal() override;
  void partial_reinitialization_internal() final;

  PRC get_filter_result(const Filter &output_filter,
                        FilterResult *filter_result) const;

  const Filter *get_output_filter() const override;

  bool update_config() override;

  Filter filter_;
  FilterObservationHandler obs_handler_;
  ObsPreprocessor obs_preprocessor_;
};

};  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_FILTER_MANAGER_SPP_H
