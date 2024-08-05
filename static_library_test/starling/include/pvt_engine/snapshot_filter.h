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

#ifndef LIBSWIFTNAV_PVT_ENGINE_SNAPSHOT_FILTER_H
#define LIBSWIFTNAV_PVT_ENGINE_SNAPSHOT_FILTER_H

#include <pvt_engine/ambiguity_map.h>
#include <pvt_engine/filter.h>
#include <pvt_engine/observation.h>
#include <pvt_engine/observation_handler.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/pvt_types.h>
#include <pvt_engine/snapshot_validate.h>
#include <starling/observation.h>

namespace pvt_engine {

class SnapshotFilter : public Filter {
 public:
  SnapshotFilter(const SnapshotConfiguration &config,
                 const CommonData &common_data,
                 const ambiguities::CodeSet &codes_to_fix);

  void initialize(const SnapshotConfiguration &config,
                  const ambiguities::CodeSet &codes_to_fix);

  bool update_config(const SnapshotConfiguration &config);

  PRC update(const FilterObservationHandler &obs_handler,
             FilterObservationHandler *filtered_obs_handler,
             FilterObservationIdSet *ids_to_reset,
             Filter::ResetFilter *is_reset_needed);

  double get_time_processed() const;

  PRC get_validated_ambiguities(
      const optional<glo_biases_t> &biases,
      ambiguities::TransformedIntegerAmbiguities *validated_ambiguities,
      ambiguities::ValidationResult *insight) const;

 private:
  bool is_fix_possible_on_sid(const gnss_signal_t &sid,
                              const optional<glo_biases_t> biases) const;
  PRC select_ambs_and_apply_glonass_biases(
      const optional<glo_biases_t> &biases,
      const ambiguities::AmbiguitiesAndCovariances &float_ambiguities,
      ambiguities::AmbiguitiesAndCovariances *processed_float_ambiguities)
      const;

  optional<gps_time_t> first_epoch_;
  optional<gps_time_t> last_epoch_;
  ambiguities::SnapshotValidator snapshot_validator_;
  ambiguities::CodeSet codes_to_fix_;
  u16 min_dual_frequency_double_differences_;
  double max_position_variance_to_fix_;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_SNAPSHOT_FILTER_H
