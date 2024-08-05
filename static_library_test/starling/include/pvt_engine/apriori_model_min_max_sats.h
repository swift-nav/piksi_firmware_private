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

#ifndef LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_MIN_MAX_SATS_H
#define LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_MIN_MAX_SATS_H

#include <pvt_engine/apriori_model_interface.h>
#include <pvt_engine/common_data.h>

static const u8 NUMBER_OF_CARRIER_PHASE_PER_SATELLITE = 2;

namespace pvt_engine {

struct SatSelectionMetric {
  SatIdentifier sat_id_;
  double elevation_;
  double cn0_;
  u8 number_of_available_freq_;
  constellation_t constellation_;

  SatSelectionMetric(const SatIdentifier &sat_id, const double elevation,
                     const double cn0, const u8 number_of_available_freq)
      : sat_id_(sat_id),
        elevation_(elevation),
        cn0_(cn0),
        number_of_available_freq_(number_of_available_freq),
        constellation_(sat_id.constellation_){};

  SatSelectionMetric(const SatIdentifier &sat_id, const double elevation)
      : sat_id_(sat_id),
        elevation_(elevation),
        cn0_(0.0),
        number_of_available_freq_(0),
        constellation_(sat_id.constellation_) {}

  SatSelectionMetric()
      : sat_id_(),
        elevation_(0),
        cn0_(0.0),
        number_of_available_freq_(0),
        constellation_(CONSTELLATION_INVALID){};
};

class MetricLessThan {
 public:
  explicit MetricLessThan(const AprioriModelConfiguration &config)
      : config_(config){};
  bool operator()(const SatSelectionMetric &a,
                  const SatSelectionMetric &b) const;

 private:
  AprioriModelConfiguration config_;
};

class AprioriModelMinMaxSats : public AprioriModelWithConfig {
 public:
  explicit AprioriModelMinMaxSats(const AprioriModelConfiguration &config)
      : AprioriModelWithConfig(config) {}
  PRC process(const optional<Eigen::Vector3d> &station_location,
              InputObservationHandler *observation_handler) override;

 private:
  using SatSelectionMetrics =
      pvt_common::containers::StaticVector<SatSelectionMetric, NUM_SATS>;
  using SatSelectionMetricsMap =
      pvt_common::containers::Map<SatIdentifier, SatSelectionMetric, NUM_SATS>;

  SatSelectionMetrics map_to_static_vector(
      const SatSelectionMetricsMap &sat_selection_metrics_map) const;

  using SatPosMap =
      pvt_common::containers::Map<SatIdentifier, Eigen::Vector3d, NUM_SATS>;

  void drop_satellites(const SatSelectionMetricsMap &sat_selection_metrics_map,
                       const s32 &num_sats_to_drop,
                       InputObservationHandler *observation_handler);

  SatSelectionMetricsMap get_sat_elevations_sorted(
      const InputObservationHandler &observation_handler);

  void get_cn0(const InputObservationHandler &obs_handler,
               SatSelectionMetricsMap *sat_selection_metrics_map) const;

  void get_number_of_df_carrier_phase(
      const InputObservationHandler &obs_handler,
      SatSelectionMetricsMap *sat_selection_metrics_map) const;

  void get_constellation_priority(
      SatSelectionMetricsMap *sat_selection_metrics_map) const;

  SatSelectionMetrics sort_metrics(
      const SatSelectionMetricsMap &sat_selection_metrics_map) const;

  static SatIdSet get_sats_to_drop(
      const SatSelectionMetrics &sat_metrics_sorted,
      const s32 num_sats_to_drop);

  static InputObservationIdSet get_obs_ids_to_drop(
      const InputObservationHandler &obs_handler, const SatIdSet &sats_to_drop);
};

}  // namespace pvt_engine
#endif  // LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_MIN_MAX_SATS_H
