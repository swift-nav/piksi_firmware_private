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
#ifndef LIBSWIFTNAV_PVT_ENGINE_FILTER_UPDATE_INSIGHT_H
#define LIBSWIFTNAV_PVT_ENGINE_FILTER_UPDATE_INSIGHT_H

#include <pvt_common/containers/map.h>
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/gaussian.h>
#include <pvt_engine/observation.h>
#include <pvt_engine/outlier_info.h>
#include <pvt_engine/update_info.h>
#include <starling/build/config.h>
#include <swiftnav/macros.h>

namespace pvt_engine {

/**
 * Stores information about what happened in the filter on a particular update.
 */
class FilterUpdateInsights {
 public:
  FilterUpdateInsights()
      : observations_(),
        updates_(),
        outliers_(),
        degrees_of_freedom_(0),
        filter_reset_due_to_outliers_(false),
        filter_reset_due_to_mde_(false) {}
  void new_observation(const ObservationIdentifier &obs_id,
                       const LabeledObservation &lobs) {
    observations_.insert(obs_id, lobs);
  }

  void new_observations(const pvt_common::containers::Map<
                        ObservationIdentifier, LabeledObservation,
                        cMaxFilterObservations> &new_map) {
    observations_.extend_on(choose_right, new_map);
  }

  void new_update(const ObservationIdentifier &obs_id,
                  const UpdateInfo &update_info) {
    if (obs_id.obs_type == BASELINE_MAGNITUDE) {
      return;
    }
    // We should never update the filter with an observation we
    // haven't recorded
    assert(observations_.contains(obs_id));
    // We should never insert a duplicate filter update for an
    // observation
    assert(!updates_.contains(obs_id));
    updates_.insert(obs_id, update_info);
  }

  void set_update_residuals(
      const pvt_common::containers::StaticVector<
          ObservationIdentifier, cMaxFilterObservations> &obs_ids,
      const pvt_common::containers::StaticVector<
          Gaussian, cMaxFilterObservations> &residual_distributions) {
    set_residuals(obs_ids, residual_distributions, &updates_);
  }

  void set_outlier_residuals(
      const pvt_common::containers::StaticVector<
          ObservationIdentifier, cMaxFilterObservations> &obs_ids,
      const pvt_common::containers::StaticVector<
          Gaussian, cMaxFilterObservations> &residual_distributions) {
    set_residuals(obs_ids, residual_distributions, &outliers_);
  }

  void new_outlier(const ObservationIdentifier &obs_id,
                   const OutlierInfo &outlier_info) {
    // We should never detect an outlier in a measurement we haven't
    // already used to update the filter.
    assert(updates_.contains(obs_id));
    updates_.drop(obs_id);
    outliers_.insert(obs_id, outlier_info);
  }

  void redo_update(const ObservationIdentifier &obs_id,
                   const UpdateInfo &update_info) {
    assert(outliers_.contains(obs_id));
    assert(!updates_.contains(obs_id));
    updates_.insert(obs_id, update_info);
  }

  void set_degrees_of_freedom(double dof) { degrees_of_freedom_ = dof; }

  void set_filter_reset_due_to_outliers() {
    filter_reset_due_to_outliers_ = true;
  }

  void set_filter_reset_due_to_mde() { filter_reset_due_to_mde_ = true; }

  using ObservationMap =
      pvt_common::containers::Map<ObservationIdentifier, LabeledObservation,
                                  cMaxFilterObservations>;
  ObservationMap &get_observations() { return observations_; }
  const ObservationMap &get_observations() const { return observations_; }
  using UpdateMap =
      pvt_common::containers::Map<ObservationIdentifier, UpdateInfo,
                                  cMaxFilterObservations>;
  UpdateMap &get_updates() { return updates_; }
  const UpdateMap &get_updates() const { return updates_; }
  using OutlierMap =
      pvt_common::containers::Map<ObservationIdentifier, OutlierInfo,
                                  cMaxFilterObservations>;
  OutlierMap &get_outliers() { return outliers_; }
  const OutlierMap &get_outliers() const { return outliers_; }

  double get_degrees_of_freedom() const { return degrees_of_freedom_; }

  bool get_filter_reset_due_to_outliers() const {
    return filter_reset_due_to_outliers_;
  }

  bool get_filter_reset_due_to_mde() const { return filter_reset_due_to_mde_; }

  pvt_common::containers::Map<SatIdentifier, double, cNumSat> get_sat_elev_map()
      const {
    pvt_common::containers::Map<SatIdentifier, double, cNumSat> sat_elev_map;
    for (const auto &ob : observations_) {
      sat_elev_map.insert(SatIdentifier(ob.key.sid), ob.value.sat_elevation);
    }
    return sat_elev_map;
  }

 private:
  static LabeledObservation choose_right(
      const ObservationIdentifier &id SWIFT_ATTR_UNUSED,
      const LabeledObservation &left_value SWIFT_ATTR_UNUSED,
      const LabeledObservation &right_value) {
    return right_value;
  }

  template <typename T>
  void set_residuals(
      const pvt_common::containers::StaticVector<
          ObservationIdentifier, cMaxFilterObservations> &obs_ids,
      const pvt_common::containers::StaticVector<
          Gaussian, cMaxFilterObservations> &residual_distributions,
      pvt_common::containers::Map<ObservationIdentifier, T,
                                  cMaxFilterObservations> *info_map) {
    assert(obs_ids.size() == residual_distributions.size());
    const pvt_common::containers::Map<ObservationIdentifier, Gaussian,
                                      cMaxFilterObservations>
        residuals(obs_ids.begin(), obs_ids.end(),
                  residual_distributions.begin());

    const auto merge_residuals =
        [](const ObservationIdentifier &k SWIFT_ATTR_UNUSED, const T &info,
           const Gaussian &res) -> optional<T> {
      T new_info(info);
      new_info.residual = res;
      return new_info;
    };

    info_map->template intersect_on<Gaussian, cMaxFilterObservations>(
        merge_residuals, residuals);
  }

  ObservationMap observations_;
  UpdateMap updates_;
  OutlierMap outliers_;
  double degrees_of_freedom_;
  bool filter_reset_due_to_outliers_;
  bool filter_reset_due_to_mde_;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_FILTER_UPDATE_INSIGHT_H
