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

#ifndef LIBSWIFTNAV_PVT_ENGINE_COMMON_DATA_H
#define LIBSWIFTNAV_PVT_ENGINE_COMMON_DATA_H

#include <pvt_engine/azimuth_elevation.h>
#include <pvt_engine/base_glonass_biases_handler.h>
#include <pvt_engine/base_position_handler.h>
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/optional.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/pvt_types.h>
#include <pvt_engine/sbas/sbas.h>
#include <pvt_engine/sbas/sbas_corrections_manager.h>
#include <swiftnav/glonass_phase_biases.h>
#include <swiftnav/ionosphere.h>

namespace pvt_engine {

struct NetworkCorrectionStd {
  NetworkCorrectionStd() : iono_std_(), tropo_std_(), range_std_() {}
  NetworkCorrectionStd(const optional<double> &iono_std,
                       const optional<double> &tropo_std,
                       const optional<double> &range_std)
      : iono_std_(iono_std), tropo_std_(tropo_std), range_std_(range_std) {}
  optional<double> iono_std_;
  optional<double> tropo_std_;
  optional<double> range_std_;
};

using NetworkStdMap =
    pvt_common::containers::Map<gnss_signal_t, NetworkCorrectionStd,
                                cMaxInputTrackingChannels>;

struct IonosphereData {
  IonosphereData();
  bool operator==(const IonosphereData &rhs) const;

  ionosphere_t parameters;
  bool valid;
};

class CommonData {
 public:
  enum class ResetBasePosition : bool { ResetNeeded, NoResetNeeded };

  explicit CommonData(const BasePositionConfiguration &base_pos_config);
  bool operator==(const CommonData &rhs) const;

  void update_config(const BasePositionConfiguration &config);

  void initialize(const ResetBasePosition &reset_base_position);

  BasePositionHandler::ResetFilter set_updated_reference_position(
      const gps_time_t &obs_time,
      const Eigen::Vector3d &spp_reference_station_position);

  Eigen::Vector3d get_current_reference_position_vector() const;

  optional<BasePosition> get_current_reference_position() const;

  bool is_reset_needed(const gps_time_t &obs_time,
                       const BasePosition &reference_station_position) const;

  void set_known_ref_pos(const Eigen::Vector3d &known_reference_position);

  PRC get_iono_parameters(ionosphere_t *parameters) const;
  void set_iono_parameters(const ionosphere_t &parameters,
                           const bool disable_klobuchar);

  optional<glo_biases_t> get_glonass_biases() const;
  void set_known_glonass_biases(const glo_biases_t &biases);

  bool is_sbas_initialized() const;

  PRC process_sbas_message(const SBASRawData &message);

  void overwrite_sbas_corrections_manager(const SBASCorrectionsManager &other);

  optional<double> get_approximate_baseline_length_km() const;

  optional<SBASCorrections> get_sbas_corrections(
      const gnss_signal_t &sid, const gps_time_t &epoch_time, const u8 &IODE,
      const Eigen::Vector3d &user_pos_ecef, const Eigen::Vector3d &user_pos_llh,
      const Eigen::Vector3d &sat_pos_ecef,
      const AzimuthElevation &sat_az_el) const;

  bool has_sbas_corrections(const gnss_signal_t &sid,
                            const gps_time_t &epoch_time, const u8 &IODE) const;

  void set_apriori_position(const Eigen::Vector3d &apriori_position) {
    apriori_position_ = apriori_position;
  }

  optional<Eigen::Vector3d> get_apriori_position() const {
    return apriori_position_;
  }

  void set_network_std_dev(const NetworkStdMap &std_map);

  const NetworkStdMap &get_std_map() const;

 private:
  IonosphereData iono_;

  BasePositionHandler base_pos_handler;
  GlonassBiases base_glonass_biases_handler;
  optional<SBASCorrectionsManager> sbas_corrections_manager;
  optional<Eigen::Vector3d> apriori_position_;
  NetworkStdMap std_map_;
};
}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_COMMON_DATA_H
