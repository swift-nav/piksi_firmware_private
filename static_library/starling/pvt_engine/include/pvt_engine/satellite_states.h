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
#ifndef STARLING_PVT_ENGINE_SATELLITE_STATES_H
#define STARLING_PVT_ENGINE_SATELLITE_STATES_H
#include <swiftnav/ephemeris.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/linear_algebra.h>
#include <swiftnav/nav_meas.h>

#include <pvt_engine/ephemeris_handler.h>
#include <pvt_engine/frequency_manager.h>
#include <pvt_engine/observation_handler.h>
#include <pvt_engine/pvt_types.h>
#include <pvt_engine/ssr_corrections.h>
#include <bitset>

namespace pvt_engine {

/**
 * Flags indicating which correction has been applied to compute a given
 * satellite state
 */
enum CORRECTION_APPLICATION_BITS {
  BROADCAST_EPH_AVAILABLE = 0,
  SSR_ORBIT_AVAILABLE,
  SSR_CLK_AVAILABLE,
  SSR_CODE_BIASES_AVAILABLE,
  SSR_PHASE_BIASES_AVAILABLE,
  NUMBER_OF_CORRECTION_APPLICATION_BITS
};

/**
 * Structure containing the states of a given satellite
 * Every element of the state is in meters
 */
struct SatelliteComputedStates {
  // Ephemeris key for the broadcast ephemeris used
  std::experimental::optional<EphemerisKey> ephemeris_key;

  // State reference time (satellite signal emission)
  std::experimental::optional<gps_time_t> time;

  // Satellite L1/L2 iono-free phase center position (ECEF)
  std::experimental::optional<Eigen::Vector3d> phase_center_position_meters;

  // Satellite L1/L2 iono-free phase center velocity (ECEF)
  std::experimental::optional<Eigen::Vector3d> velocity_meters_seconds;

  // Satellite clock (P1/P2 iono-free, meters)
  std::experimental::optional<double> clock_offset_meters;

  // Satellite clock rate error (P1/P2 iono-free, meters/seconds)
  std::experimental::optional<double> clock_rate_meters_seconds;

  // Satellite code biases map (meters)
  std::experimental::optional<CodeBiasSignalMap> code_biases;

  // Satellite phase biases map (meters)
  std::experimental::optional<PhaseBiasSignalMap> phase_biases;

  // Satellite yaw angle (radians)
  std::experimental::optional<double> satellite_yaw_angle_radians;

  // Corrections applied to obtain the states
  std::bitset<NUMBER_OF_CORRECTION_APPLICATION_BITS> corrections_available;

  bool operator==(const SatelliteComputedStates &other) const;
  bool operator!=(const SatelliteComputedStates &other) const;
  void reset(void);
  SatelliteComputedStates();
};

/**
 * Class containing a satellite map
 */
class SatelliteStatesHandler {
 public:
  explicit SatelliteStatesHandler(const SatelliteInformationHandler &sat_info);
  explicit SatelliteStatesHandler(
      const ConstellationBoolMap &expect_phase_biases,
      const SatelliteInformationHandler &sat_info);
  bool update_config();
  bool update_config(const ConstellationBoolMap &expect_phase_biases);
  void get_sid_states(const CorrectionKey &correction_key,
                      const gnss_signal_t &sid, const gps_time_t &time,
                      SatelliteComputedStates *states) const;
  void get_sid_states_at_time_of_transmission(
      const CorrectionKey &correction_key, const gnss_signal_t &sid,
      const gps_time_t &time_of_reception,
      const Eigen::Vector3d &reception_location,
      SatelliteComputedStates *states) const;
  void set_sat_corrections_handler(
      const SatelliteCorrectionsHandler &satellite_corrections_handler) {
    satellite_corrections_handler_ = satellite_corrections_handler;
  };

  PRC compute_ssr_corrections(
      InputObservationHandler *obs_handler,
      InputObservationIdSet *obs_without_SSR_corrections,
      ObsId2DoubleMap *phase_biases) const;
  PRC apply_corrections(InputObservationHandler *obs_handler) const;
  void clear();
  SatIdSet get_sats_with_corrections() const;
  pvt_common::containers::Map<constellation_t, gps_time_t, CONSTELLATION_COUNT>
  get_best_correction_times(const gps_time_t &time,
                            const SatIdSet &sats_used) const;
  bool are_integer_precise_corrections_available(
      const CorrectionKey &correction_key, const gnss_signal_t &sid,
      const gps_time_t &time) const;
  bool are_float_precise_corrections_available(
      const CorrectionKey &correction_key, const gnss_signal_t &sid,
      const gps_time_t &time) const;
  bool is_phase_bias_expected(const constellation_t &constellation) const;

  static void rsw_to_xyz(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel,
                         const Eigen::Vector3d &rsw, Eigen::Vector3d *xyz);
  SatelliteStatesHandler &operator=(const SatelliteStatesHandler &rhs);

 private:
  SatelliteCorrectionsHandler satellite_corrections_handler_;
  BiasReference bias_reference_;
  ConstellationBoolMap expect_phase_biases_;
  const SatelliteInformationHandler &satellite_information_handler_;
  void apply_ssr_delta_corrections(const CorrectionKey &correction_key,
                                   const gnss_signal_t &sid,
                                   const gps_time_t &time,
                                   SatelliteComputedStates *states) const;

  void compute_corrections(const CorrectionKey &correction_key,
                           const gnss_signal_t &sid, const gps_time_t &time,
                           SatelliteComputedStates *states) const;
  void compute_integer_sid_phase_biases(const CorrectionKey &correction_key,
                                        const gnss_signal_t &sid,
                                        const gps_time_t &time,
                                        SatelliteComputedStates *states) const;
  gps_time_t update_time_of_transmission(
      const gps_time_t &obs_time_of_transmission,
      const gnss_signal_t &sid) const;
};
bool test_integerness_phase_biases(const PhaseBiases &biases);
double get_delta_clock_correction(const gps_time_t &time,
                                  const ClockCorrection &clock_correction);

}  // namespace pvt_engine

#endif  // STARLING_PVT_ENGINE_SATELLITE_STATES_H
