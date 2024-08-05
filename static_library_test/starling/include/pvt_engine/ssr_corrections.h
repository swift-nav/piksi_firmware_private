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

#ifndef LIBSWIFTNAV_SSR_CORRECTIONS_H
#define LIBSWIFTNAV_SSR_CORRECTIONS_H

#include <swiftnav/ephemeris.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/linear_algebra.h>

#include <libsbp/ssr.h>
#include <pvt_common/containers/map.h>
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/mean_and_variance.h>
#include <pvt_engine/observation.h>
#include <pvt_engine/sat_identifier.h>
#include <pvt_engine/ssr_grid.h>
#include "configuration.h"

// Maximum RTCM SSR orbit correction validity in seconds
#define MAX_SSR_ORBIT_CORRECTION_VALIDITY_SECONDS 1800

// Maximum RTCM SSR clock correction validity in seconds
#define MAX_SSR_CLOCK_CORRECTION_VALIDITY_SECONDS 30

// Maximum RTCM code bias validity in seconds
#define MAX_CODE_BIAS_VALIDITY_SECONDS 300

// Maximum RTCM phase bias validity in seconds
#define MAX_PHASE_BIAS_VALIDITY_SECONDS 300

// Maximum STEC polynomial corrections validity in seconds
#define MAX_STEC_POLY_CORRECTIONS_VALIDITY_SECONDS 300

// Maximum STEC polynomial corrections validity in seconds
// TODO (Leith): what should these values actually be?
#define MAX_STEC_GRIDDED_ATMO_VALIDITY_SECONDS 300

constexpr double GAL_EPHEMERIS_MAX_CACHE_AGE = 10 * 60;

namespace pvt_engine {

struct OrbitCorrection {
  // issue of data ephemeris (unitless integer)
  u32 iode;

  // issue of data SSR (unitless integer)
  u8 iod_ssr;

  // correction reference time
  gps_time_t correction_time;

  // radial, along-track, and out-of-plane correction components (meters)
  Eigen::Vector3d rao;

  // radial, along-track, and out-of-plane correction rate (meters/second)
  Eigen::Vector3d dot_rao;

  OrbitCorrection();

  bool operator==(const OrbitCorrection &other) const;
  bool operator!=(const OrbitCorrection &other) const;
};

struct ClockCorrection {
  // issue of data ephemeris (unitless integer)
  u32 iode;

  // issue of data SSR (unitless integer)
  u8 iod_ssr;

  // correction reference time
  gps_time_t correction_time;

  // clock correction: 1st coefficient (metres)
  double c0;

  // clock correction: 2nd coefficient (metres/second)
  double c1;

  // clock correction: 3rd coefficient (meters/second^2)
  double c2;

  bool operator==(const ClockCorrection &other) const;
  bool operator!=(const ClockCorrection &other) const;
};

// Tolerance for comparing code phase biases with e.g. == operator
#define CODE_PHASE_BIAS_TOLERANCE 5e-5

class CodeBiasSignalMap
    : public pvt_common::containers::Map<code_t, float, CODE_COUNT> {
 public:
  bool operator==(const CodeBiasSignalMap &other) const;
  bool operator!=(const CodeBiasSignalMap &other) const;
};

struct CodeBiases {
  // issue of data SSR (unitless integer)
  u8 iod_ssr;

  // correction reference time
  gps_time_t correction_time;

  // map of bias values per signal (meters)
  CodeBiasSignalMap biases;

  CodeBiases();

  bool operator==(const CodeBiases &other) const;
  bool operator!=(const CodeBiases &other) const;

  void collapse_code_biases();
};

struct PhaseBias {
  // bias value (meters)
  float value;

  // discontinuity counter (unitless integer)
  u8 discontinuity_value;

  // integer indicator (boolean)
  bool int_indicator;

  // WL-integer indicator (unitless integer)
  u8 wl_int_indicator;

  bool operator==(const PhaseBias &other) const;
  bool operator!=(const PhaseBias &other) const;
};

using PhaseBiasSignalMap =
    pvt_common::containers::Map<code_t, PhaseBias, CODE_COUNT>;

struct PhaseBiases {
  // issue of data SSR (unitless integer)
  u8 iod_ssr;

  // correction reference time
  gps_time_t correction_time;

  // yaw (radians)
  double yaw_angle_radians;

  // dispersive Bias Consistency Indicator (boolean)
  bool dispersive;

  // MW Consistency Indicator (boolean)
  bool mw_consistency;

  // map of bias values per signal
  PhaseBiasSignalMap biases;

  PhaseBiases();

  bool operator==(const PhaseBiases &other) const;
  bool operator!=(const PhaseBiases &other) const;
};

/**
 * Class mapping code and phase biases to a reference signal: valid since phase
 * corrections are not modulation specific
 */
class BiasReference {
 public:
  BiasReference();
  code_t rtcm_signal_to_lsn_code_t(const gnss_signal_t &sid,
                                   const u8 rtcm_code) const;
  code_t phase_bias_code_t_to_lsn_code_t(const code_t &lsn_code) const;

 private:
  pvt_common::containers::Map<u8, code_t, NUM_CODES>
      rtcm_to_glo_reference_phase_;
  pvt_common::containers::Map<u8, code_t, NUM_CODES>
      rtcm_to_gps_reference_phase_;
  pvt_common::containers::Map<u8, code_t, NUM_CODES>
      rtcm_to_gal_reference_phase_;
  pvt_common::containers::Map<u8, code_t, NUM_CODES>
      rtcm_to_bds2_reference_phase_;
  pvt_common::containers::Map<code_t, code_t, NUM_CODES>
      lsn_to_reference_phase_;
};

// Modified DF389 user range accuracy
class SnavUra {
 public:
  SnavUra(u8 c, u8 value);
  SnavUra(double stddev);
  SnavUra(u8 encoded_stddev);
  SnavUra &operator--();
  bool operator==(const SnavUra &other) const;
  bool operator!=(const SnavUra &other) const;
  double stddev_upper_bound() const;
  double stddev_lower_bound() const;
  double stddev() const;
  u8 stddev_encoded() const;

 private:
  static constexpr u8 NUM_BITS_VALUE = 5;
  static constexpr u8 NUM_BITS_CLASS = 3;
  static constexpr u8 MAX_CLASS = (1 << NUM_BITS_CLASS) - 1;
  static constexpr u8 MAX_VALUE = (1 << NUM_BITS_VALUE) - 1;

  u8 class_;
  u8 value_;
};

// can't just have a "using" directive here because then == and !=
// will collide with those in the underlying map
struct StecResidualList {
  pvt_common::containers::Map<pvt_engine::SatIdentifier, MeanAndVariance,
                              NUM_SATS>
      satid_tecu_map;

  StecResidualList();

  bool operator==(const StecResidualList &other) const;
  bool operator!=(const StecResidualList &other) const;
  const pvt_common::containers::MapElement<pvt_engine::SatIdentifier,
                                           MeanAndVariance>
      *begin() const;
  const pvt_common::containers::MapElement<pvt_engine::SatIdentifier,
                                           MeanAndVariance>
      *end() const;
};

using StecResidualLists =
    pvt_common::containers::StaticVector<StecResidualList, MAX_SSR_GRID_POINTS>;
bool operator==(const StecResidualLists &lhs, const StecResidualLists &rhs);
bool operator!=(const StecResidualLists &lhs, const StecResidualLists &rhs);

// meters
struct TropoPoint {
  MeanAndVariance dry_delay;  // also known as hydrostatic delay
  MeanAndVariance wet_delay;

  TropoPoint();

  bool operator==(const TropoPoint &other) const;
  bool operator!=(const TropoPoint &other) const;
};

using TropoList =
    pvt_common::containers::StaticVector<TropoPoint, MAX_SSR_GRID_POINTS>;

struct GriddedAtmo {
  u8 iod_atmo;
  bool has_std;
  gps_time_t correction_time;
  StecResidualLists stec_residuals;
  TropoList tropo;

  GriddedAtmo();

  bool operator==(const GriddedAtmo &other) const;
  bool operator!=(const GriddedAtmo &other) const;
};

struct StecPolynomial {
  float coefficients[4];

  // encoded quality indicator (follows RTCM DF389 but using TECU)
  u8 iono_stddev;

  StecPolynomial();

  bool operator==(const StecPolynomial &other) const;
  bool operator!=(const StecPolynomial &other) const;
};

using StecPolynomialSatMap =
    pvt_common::containers::Map<pvt_engine::SatIdentifier, StecPolynomial,
                                NUM_SATS>;

struct StecPolynomialCorrections {
  u8 iod_atmo;
  gps_time_t correction_time;
  StecPolynomialSatMap satellite_map;

  StecPolynomialCorrections();

  bool operator==(const StecPolynomialCorrections &other) const;
  bool operator!=(const StecPolynomialCorrections &other) const;
};

constexpr s32 NUMBER_OF_SSR_CLOCK_CORRECTIONS_TO_STORE = 6;
constexpr s32 NUMBER_OF_SSR_ORBIT_CORRECTIONS_TO_STORE = 6;
constexpr s32 NUMBER_OF_SSR_CODE_BIAS_CORRECTIONS_TO_STORE = 2;
constexpr s32 NUMBER_OF_SSR_PHASE_BIAS_CORRECTIONS_TO_STORE = 2;
constexpr s32 NUMBER_OF_SSR_STEC_POLY_CORRECTIONS_TO_STORE = 2;
constexpr s32 NUMBER_OF_EPHEMERIDES_CORRECTIONS_TO_STORE = 3;
constexpr s32 MAX_SAVED_GRIDDED_ATMO = 2;

// Provided we don't get SSR corrections at a rate higher than 1Hz (!), the
// maximum number of valid correction times is bound by the clock validity
// period, since that's the correction with the shortest validity
constexpr s32 MAX_NUMBER_OF_CORRECTION_TIMES =
    MAX_SSR_CLOCK_CORRECTION_VALIDITY_SECONDS;

struct CorrectionKey {
  CorrectionKey(const pvt_engine::CORRECTION_TYPE &correction_type,
                const optional<gps_time_t> &correction_time);
  const pvt_engine::CORRECTION_TYPE correction_type_;
  const optional<gps_time_t> correction_time_;
};

template <typename T, s32 size>
struct TimeCorrectionMap {
  TimeCorrectionMap() : storage() {}
  void insert(const gps_time_t &key, const T &value);
  void add(const gps_time_t &key, const T &value);
  optional<const T &> lookup(const gps_time_t &key) const;

  optional<const T &> get_last_in_validity_period(const gps_time_t &key,
                                                  double validity_period) const;
  optional<u32> get_last_iode() const;

  pvt_common::containers::FilteredMapView<gps_time_t, T, gps_time_t, size>
  get_all_in_validity_period(const gps_time_t &key,
                             double validity_period) const;

  pvt_common::containers::Map<gps_time_t, T, size> storage;
};

/**
 * Structure containing all the corrections for a given satellite sid
 */
class SatelliteCorrections {
 public:
  SatelliteCorrections();

  void set_ephemeris(const ephemeris_t &new_eph);
  void set_orbit_correction(const OrbitCorrection &new_orbit_correction);
  void set_clock_correction(const ClockCorrection &new_clock_correction);
  void set_code_biases(const CodeBiases &new_code_biases);
  void set_phase_biases(const PhaseBiases &new_phase_biases);
  void overwrite_code_biases(const CodeBiases &new_code_biases);
  void overwrite_phase_biases(const PhaseBiases &new_phase_biases);
  optional<const OrbitCorrection &> get_orbit_correction(
      const gps_time_t &time) const;
  optional<const ClockCorrection &> get_clock_correction(
      const gps_time_t &time) const;
  optional<const CodeBiases &> get_code_biases(const gps_time_t &time) const;
  optional<const PhaseBiases &> get_phase_biases(const gps_time_t &time) const;
  optional<const ephemeris_t &> get_ephemeris(
      const CorrectionKey &correction_key) const;
  optional<const ephemeris_t &> get_ephemeris_by_iode(const u32 &iode) const;

  optional<s32> iod_orbit_diff() const;
  optional<s32> iod_clock_diff() const;
  optional<u32> get_ephemeris_iode() const;

  void add_to_correction_times(
      const gps_time_t &time, const bool &expect_phase_bias_flag,
      pvt_common::containers::Map<gps_time_t, u32,
                                  MAX_NUMBER_OF_CORRECTION_TIMES>
          *correction_times_count) const;

 private:
  using TimeSet =
      pvt_common::containers::Set<gps_time_t, MAX_NUMBER_OF_CORRECTION_TIMES>;

  template <typename T>
  void get_valid_times(const gps_time_t &time, const double &validity_period,
                       const T &bias_map,
                       SatelliteCorrections::TimeSet *valid_times) const;
  void get_valid_clock_times(
      const gps_time_t &time,
      SatelliteCorrections::TimeSet *valid_orbit_times) const;
  template <typename T>
  void filter_invalid_bias_times(
      const gps_time_t &time, const TimeSet &valid_set_pre_filtering,
      const double &validity_period, const T &bias_map,
      SatelliteCorrections::TimeSet *valid_set_post_filtering) const;

  TimeCorrectionMap<OrbitCorrection, NUMBER_OF_SSR_ORBIT_CORRECTIONS_TO_STORE>
      orbit_corrections_;
  TimeCorrectionMap<ClockCorrection, NUMBER_OF_SSR_CLOCK_CORRECTIONS_TO_STORE>
      clock_corrections_;
  TimeCorrectionMap<CodeBiases, NUMBER_OF_SSR_CODE_BIAS_CORRECTIONS_TO_STORE>
      code_bias_corrections_;
  TimeCorrectionMap<PhaseBiases, NUMBER_OF_SSR_PHASE_BIAS_CORRECTIONS_TO_STORE>
      phase_bias_corrections_;
  pvt_common::containers::Map<u32, ephemeris_t,
                              NUMBER_OF_EPHEMERIDES_CORRECTIONS_TO_STORE>
      ephemerides_;
  optional<u32> most_recent_iode_;
};

using SatelliteCorrectionsMap =
    pvt_common::containers::Map<pvt_engine::SatIdentifier, SatelliteCorrections,
                                NUM_SATS>;

class IgsDerivedBiases {
 public:
  IgsDerivedBiases();
  IgsDerivedBiasesConfiguration config_;
  CodeBiases calculate_code_bias(const gnss_signal_t &l1_sid,
                                 const CodeBiases &code_biases) const;
  PhaseBiases calculate_phase_bias(const gnss_signal_t &l1_sid,
                                   const CodeBiases &code_biases,
                                   const PhaseBiases &phase_biases) const;
};

class OffsetBiases {
 public:
  OffsetBiases();
  OffsetBiasesConfiguration config_;
  CodeBiases calculate_code_bias(const CodeBiases &code_biases) const;
};

/**
 * Class containing all the corrections for all satellites
 */
class SatelliteCorrectionsHandler {
 public:
  SatelliteCorrectionsHandler();
  bool update_config(const IgsDerivedBiasesConfiguration &config,
                     const OffsetBiasesConfiguration &config_offset);

  void set_satellite_broadcast_ephemeris(const gnss_signal_t &sid,
                                         const ephemeris_t &ephemeris);
  void set_satellite_orbit_correction(const gnss_signal_t &sid,
                                      const OrbitCorrection &orbit_correction);
  void set_satellite_clock_correction(const gnss_signal_t &sid,
                                      const ClockCorrection &clock_correction);
  void set_code_biases(const gnss_signal_t &sid, const CodeBiases &code_biases);
  void set_phase_biases(const gnss_signal_t &sid,
                        const PhaseBiases &phase_biases);

  optional<const SatelliteCorrections &> get_satellite_corrections(
      const pvt_engine::SatIdentifier &sat) const;
  pvt_engine::SatIdSet get_sats_with_corrections() const;
  pvt_common::containers::Map<constellation_t, gps_time_t, CONSTELLATION_COUNT>
  get_best_correction_times(
      const gps_time_t &time, const pvt_engine::SatIdSet &sats_used,
      const pvt_engine::ConstellationBoolMap &expect_phase_bias) const;
  PRC compute_IGS_based_corrections();
  PRC apply_offset_to_code_biases();

  bool are_broadcast_corrections_available(
      const CorrectionKey &correction_key,
      const pvt_engine::SatIdentifier &sat) const;
  bool are_float_precise_corrections_available(
      const CorrectionKey &correction_key, const gnss_signal_t &sid,
      const gps_time_t &time) const;
  bool are_integer_precise_corrections_available(
      const CorrectionKey &correction_key, const gnss_signal_t &sid,
      const gps_time_t &time) const;

  void clear();

 private:
  SatelliteCorrectionsMap sat_correction_map_;
  BiasReference bias_reference_;
  SatelliteCorrections *get_or_create_satellite_corrections(
      const gnss_signal_t &sid);
  bool use_IGS_derived_biases;
  IgsDerivedBiases IGS_derived_biases_;
  bool recalculate_IGS_derived_biases;
  OffsetBiases offset_biases_;
  gps_time_t last_correction_time_;
};

/**
 * Contains residuals for the four grid points surrounding the rover
 */
struct GridValues {
  MeanAndVariance nw;
  MeanAndVariance ne;
  MeanAndVariance sw;
  MeanAndVariance se;

  GridValues();
  GridValues(MeanAndVariance nw_value, MeanAndVariance ne_value,
             MeanAndVariance sw_value, MeanAndVariance se_value);
};

class StecSatelliteCorrections {
 public:
  StecSatelliteCorrections(const optional<gps_time_t> &time,
                           const optional<StecPolynomial> &stec_polynomial,
                           const optional<GridValues> &stec_residuals,
                           bool has_variance);

  // The correction time is only needed for ObservationGenerator's iono change
  // smoothing algorithm
  optional<gps_time_t> get_time() const;
  optional<StecPolynomial> get_stec_polynomial() const;
  optional<GridValues> get_stec_residuals() const;
  bool has_variance() const;

 private:
  optional<gps_time_t> time_;
  optional<StecPolynomial> stec_polynomial_;
  optional<GridValues> stec_residuals_;
  bool has_variance_;
};

class TroposphereCorrections {
 public:
  TroposphereCorrections(const optional<GridValues> &dry_delays,
                         const optional<GridValues> &wet_delays,
                         bool has_variance);

  optional<GridValues> get_dry_delays() const;
  optional<GridValues> get_wet_delays() const;
  bool has_variance() const;

 private:
  optional<GridValues> dry_delays_;
  optional<GridValues> wet_delays_;
  bool has_variance_;
};

class AtmosphericCorrectionsHandler {
 public:
  AtmosphericCorrectionsHandler();

  void set_polynomial_corrections(
      const StecPolynomialCorrections &polynomial_corrections);

  void set_gridded_atmo(const GriddedAtmo &new_gridded_atmo);

  void set_grid_definition(const SsrGrid &new_grid_def);

  optional<SsrGrid> get_grid_definition(const gps_time_t &time) const;

  StecSatelliteCorrections get_stec_corrections(
      const gps_time_t &time, const LatLonDeg &point,
      const pvt_engine::SatIdentifier &sat) const;

  TroposphereCorrections get_troposphere_corrections(
      const gps_time_t &time, const LatLonDeg &point) const;

 private:
  bool find_matching_atmo(const gps_time_t &time,
                          StecPolynomialCorrections *polynomial_corrections,
                          GriddedAtmo *gridded_atmo) const;

  TimeCorrectionMap<StecPolynomialCorrections,
                    NUMBER_OF_SSR_STEC_POLY_CORRECTIONS_TO_STORE>
      polynomial_corrections_;

  TimeCorrectionMap<GriddedAtmo, MAX_SAVED_GRIDDED_ATMO> gridded_atmo_;

  optional<SsrGrid> grid_def_;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_SSR_CORRECTIONS_H
