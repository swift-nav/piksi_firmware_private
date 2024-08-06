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

#ifndef LIBSWIFTNAV_PVT_ENGINE_CONFIGURATION_H
#define LIBSWIFTNAV_PVT_ENGINE_CONFIGURATION_H

#include <pvt_common/optional.h>
#include <pvt_engine/RTKLib_apriori_models/rtklib_common_tides.h>
#include <pvt_engine/cno_weighting_params.h>
#include <pvt_engine/ephemeris_handler.h>
#include <pvt_engine/integrity_support_message.h>
#include <pvt_engine/observation.h>
#include <pvt_engine/otl_params.h>
#include <pvt_engine/process_noise_motion.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/pvt_types.h>
#include <pvt_engine/sid_set.h>
#include <starling/build/config.h>
#include <swiftnav/signal.h>

// See `configuration.cc` for default constructors and default values.
// These do not belong in this file.

namespace pvt_engine {

struct code_and_obs_type_t {
  code_and_obs_type_t()
      : code_(CODE_INVALID), obs_type_(INVALID_OBSERVATION_TYPE) {}
  code_and_obs_type_t(code_t code, OBSERVATION_TYPE obs_type)
      : code_(code), obs_type_(obs_type) {}

  bool operator==(const struct code_and_obs_type_t &rhs) const {
    return obs_type_ == rhs.obs_type_ && code_ == rhs.code_;
  }

  bool operator<(const struct code_and_obs_type_t &rhs) const {
    if (code_ == rhs.code_) {
      return obs_type_ < rhs.obs_type_;
    }
    return code_ < rhs.code_;
  }

  code_t code_;
  OBSERVATION_TYPE obs_type_;
};

constexpr s32 MAX_OBS_TYPES_SUPPORTED = 4;

using CodeAndObsTypeSet = pvt_common::containers::Set<
    code_and_obs_type_t, MAX_CODE_TYPES_ON_SATELLITE * MAX_OBS_TYPES_SUPPORTED>;

class SupportedObservations {
 public:
  SupportedObservations() : supported_observations_() {}
  SupportedObservations(const SupportedObservations &other)
      : supported_observations_(other.supported_observations_) {}

  SupportedObservations &operator=(const SupportedObservations &) = default;

  bool is_supported(const ObservationIdentifier &obs_id) const;

  bool is_supported(constellation_t constel) const;

  optional<const CodeAndObsTypeSet &> get_supported(
      const SatIdentifier &sat) const;

  optional<const CodeAndObsTypeSet &> get_supported(
      const sub_constellation_t &sub) const;

  void set_supported(const sub_constellation_t &sub_constellation,
                     const code_and_obs_type_t &code_and_obs_type);

  void set_unsupported(const sub_constellation_t &sub_constellation,
                       const code_and_obs_type_t &code_and_obs_type);

  void add_supported(const SupportedObservations &other);

  void remove_supported(const SupportedObservations &other);

  pvt_common::containers::Set<sub_constellation_t, SUB_CONSTELLATION_COUNT>
  get_supported_subconstellations() const;

  SatIdSet get_supported_satellites(
      const pvt_engine::SatelliteInformationHandler &sat_info) const;

  const pvt_common::containers::MapElement<sub_constellation_t,
                                           CodeAndObsTypeSet>
      *begin() const;

  const pvt_common::containers::MapElement<sub_constellation_t,
                                           CodeAndObsTypeSet>
      *end() const;

  s32 size() const;

 private:
  pvt_common::containers::Map<sub_constellation_t, CodeAndObsTypeSet,
                              SUB_CONSTELLATION_COUNT>
      supported_observations_;
};

SupportedObservations get_supported_obs(
    const obs_filters::ConstellationSet &supported_constellations,
    const obs_filters::FrequencySet &supported_frequencies,
    const obs_filters::ObservationTypeSet &supported_obs_types = {
        PSEUDORANGE, CARRIER_PHASE, COMPUTED_DOPPLER});

SupportedObservations get_supported_obs(
    const obs_filters::CodeSet &supported_codes,
    const obs_filters::ObservationTypeSet &supported_obs_types);

enum class ConfigurationType : int {
  kPiksiMulti,
  kOrionNetwork,
  kGenericL1L2,
  kGenericL1L5,
  kAndroidL1L5,
  kSsrConsumer,
  kTeseoVL1L2,
  kTeseoVL1L5,
  kUbloxF9,
  kUbloxM8L,
};

enum class MeasEngineType : int {
  kGeneric,
  kOrion,
  kPiksiMulti,
  kAndroid,
  kTeseoV,
  kUbloxM8L,
  kUbloxF9,
};

enum class DataOriginFormat : int { ubx, rtcm, unknown };

class GnssInputConfiguration final {
 public:
  using FrequencySet = pvt_common::containers::Set<FREQUENCY, MAX_FREQUENCY>;
  using ConstellationSet =
      pvt_common::containers::Set<constellation_t, CONSTELLATION_COUNT>;
  using CodeSet = pvt_common::containers::Set<code_t, CODE_COUNT>;
  using ObsSet =
      pvt_common::containers::Set<OBSERVATION_TYPE, MAX_OBSERVATION_TYPE>;
  using ObsVarianceMap = pvt_common::containers::Map<OBSERVATION_TYPE, double,
                                                     MAX_OBSERVATION_TYPE>;
  using ObsCodeSetMap = pvt_common::containers::Map<OBSERVATION_TYPE, CodeSet,
                                                    MAX_OBSERVATION_TYPE>;

  // A convenience initializer which can be used as constructor argument like:
  // pvt_engine::Configuration
  //     my_config(pvt_engine::Configuration::QuickInitialization::kPiksiMulti);
  // Which would set:
  //     MeasEngineType = kGeneric
  //     FrequencySet = {L1, L2}
  //     ConstellationSet = {GPS, GAL, BDS, GLO}
  enum class QuickInitialization { kPiksiMulti, kOrion };

  GnssInputConfiguration(const MeasEngineType me_type,
                         const FrequencySet &freq_bands,
                         const ConstellationSet &constellations);
  explicit GnssInputConfiguration(QuickInitialization quick_init_type);

  MeasEngineType get_me_type() const noexcept;

  FrequencySet get_frequencies() const noexcept;
  bool has_frequency(const FREQUENCY freq_band) const noexcept;

  bool is_L1L2() const noexcept;
  bool is_L1L5() const noexcept;

  ConstellationSet get_constellations() const noexcept;
  bool has_constellation(const constellation_t constellation) const noexcept;
  void enable_constellation(const constellation_t constellation) noexcept;
  void enable_constellations(const ConstellationSet &constellations) noexcept;

  bool is_sbas_enabled() const noexcept;
  void set_sbas_enabled(bool enabled) noexcept;

  void set_min_modelled_baseline_len_km(double value) noexcept;
  double get_min_modelled_baseline_len_km() const noexcept;

  void set_elevation_mask_deg(double value) noexcept;
  double get_elevation_mask_deg() const noexcept;

  bool is_instantaneous_velocity_enabled() const noexcept;
  void set_instantaneous_velocity_enabled(bool enabled) noexcept;

  CNoWeightCollection get_cno_weight_params() const noexcept;
  void set_cno_weight_params(
      const CNoWeightCollection &weight_collection) noexcept;

  void set_obs_variances(const ObsVarianceMap &variance_map) noexcept;
  void set_obs_variance(const OBSERVATION_TYPE obs_type,
                        const double std_dev) noexcept;
  double get_obs_variance(const OBSERVATION_TYPE obs_type) const noexcept;

  void set_supported_ssr_codes(
      const OBSERVATION_TYPE obs_type,
      const GnssInputConfiguration::CodeSet &codes) noexcept;
  ObsCodeSetMap get_supported_ssr_codes() const noexcept;
  SupportedObservations get_supported_ssr_observations() const noexcept;

  void set_supported_codes(
      const GnssInputConfiguration::ObsCodeSetMap &code_map) noexcept;
  void set_supported_codes(const OBSERVATION_TYPE obs_type,
                           const CodeSet &codes) noexcept;
  SupportedObservations filter_against_supported_codes(
      const SupportedObservations &supported_obs,
      const ObsSet &observation_types = {PSEUDORANGE, CARRIER_PHASE,
                                         COMPUTED_DOPPLER,
                                         MEASURED_DOPPLER}) const noexcept;
  ObsCodeSetMap get_supported_codes() const noexcept;
  SupportedObservations get_supported_observations(
      const ObsSet &observation_types) const noexcept;

  void set_data_format_origin() noexcept;
  void set_data_format_origin(const DataOriginFormat &data_origin) noexcept;

  DataOriginFormat get_data_format_origin() const noexcept;

 private:
  MeasEngineType me_type_;
  FrequencySet freq_bands_;

  // The constellations enabled through settings (can change run-time),
  // must be a subset of the ones returned from
  // get_build_enabled_constellations_()
  ConstellationSet enabled_constellations_;

  bool sbas_enabled_;
  bool instantaneous_velocity_enabled_;

  double min_modelled_baseline_len_km_;

  CNoWeightCollection cno_weights_collection_;
  ObsVarianceMap obs_variances_;

  ObsCodeSetMap supported_codes_;
  ObsCodeSetMap supported_ssr_codes_;
  DataOriginFormat data_origin_format_;

  double elevation_mask_deg_;

  SupportedObservations get_supported_observations(
      const ObsCodeSetMap &code_map, const ObsSet &observation_types) const
      noexcept;

  // Returns the constellations which are allowed based on the include file
  // <starling/build/config.h> which is set up at compile time
  static ConstellationSet get_build_enabled_constellations();

  void initialize_constellation_sets(
      const ConstellationSet &user_config_constellations) noexcept;
  void initialize_cno_weights() noexcept;
  void initialize_obs_variances() noexcept;
  void initialize_supported_codes() noexcept;
  void initialize_supported_ssr_codes() noexcept;
};

optional<GnssInputConfiguration> parse_pvt_config_str(const char *str);

struct VarianceHandlerConfiguration {
  WEIGHTING_MODEL weighting_model;
  double pseudorange_var;         // m^2
  double carrier_phase_var;       // cycles^2
  double measured_doppler_var;    // (cycles/sec)^2
  double computed_doppler_var;    // (cycles/sec)^2
  double baseline_magnitude_var;  // m^2
  // A parameter which determines the shape of the weighting
  // curve.  Signals that are closer to the horizon are given
  // a larger variance, and larger values of alpha will increase
  // the relative variance.
  double alpha_weighting;
  // Describes by which coefficient GLONASS measurements are down-weighted
  // versus GPS measurements. This coefficient applies to variance
  pvt_common::containers::Map<code_t, double, CODE_COUNT>
      variance_downweighting_factor;
  // Describes for how long after the RTK filter first sees a signal
  // the lock time de-weighting scheme should be applied.
  pvt_common::containers::Map<OBSERVATION_TYPE, double, MAX_OBSERVATION_TYPE>
      lock_time_weighting_period;  // seconds
  // Describes how to scale the variance of a signal on the first
  // epoch we see it, fading to 1.0 over the
  // `lock_time_weighting_period_`.
  pvt_common::containers::Map<OBSERVATION_TYPE, double, MAX_OBSERVATION_TYPE>
      lock_time_weight_scale;  // unitless
  // Scale factor to scale the propagation time for to increase the observation
  // variance
  double propagation_scale_factor;

  // Describes the variance of the zenith iono residual after application of the
  // correction
  pvt_common::containers::Map<IONO_CORRECTION_TYPE, double,
                              NUM_IONO_CORRECTION_TYPE>
      iono_zenith_residual_variance;
  // Describes the variance of the zenith tropo residual after application of
  // the
  // correction
  pvt_common::containers::Map<TROPO_CORRECTION_TYPE, double,
                              NUM_TROPO_CORRECTION_TYPE>
      tropo_zenith_residual_variance;
  // Describes the variance of the orbit/clock residual after application of
  // the correction
  pvt_common::containers::Map<ORBIT_CLOCK_CORRECTION_TYPE, double,
                              NUM_ORBIT_CLOCK_CORRECTION_TYPE>
      orbit_clock_residual_variance;

  // C/N0-based weighting function parameters
  pvt_engine::CNoWeightCollection cno_weight_params_collection;

  double half_cycle_slip_scaling;

  PRC valid_values() const;

  explicit VarianceHandlerConfiguration(const GnssInputConfiguration &config);

 private:
  VarianceHandlerConfiguration();
};

constexpr double cMaxAmbiguityVariance = 1e60;

enum IONO_MAPPING_FUNCTION { SLANT, ZENITH };

// We only estimate iono states for satellites with dual frequency phase
// observations
enum IONO_STATES_TO_MODEL { DUAL_FREQ_PHASE_ONLY, ALL_OBS };

enum IONO_MODE { ABSOLUTE, DIFFERENTIAL };

struct ObservationModelConfiguration {
  OBSERVATION_MODEL_POSITION_MODE position_model_mode;
  double
      initial_variance_position[MAX_POSITION_MODE];  // pos, vel, acc, inst vel
  double initial_variance_decoupled_clock;
  double initial_variance_single_clock;
  double initial_variance_hardware_biases;
  obs_filters::FrequencySet hardware_bias_frequency_set;
  double satellite_independent_dof_scaling;
  IONO_MAPPING_FUNCTION iono_mapping_function;
  IONO_STATES_TO_MODEL iono_states_to_model;
  IONO_MODE iono_mode;
  double initial_variance_iono;
  double initial_variance_iono_decorrelation_distance;
  double initial_variance_ambiguity;
  double initial_variance_constrained_ambiguity;
  double initial_variance_troposphere;
  double initial_variance_phase_windup;
  double initial_state_estimate;
  double amb_constraint_variance;
  optional<Eigen::Vector3d> initial_rover_position;
  pvt_common::containers::Map<gnss_signal_t, s32, cMaxAmbiguities>
      constrained_ambs_map;

  PRC validate_constrained_positions() const;
  PRC valid_values() const;

  explicit ObservationModelConfiguration(const GnssInputConfiguration &config);

 private:
  ObservationModelConfiguration();
  bool bypass_constrained_position_validation;
};

struct ProcessModelConfiguration {
  PROCESS_NOISE_MOTION_TYPE process_noise_motion_mode;
  double process_noise_motion_north_east[MAX_POSITION_MODE];
  double process_noise_motion_down[MAX_POSITION_MODE];
  double process_noise_decoupled_clock;
  double process_noise_single_clock;
  double process_noise_hardware_biases;
  double process_noise_satellite_orbit_clock;
  IONO_MODE iono_mode;
  double process_noise_iono;
  double process_noise_iono_decorrelation_distance;
  double process_noise_ambiguity;
  double process_noise_troposphere;
  double process_noise_phase_windup;

  double default_DoF_proc_noise_constant;
  double hardware_bias_DoF_proc_noise_constant;
  double phase_windup_DoF_proc_noise_constant;

  bool constrain_rover_position;

  PRC valid_values() const;
  void set_correction_source(const CORRECTION_SOURCE &source);

  explicit ProcessModelConfiguration(const GnssInputConfiguration &config);

 private:
  ProcessModelConfiguration();
  // A private function that allows us to get the value to assign to
  // process_noise_satellite_orbit_clock in the constructor. This shouldn't be
  // exposed and set_correction_source should be used instead to set
  // process_noise_satellite_orbit_clock based on the correction source type.
  static double get_sat_orbit_clock_proc_noise(const CORRECTION_SOURCE &source);
};

struct ObsProcModelConfiguration {
  ObservationModelConfiguration obs_model_config;
  ProcessModelConfiguration process_model_config;
  VarianceHandlerConfiguration variance_handler_config;
  ObsProcModelTypes models_not_to_reset_in_partial_reinit;

  PRC valid_values() const;

  explicit ObsProcModelConfiguration(const GnssInputConfiguration &config);

 private:
  ObsProcModelConfiguration(
      const ObservationModelConfiguration &obs_model_config_,
      const ProcessModelConfiguration &process_model_config_,
      const VarianceHandlerConfiguration &variance_handler_config_);
};

struct CommonNonOTLConfiguration {
  std::string external_data_path;
  std::string stn_code_;
  std::string atx_filename_;
  bool load_receiver_ant_from_file_;
  bool assert_on_no_ant_file_;
  TROPO_MODEL tropo_model;
  std::string ant_type_;
  std::string ref_ant_type_;
  std::string receiver_type_;

  PRC valid_values() const;

  explicit CommonNonOTLConfiguration(const GnssInputConfiguration &config);

 private:
  CommonNonOTLConfiguration();
};

struct OTLConfiguration {
  OtlParams otl_params;
  bool use_compiled_otl;
  double otl_update_time;

  PRC valid_values() const;

  explicit OTLConfiguration(const GnssInputConfiguration &config);

 private:
  OTLConfiguration();
};

struct CommonAprioriModelConfiguration {
  CommonNonOTLConfiguration non_otl_config_;
  OTLConfiguration otl_config_;

  double max_reference_observation_age;
  bool use_sbas;

  PRC valid_values() const;

  explicit CommonAprioriModelConfiguration(
      const GnssInputConfiguration &config);
};

enum FILTER_MANAGER_TYPE { SPP_MODE, PPP_MODE, RTK_MODE };

struct UndifferencedObsAprioriModelTypesConfiguration : AprioriModelTypes {
  PRC valid_values() const;

  explicit UndifferencedObsAprioriModelTypesConfiguration(
      const GnssInputConfiguration &config);

 private:
  UndifferencedObsAprioriModelTypesConfiguration();
};

struct SdiffAprioriModelTypesConfiguration : public AprioriModelTypes {
  PRC valid_values() const;

  explicit SdiffAprioriModelTypesConfiguration(
      const GnssInputConfiguration &config);

 private:
  SdiffAprioriModelTypesConfiguration();
};

struct FilterManagerAprioriModelConfiguration {
  static constexpr double kDefaultElevationMask = 10.0;

  double elevation_mask;  //  in degrees
  pvt_common::containers::Map<FREQUENCY, u8, MAX_FREQUENCY> code_phase_cn0_mask;
  pvt_common::containers::Map<FREQUENCY, u8, MAX_FREQUENCY>
      measured_doppler_cn0_mask;
  s32 min_sats;
  s32 min_gps_sats;
  s32 max_sats;
  // Maximum number of carrier phase allowable in the filter
  s32 max_carrier_phase;
  // The nominal number of maximum satellites for the Min/Max model to consider
  // You probably want to change max_sats, not this
  s32 nominal_num_max_sats;
  pvt_common::containers::Map<constellation_t, u8, CONSTELLATION_COUNT>
      constellation_priority;
  bool use_klobuchar;

  SdiffAprioriModelTypesConfiguration sdiff_apriori_model_types_;

  double min_modelled_baseline_len_km_;

  PRC valid_values() const;

  explicit FilterManagerAprioriModelConfiguration(
      const GnssInputConfiguration &config,
      const FILTER_MANAGER_TYPE &filter_manager_type);
  const UndifferencedObsAprioriModelTypesConfiguration &
  get_undifferenced_apriori_models() const {
    return undifferenced_obs_apriori_model_types_;
  }

  FilterManagerAprioriModelConfiguration(
      const UndifferencedObsAprioriModelTypesConfiguration
          &undifferenced_obs_apriori_model_types,
      const SdiffAprioriModelTypesConfiguration &sdiff_apriori_model_types);
  UndifferencedObsAprioriModelTypesConfiguration
      undifferenced_obs_apriori_model_types_;
};

struct FilterModels
    : public pvt_common::containers::StaticVector<MODEL_TYPE, MAX_MODEL_NUM> {
  FilterModels &operator=(
      const pvt_common::containers::StaticVector<MODEL_TYPE, MAX_MODEL_NUM>
          &rhs);
  PRC valid_values() const;
  void set_single_differenced_without_iono();
  void set_single_differenced_with_iono();
  bool hardware_bias_jump_expected;

  explicit FilterModels(const GnssInputConfiguration &config);

 private:
  FilterModels();
};

struct RobustEstimatorConfiguration {
  bool do_outlier_detection;
  double min_residual_for_outlier_detection;
  double min_sigma_for_outlier_detection;
  double max_percentage_of_outliers;
  s32 max_number_of_outliers;
  s32 max_outlier_failures_before_init;
  // How long to tolerate too high MDE before resetting filter, in seconds
  s32 mde_too_high_timeout_s;

  // The threshold for detemining if an observation to the position has provided
  // a degree of freedom to the solution. The coefficient of the mapping
  // function from observation to residual space is compared to this number - if
  // it's too low, observation errors simply map into states (e.g. if we only
  // have one observation relating to a clock parameter) and are therefore not
  // providing observability
  double DoF_threshold;

  // The flag would result in both carrier-phase (and subsequently, computed
  // Doppler) observations with the half-cycle-slip ambiguity unresolved to be
  // accepted for processing.
  bool allow_half_cycle_slip;

  // The minimum number of obs to the position with degrees of freedom
  // (providing
  // observability above the DOF_THRESHOLD, see above) that we require to accept
  // an epoch's observations.
  s32 min_pos_observability_rank;

  optional<double> max_allowable_position_error_meters;
  optional<double> max_allowable_velocity_error_meters_per_second;

  double pseudorange_outlier_threshold_for_all_signal_rejection;

  explicit RobustEstimatorConfiguration(const GnssInputConfiguration &config);
  PRC valid_values() const { return RC_S_OK; }
};

/**
 * Controls whether to generate insights in the filter in order to save on CPU
 * usage.
 */
struct FilterInsightsGenerationConfig {
  bool compute_insights() const {
    if (BUILD_CONFIG_ENABLE_INSIGHTS) {
      return compute_insights_;
    }
    return false;
  }

  void set_compute_insights(bool compute_insights) {
    compute_insights_ = compute_insights;
  }

  explicit FilterInsightsGenerationConfig(bool compute)
      : compute_insights_(compute) {}
  PRC valid_values() const;

 private:
  bool compute_insights_;
};

struct VarianceScalingConfiguration {
  double min_pos_variance_factor;
  double min_avg_vel_variance_factor;
  double avg_vel_variance_scale_factor;
  double min_inst_vel_variance_factor;
  double inst_vel_variance_scale_factor;

  explicit VarianceScalingConfiguration();
  PRC valid_values() const;
};

struct FilterConfiguration {
  static constexpr double kDefaultDoFScaleFactor = 8.0 / 9.0;

  ObsProcModelConfiguration obs_proc_model_config;
  s32 num_iterations_on_initialization;
  double max_tolerance_to_iterate;
  DOPS dop_limit;
  FilterModels filter_models;
  RobustEstimatorConfiguration robust_config;
  obs_filters::ObservationTypeSet obs_types;
  bool should_change_measured_doppler_sign;
  // This scale factor is used to ensure that the degrees of freedom remains
  // bounded and that we therefore don't eventually accept anything in the
  // ambiguity search because our DoF is so high that the threshold becomes very
  // low. This factor is used to scale the current number of degrees of freedom
  // (from previous epochs) before adding the new ones from the current epoch.
  double DoF_scale_factor;
  FilterInsightsGenerationConfig filter_insights_generation_config;

  optional<VarianceScalingConfiguration> var_scaling_config;

  // The known (truth) position that the filter should be constrained to when
  // debugging. This may be a static position or a dynamic one that must be
  // changed each epoch.
  optional<Eigen::Vector3d> constrained_rover_position;

  PRC valid_values() const;
  void set_correction_source(const CORRECTION_SOURCE &source);

  explicit FilterConfiguration(const GnssInputConfiguration &config);

 private:
  FilterConfiguration(const GnssInputConfiguration &config,
                      const ObsProcModelConfiguration &obs_proc_model_config_,
                      const FilterModels &filter_models_);
};

enum class LambdaDropStrategy {
  ALL,               // Drop all satellites
  EXTREME_VARIANCE,  // Drop sats with highest and lowest variance
  MAX_VARIANCE,      // Drop sat with highest variance
  MIN_VARIANCE,      // Drop sat with lowest variance
  NONE               // Drop no satellites
};

enum class LambdaConfigurationMethod { MANUAL, AUTOMATIC };

enum class LambdaSearchStrategy { ONE_BY_ONE, ALL_TOGETHER };

enum class LambdaCombinationStrategy { WIDELANE, UNCOMBINED };

struct LAMBDAValidationConfiguration {
  s32 num_candidates;
  s32 min_dd_signals;
  s32 min_signals_for_f_test;
  s32 min_signals_for_f_test_static;
  double r_ratio_min;
  double r_ratio_min_static;
  LambdaSearchStrategy search_strategy;
  LambdaCombinationStrategy combination_strategy;
  LambdaDropStrategy drop_strategy;
  double code_sum_squared_normalized_residuals_weight;
  double code_sum_squared_normalized_residuals_weight_static;
  double minimum_variance_for_one_by_one_fixing;
  double maximum_speed_for_conservative_fixing_meters_per_second;
  LambdaConfigurationMethod config_method;

  PRC valid_values() const;
  void set_correction_source(const CORRECTION_SOURCE &source);
  void set_config_for_static_or_dynamic_mode(const bool is_dynamic);

  explicit LAMBDAValidationConfiguration(const GnssInputConfiguration &config);

 private:
  LAMBDAValidationConfiguration();
};

struct SnapshotConfiguration {
  FilterConfiguration snapshot_filter_config;
  LAMBDAValidationConfiguration validation_config;
  u16 min_dual_frequency_double_differences;
  double max_position_variance_to_fix;

  PRC valid_values() const;
  void set_correction_source(const CORRECTION_SOURCE &source);
  void set_config_for_static_or_dynamic_mode(const bool is_dynamic);

  explicit SnapshotConfiguration(const GnssInputConfiguration &config);

 private:
  SnapshotConfiguration(
      const FilterConfiguration &ss_filter_config_,
      const LAMBDAValidationConfiguration &validation_config_);
};

// Maximum number of historical snapshot windows that can be
// configured at runtime.
constexpr s32 cMaxSnapshots = 4;
constexpr s32 cMaxCandidates = 2;

struct CrossValidationConfiguration {
  // Minimum number of ambiguities required for us to attempt to
  // cross-validate
  s32 min_ambiguities;

  // Maximum allowed deviation between the float filter's baseline
  // solution and the fixed baseline solution, in meters, above which
  // we assume the fixing has gone wrong somehow.
  double max_float_baseline_discrepancy;

  // Number of independent history snapshots to use for
  // cross-validation of fixed ambiguity values.
  s32 n_snapshots;

  // Minimum number of signals in common between the previous
  // validated set and any new validated set for us to attempt to
  // adjust the previous ambiguities.  Note that this is applied
  // independently per code_t.
  s32 min_repair_overlap;

  // Fraction of signals in common between the previous validated set
  // and any new validated set above `min_repair_overlap` that may
  // disagree without preventing us from adjusting the previous
  // ambiguities.  Note that like `min_repair_overlap`, this is
  // applied independently per code_t.
  //
  // This is a denominator; higher values are more conservative.
  // Suppose we see `k` overlapping signals between the previous and
  // current validated sets.  We require at least `min_repair_overlap`
  // signals to be present before we attempt adjustment, but we will
  // not adjust previous ambiguities unless
  //
  //     k - (k - min_repair_overlap)/repair_overlap_fraction
  //
  // (integer division, i.e. with `floor`) ambiguity differences agree
  // between the two sets.  For example, say we see `k = 5`
  // overlapping signals.  If `min_repair_overlap` is set to 3 and
  // this (`repair_overlap_fraction`) is set to 2, then
  //
  //     5 - (5 - 3) / 2 = 4
  //
  // differences must agree for us to proceed with adjustment.  In
  // contrast, if this is set to 3, then
  //
  //     5 - (5 - 3) / 3 = 5
  //
  // or all of the differences must be the same.
  s32 repair_overlap_fraction;

  u32 candidates_to_store;
  u32 min_confirmations_without_conflicts;
  u32 min_confirmations_with_conflicts;
  double min_best_second_best_confirmation_ratio_with_conflicts;

  explicit CrossValidationConfiguration(const GnssInputConfiguration &config);

 private:
  CrossValidationConfiguration();

 public:
  PRC valid_values() const;
};

struct BaselineConfiguration {
  double max_baseline;
  optional<double> max_uncombined_fixed_baseline;

  PRC valid_values() const;

  explicit BaselineConfiguration(const GnssInputConfiguration &config);

 private:
  BaselineConfiguration();
};

struct AmbiguityManagerConfiguration {
  optional<double> max_secs_processed;
  SnapshotConfiguration snapshot_config;
  CrossValidationConfiguration validation_config;
  obs_filters::CodeSet codes_to_fix;
  BaselineConfiguration baseline_config;
  bool attempt_to_fix;

  PRC valid_values() const;
  void set_correction_source(const CORRECTION_SOURCE &source);
  void set_config_for_static_or_dynamic_mode(const bool is_dynamic);

  explicit AmbiguityManagerConfiguration(const GnssInputConfiguration &config);

 private:
  AmbiguityManagerConfiguration(
      const SnapshotConfiguration &ss_config,
      const CrossValidationConfiguration &validation_config_,
      const BaselineConfiguration &baseline_config_);

  using FreqToCodesMap =
      pvt_common::containers::Map<FREQUENCY, obs_filters::CodeSet,
                                  MAX_FREQUENCY>;
  using ConstellationToFreqCodesMap =
      pvt_common::containers::Map<constellation_t, FreqToCodesMap,
                                  CONSTELLATION_COUNT>;
  ConstellationToFreqCodesMap get_default_codes_to_fix() const noexcept;
};

struct BasePositionConfiguration {
  explicit BasePositionConfiguration();
  bool operator==(const BasePositionConfiguration &rhs) const;

  double SPP_base_difference_warning_threshold;
  double SPP_base_difference_reset_threshold;
  double surveyed_base_movement_reset_threshold;
};

struct EphemerisConfiguration {
  bool use_polynomials;
  // timespan of the interpolating polynomial, value chosen to constrain
  // polynomial and velocity approximation errors within 1cm in position and
  // 1mm/s in velocity
  double poly_interval_s;
  // extend the polynomial validity before the first node
  double poly_interval_pre_s;

  EphemerisConfiguration();
  PRC valid_values() const;
};

struct IgsDerivedBiasesConfiguration {
  bool use_IGS_derived_biases;
  pvt_common::containers::Map<gnss_signal_t, double, cNumSat> MW_biases;
  pvt_common::containers::Map<gnss_signal_t, double, cNumSat> delta_code_biases;
  pvt_common::containers::Set<code_t, CODE_COUNT> codes_for_bias_calculation;

  IgsDerivedBiasesConfiguration();
  PRC valid_values() const;
};

struct OffsetBiasesConfiguration {
  bool apply_offset;
  pvt_common::containers::Map<code_t, double, NUM_CODES> offsets;
  pvt_common::containers::Set<code_t, CODE_COUNT> codes_for_bias_calculation;

  OffsetBiasesConfiguration();
  PRC valid_values() const;
};

struct ProtectionLevelConfiguration {
  IntegritySupportMessage ism;

  // Target integrity risk level allocates Starling's total integrity budget,
  // comprising the sum integrity risk due to an internal integrity event
  // within Starling and the total probability of hazardously misleading
  // information (PHMI) from any GNSS satellite
  // i.e., TIR = P_internal_integrity_event + PHMI_total
  TIR tir;

  // Probability of an integrity event caused by Starling's internal
  // processes (e.g. false ambiguity fix, missed cycle slip, etc.)
  double P_internal_integrity_event;

  // Relative allocations of PHMI to each position state (NED coordinates)
  //   Note: - these allocations define the relative ARAIM integrity budget
  //           assigned to each dimension
  //         - the allocations will be normalized in usage to sum up to 1
  Eigen::Vector3d PHMI_allocations;

  // Threshold for integrity risk due to unmonitored GNSS faults as a
  // fraction of the total PHMI
  double ratio_unmonitored_fault_threshold_to_PHMI;

  // Prior probability that a nominal solution separation triggers a false
  // integrity alert on each dimension (NED), used for setting thresholds on
  // solution separation in ARAIM
  Eigen::Vector3d P_FA;

  // Maximum number of exclusion attempts if solution separation test fails
  s32 N_max_exclusion_attempts;
  // Numerical tolerance for iterative least squares position solves
  double lsq_tol_m;
  // Maximum number of iterations for iterative least squares position solves
  s32 N_max_lsq_iterations;
  // Numerical tolerance for iterative computation of protection levels
  double PL_tol_m;
  // Maximum number of iterations for computation of protection levels
  s32 N_max_PL_iterations;

  explicit ProtectionLevelConfiguration(
      const IntegritySupportMessageConfiguration &ism_config);
  PRC valid_values() const;

  // Get the total probability of hazardously misleading information, defining
  // the overall integrity budget for ARAIM
  double PHMI_total() const;
  // Get the ARAIM integrity budget allocated to each dimension (NED)
  Eigen::Vector3d PHMI() const;
  // Get the absolute value of the threshold for integrity risk due to
  // unmonitored faults
  double P_thres() const;
};

using ConstellationBoolMap =
    pvt_common::containers::Map<constellation_t, bool, CONSTELLATION_COUNT>;

enum class FixedSolutionType : s32 {
  ConditionalOnFloat,
  SingleEpoch,
  SingleEpochOnlyForFixedSats
};

struct FilterManagerConfiguration {
  static constexpr double kDefaultMaxVarForFixedPos = 1;
  static constexpr double kDefaultRatioThresholdRegular = 3.0;
  static constexpr double kDefaultRatioStaticThresholdStrict = 4.0;

  AmbiguityManagerConfiguration ambiguity_manager_config;
  FilterConfiguration time_matched_filter_config;
  FilterConfiguration low_latency_filter_config;

  // time (sec) between two obs beyond which we should reinitialize the filter
  // to prevent the dynamic model or process uncertainty calculation from
  // numerical instability
  double filter_reset_delta_time;

  SupportedObservations supported_observations;
  SupportedObservations supported_measured_doppler_observations_;
  FilterManagerAprioriModelConfiguration filter_manager_apriori_model_config_;

  optional<double> known_baseline_magnitude;

  double max_var_for_fixed_pos;
  double max_var_for_fixed_pos_strict;

  // There are various biases (orbit error, iono, tropo, etc) which
  // aren't explicitly modeled in our filter. The result is that our position
  // variance can be overly optimistic. We add a nominal value to each
  // component so that the variance more realistically defines the expected
  // error.
  double spp_position_bias_std_dev;
  double sbas_position_bias_std_dev;

  s32 min_sats_for_fixed_flag;
  s32 min_signals_for_fixed_flag;
  s32 min_signals_for_fixed_flag_strict;
  bool enable_fixed_state_output;

  CORRECTION_TYPE correction_type;
  ConstellationBoolMap expect_phase_biases;

  bool disable_raim;

  FixedSolutionType fixed_solution_type;
  DataOriginFormat data_origin_format;

  bool enable_protection_level_output = false;
  ProtectionLevelConfiguration protection_level_config;

  bool enable_output_gating;

  bool enable_instantaneous_velocity_filter;

  bool require_valid_carrier;

  PRC valid_values() const;

  void set_config_for_correction_source(const CORRECTION_SOURCE &source);
  void set_config_for_static_or_dynamic_mode(const bool is_dynamic);

  explicit FilterManagerConfiguration(
      const pvt_engine::GnssInputConfiguration &config,
      const FILTER_MANAGER_TYPE &filter_manager_type);

 private:
  FilterManagerConfiguration(
      const AmbiguityManagerConfiguration &amb_man_config,
      const FilterConfiguration &tm_filter_config,
      const FilterConfiguration &ll_filter_config,
      const FilterManagerAprioriModelConfiguration
          &filter_manager_apriori_model_config);
};

struct MasterConfiguration {
  BasePositionConfiguration base_pos_config;
  EphemerisConfiguration ephemeris_config;

  CommonAprioriModelConfiguration common_apriori_model_config_;
  FilterManagerConfiguration filter_manager_config_;

  double high_rate_time_matched;
  double low_rate_time_matched;

  s32 signal_limit_high_rate_time_matched;

  // `false` by default to ensure only messages with the _GNSS decorator are
  // sent when the fusion engine is used, which is typical for Starling.
  //
  // `true` when only the GNSS engine is used, as the GNSS engine would need to
  // send messages with the _GNSS decorator and additional metadata in the
  // absence of the fusion engine.
  bool enable_gnss_only_output;

  IgsDerivedBiasesConfiguration igs_derived_biases_config;

  bool prefer_l2c;

  PRC valid_values() const;

  explicit MasterConfiguration(
      const GnssInputConfiguration &config,
      const FILTER_MANAGER_TYPE &filter_manager_type_,
      const CommonAprioriModelConfiguration &common_apriori_model_config);

 private:
  MasterConfiguration(
      const CommonAprioriModelConfiguration &common_apriori_model_config,
      const FilterManagerConfiguration &filter_manager_config);
};

struct ObservationGeneratorConfiguration {
  pvt_common::containers::Set<code_t, CODE_COUNT> codes_for_generation;
  CORRECTION_TYPE correction_type;
  ConstellationBoolMap expect_phase_biases;
  optional<Eigen::Vector3d> virtual_base_location;
  AprioriModelTypes apriori_model_list;
  CommonAprioriModelConfiguration common_config;
  SupportedObservations supported_observations;
  FilterManagerAprioriModelConfiguration filter_manager_apriori_model_config;
  FIXING_TYPE fixing_type;
  IgsDerivedBiasesConfiguration igs_derived_biases_config;
  OffsetBiasesConfiguration offset_biases_config;
  double ssr_orbit_clock_range_std_meters;
  double otl_update_threshold_meters;
  double vrs_placement_distance_meters;
  double maximum_median_ionosphere_change;
  double maximum_hardware_bias_change;
  double iono_std_dev_float_threshold;
  pvt_common::containers::Map<constellation_t, double, CONSTELLATION_COUNT>
      iono_std_dev_fixed_threshold;
  double tropo_std_dev_float_threshold;
  double tropo_std_dev_fixed_threshold;

  PRC valid_values() const;

  explicit ObservationGeneratorConfiguration(
      const GnssInputConfiguration &config);

 private:
  ObservationGeneratorConfiguration(
      const CommonAprioriModelConfiguration &common_apriori_config,
      const FilterManagerAprioriModelConfiguration &fm_apriori_config);
};

struct ManualSsrQueryPositionConfiguration {
  // Define the query position path by the linear (Cartesian) path between two
  // points
  Eigen::Vector3d point_1;
  Eigen::Vector3d point_2;
  double travel_time;

  PRC valid_values() const;

  explicit ManualSsrQueryPositionConfiguration(
      const ConfigurationType &config_type);
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_CONFIGURATION_H
