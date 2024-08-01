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

#include <pvt_engine/RTKLib_apriori_models/rtklib_common_tides.h>
#include <pvt_engine/integrity_support_message.h>
#include <pvt_engine/observation.h>
#include <pvt_engine/optional.h>
#include <pvt_engine/otl_params.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/pvt_types.h>
#include <pvt_engine/sid_set.h>
#include <swiftnav/signal.h>

// See `configuration.cc` for default constructors and default values.
// These do not belong in this file.

namespace pvt_engine {

enum class ConfigurationType : int {
  kPiksiMulti,
  kOrionNetwork,
  kGenericL1L2,
  kGenericL1L5,
  kAndroidL1L5,
};

optional<ConfigurationType> parse_pvt_config_str(const char *str);

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
  double lock_time_weighting_period;  // seconds
  // Describes how to scale the variance of a signal on the first
  // epoch we see it, fading to 1.0 over the
  // `lock_time_weighting_period_`.
  double lock_time_weight_scale;  // unitless
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

  PRC valid_values() const;

  explicit VarianceHandlerConfiguration(const ConfigurationType &config_type);

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
  double initial_variance_position[MAX_POSITION_MODE];  // pos, vel, acc
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

  explicit ObservationModelConfiguration(const ConfigurationType &config_type);

 private:
  ObservationModelConfiguration();
  bool bypass_constrained_position_validation;
};

struct ProcessModelConfiguration {
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

  explicit ProcessModelConfiguration(const ConfigurationType &config_type);

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

  explicit ObsProcModelConfiguration(const ConfigurationType &config_type);

 private:
  ObsProcModelConfiguration(
      const ObservationModelConfiguration &obs_model_config_,
      const ProcessModelConfiguration &process_model_config_,
      const VarianceHandlerConfiguration &variance_handler_config_);
};

struct AprioriModelConfiguration {
  static constexpr double kDefaultElevationMask = 10.0;

  std::string external_data_path;
  double elevation_mask;  //  in degrees
  pvt_common::containers::Map<FREQUENCY, u8, MAX_FREQUENCY> cn0_mask;
  pvt_engine::obs_filters::ConstellationSet supported_constellations;
  pvt_engine::obs_filters::FrequencySet supported_frequencies;
  s32 min_sats;
  s32 min_gps_sats;
  s32 max_sats;
  // Maximum number of carrier phase allowable in the filter
  s32 max_carrier_phase;
  // The nominal number of maximum satellites for the Min/Max model to consider
  // You probably want to change max_sats, not this
  s32 nominal_num_max_sats;
  pvt_common::containers::Map<constellation_t, u8, MAX_NUM_SATS>
      constellation_priority;
  TROPO_MODEL tropo_model;
  std::string stn_code_;
  std::string atx_filename_;
  std::string ant_type_;
  bool load_receiver_ant_from_file_;
  OtlParams otl_params;
  bool use_compiled_otl;
  double otl_update_time;
  bool use_klobuchar;
  PRC valid_values() const;

  explicit AprioriModelConfiguration(const ConfigurationType &config_type);

 private:
  AprioriModelConfiguration();
};

struct FilterModels
    : public pvt_common::containers::StaticVector<MODEL_TYPE, MAX_MODEL_NUM> {
  FilterModels &operator=(
      const pvt_common::containers::StaticVector<MODEL_TYPE, MAX_MODEL_NUM>
          &rhs);
  PRC valid_values() const;
  void set_single_differenced_without_iono();
  void set_single_differenced_with_iono();

  explicit FilterModels(const ConfigurationType &config_type);

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

  // The threshold for detemining if an observation to the position has provided
  // a degree of freedom to the solution. The coefficient of the mapping
  // function from observation to residual space is compared to this number - if
  // it's too low, observation errors simply map into states (e.g. if we only
  // have one observation relating to a clock parameter) and are therefore not
  // providing observability
  double DoF_threshold;

  // The minimum number of obs to the position with degrees of freedom
  // (providing
  // observability above the DOF_THRESHOLD, see above) that we require to accept
  // an epoch's observations.
  s32 min_pos_observability_rank;

  RobustEstimatorConfiguration();
  PRC valid_values() const { return RC_S_OK; }
};

struct InsightConfiguration {
  bool compute_insights;

  explicit InsightConfiguration(bool compute) : compute_insights(compute) {}
  PRC valid_values() const;
};

struct VarianceScalingConfiguration {
  double min_pos_variance_factor;
  double min_vel_variance_factor;
  double vel_variance_scale_factor;

  explicit VarianceScalingConfiguration();
  PRC valid_values() const;
};

struct FilterConfiguration {
  static constexpr double kDefaultDoFScaleFactor = 8.0 / 9.0;

  ObsProcModelConfiguration obs_proc_model_config;
  ESTIMATOR_TYPE estimator_type;
  s32 num_iterations_on_initialization;
  double max_tolerance_to_iterate;
  DOPS dop_limit;
  FilterModels filter_models;
  RobustEstimatorConfiguration robust_config;
  obs_filters::ObservationTypeSet obs_types;
  // This scale factor is used to ensure that the degrees of freedom remains
  // bounded and that we therefore don't eventually accept anything in the
  // ambiguity search because our DoF is so high that the threshold becomes very
  // low. This factor is used to scale the current number of degrees of freedom
  // (from previous epochs) before adding the new ones from the current epoch.
  double DoF_scale_factor;
  InsightConfiguration insight_config;

  optional<VarianceScalingConfiguration> var_scaling_config;

  // The known (truth) position that the filter should be constrained to when
  // debugging. This may be a static position or a dynamic one that must be
  // changed each epoch.
  optional<Eigen::Vector3d> constrained_rover_position;

  PRC valid_values() const;
  void set_correction_source(const CORRECTION_SOURCE &source);

  explicit FilterConfiguration(const ConfigurationType &config_type);

 private:
  FilterConfiguration(const ObsProcModelConfiguration &obs_proc_model_config_,
                      const FilterModels &filter_models_);
};

enum class LambdaDropStrategy {
  ALL,               // Drop all satellites
  EXTREME_VARIANCE,  // Drop sats with highest and lowest variance
  MAX_VARIANCE,      // Drop sat with highest variance
  MIN_VARIANCE,      // Drop sat with lowest variance
  NONE               // Drop no satellites
};

enum class LambdaSearchStrategy { ONE_BY_ONE, ALL_TOGETHER };

enum class LambdaCombinationStrategy { WIDELANE, UNCOMBINED };

struct LAMBDAValidationConfiguration {
  s32 num_candidates;
  s32 min_dd_signals;
  s32 min_signals_for_f_test;
  double r_ratio_min;
  LambdaSearchStrategy search_strategy;
  LambdaCombinationStrategy combination_strategy;
  LambdaDropStrategy drop_strategy;
  double code_sum_squared_normalized_residuals_weight;
  double minimum_variance_for_one_by_one_fixing;

  PRC valid_values() const;
  void set_correction_source(const CORRECTION_SOURCE &source);

  explicit LAMBDAValidationConfiguration(const ConfigurationType &config_type);

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

  explicit SnapshotConfiguration(const ConfigurationType &config_type);

 private:
  SnapshotConfiguration(
      const FilterConfiguration &ss_filter_config_,
      const LAMBDAValidationConfiguration &validation_config_);
};

// Maximum number of historical snapshot windows that can be
// configured at runtime.
constexpr int cMaxSnapshots = 4;

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

  explicit CrossValidationConfiguration(const ConfigurationType &config_type);

 private:
  CrossValidationConfiguration();

 public:
  PRC valid_values() const;
};

struct BaselineConfiguration {
  double max_baseline;
  optional<double> max_uncombined_fixed_baseline;

  PRC valid_values() const;

  explicit BaselineConfiguration(const ConfigurationType &config_type);

 private:
  BaselineConfiguration();
};

struct AmbiguityManagerConfiguration {
  optional<double> max_secs_processed;
  SnapshotConfiguration snapshot_config;
  CrossValidationConfiguration validation_config;
  ambiguities::CodeSet codes_to_fix;
  BaselineConfiguration baseline_config;
  bool attempt_to_fix;

  PRC valid_values() const;
  void set_correction_source(const CORRECTION_SOURCE &source);

  explicit AmbiguityManagerConfiguration(const ConfigurationType &config_type);

 private:
  AmbiguityManagerConfiguration(
      const SnapshotConfiguration &ss_config,
      const CrossValidationConfiguration &validation_config_,
      const BaselineConfiguration &baseline_config_);
};

struct UndifferencedObsAprioriModelTypesConfiguration : AprioriModelTypes {
  PRC valid_values() const;

  explicit UndifferencedObsAprioriModelTypesConfiguration(
      const ConfigurationType &config_type);

 private:
  UndifferencedObsAprioriModelTypesConfiguration();
};

struct SdiffAprioriModelTypesConfiguration : AprioriModelTypes {
  PRC valid_values() const;

  explicit SdiffAprioriModelTypesConfiguration(
      const ConfigurationType &config_type);

 private:
  SdiffAprioriModelTypesConfiguration();
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
  pvt_common::containers::Map<gnss_signal_t, double, NUM_SATS> MW_biases;
  pvt_common::containers::Map<gnss_signal_t, double, NUM_SATS>
      delta_code_biases;
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

  // Target integrity risk level
  TIR tir;
  // Probability of hazardously misleading information (total integrity budget)
  Eigen::Vector3d PHMI;
  // Threshold for integrity risk coming from unmonitored faults
  double P_thres;
  // Continuity budget allocated to disruptions due to false alert
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

  ProtectionLevelConfiguration(
      const IntegritySupportMessageConfiguration &ism_config);
  PRC valid_values() const;
};

enum FILTER_MANAGER_TYPE { SPP_MODE, PPP_MODE, RTK_MODE };

using ConstellationBoolMap =
    pvt_common::containers::Map<constellation_t, bool, CONSTELLATION_COUNT>;

enum class FloatSolutionTypeToUse : bool { Converged, SingleEpoch };

struct MasterConfiguration {
  static constexpr double kDefaultMaxVarForFixedPos = 1;

  AmbiguityManagerConfiguration ambiguity_manager_config;
  FilterConfiguration time_matched_filter_config;
  FilterConfiguration low_latency_filter_config;
  BasePositionConfiguration base_pos_config;
  EphemerisConfiguration ephemeris_config;
  // time (sec) between two obs beyond which we should reinitialize the filter
  // to prevent the dynamic model or process uncertainty calculation from
  // numerical instability
  double filter_reset_delta_time;
  SdiffAprioriModelTypesConfiguration sdiff_apriori_model_types_;
  AprioriModelConfiguration apriori_model_config;

  optional<double> known_baseline_magnitude;
  double max_reference_observation_age;
  // There are various biases (orbit error, iono, tropo, etc) which
  // aren't explicitly modeled in our filter. The result is that our position
  // variance can be overly optimistic. We add a nominal value to each
  // component so that the variance more realistically defines the expected
  // error. (https://github.com/swift-nav/estimation_team_planning/issues/685)
  double spp_position_bias_std_dev;
  double sbas_position_bias_std_dev;

  double max_var_for_fixed_pos;

  double high_rate_time_matched;
  double low_rate_time_matched;

  s32 signal_limit_high_rate_time_matched;
  s32 min_sats_for_fixed_flag;
  s32 min_signals_for_fixed_flag;
  bool enable_fixed_state_output;

  CORRECTION_TYPE correction_type;
  ConstellationBoolMap expect_phase_biases;
  IgsDerivedBiasesConfiguration igs_derived_biases_config;

  bool prefer_l2c;

  bool disable_raim;

  FloatSolutionTypeToUse float_type_to_use_for_fixed_output;

  bool enable_protection_level_output = false;
  ProtectionLevelConfiguration protection_level_config;

  PRC valid_values() const;

  void set_config_for_correction_source(const CORRECTION_SOURCE &source);

  explicit MasterConfiguration(const ConfigurationType &config_type);
  explicit MasterConfiguration(const ConfigurationType &config_type,
                               const FILTER_MANAGER_TYPE &filter_manager_type_);
  const UndifferencedObsAprioriModelTypesConfiguration &
  get_undifferenced_apriori_models() const {
    return undifferenced_obs_apriori_model_types_;
  }

 private:
  UndifferencedObsAprioriModelTypesConfiguration
      undifferenced_obs_apriori_model_types_;
  MasterConfiguration(
      const AmbiguityManagerConfiguration &amb_man_config,
      const FilterConfiguration &tm_filter_config,
      const FilterConfiguration &ll_filter_config,
      const UndifferencedObsAprioriModelTypesConfiguration
          &undifferenced_obs_apriori_config,
      const SdiffAprioriModelTypesConfiguration &sdiff_apriori_config,
      const AprioriModelConfiguration &apriori_model_config_);
};

struct ObservationGeneratorConfiguration {
  pvt_common::containers::Set<code_t, CODE_COUNT> codes_for_generation;
  CORRECTION_TYPE correction_type;
  ConstellationBoolMap expect_phase_biases;
  optional<Eigen::Vector3d> virtual_base_location;
  AprioriModelTypes apriori_model_list;
  AprioriModelConfiguration apriori_model_config;
  FIXING_TYPE fixing_type;
  IgsDerivedBiasesConfiguration igs_derived_biases_config;
  OffsetBiasesConfiguration offset_biases_config;
  double ssr_orbit_clock_range_std_meters;
  double otl_update_threshold_meters;
  double vrs_placement_distance_meters;
  double maximum_median_ionosphere_change;
  double maximum_hardware_bias_change;
  double iono_std_dev_float_threshold;
  double iono_std_dev_fixed_threshold;
  double tropo_std_dev_float_threshold;
  double tropo_std_dev_fixed_threshold;

  PRC valid_values() const;

  explicit ObservationGeneratorConfiguration(
      const ConfigurationType &config_type);

 private:
  explicit ObservationGeneratorConfiguration(
      const AprioriModelConfiguration &apriori_model_config_);
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_CONFIGURATION_H
