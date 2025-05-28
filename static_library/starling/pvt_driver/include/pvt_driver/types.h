/**
 * Copyright (C) 2020 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef PVT_DRIVER_TYPES_H
#define PVT_DRIVER_TYPES_H

#include <swiftnav/ephemeris.h>

#include <starling/build/config.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  PVT_DRIVER_ANT_NONE = 0,
  PVT_DRIVER_ANT_GPS1000,
  PVT_DRIVER_ANT_AMOTECH_L1L2_A14,
} pvt_driver_antenna_t;

typedef enum {
  PVT_DRIVER_BRIDGE_MODE_DEFAULT,
  PVT_DRIVER_BRIDGE_MODE_BYPASS,
} pvt_driver_bridge_mode_t;

typedef enum {
  PVT_DRIVER_INSIGHTS_DISABLE_TRUTH,
  PVT_DRIVER_INSIGHTS_TRUTH_TRAJECTORY,
  PVT_DRIVER_INSIGHTS_STATIC_TRUTH,
} pvt_driver_insights_truth_type_t;

typedef struct {
  pvt_driver_insights_truth_type_t truth_type;
  const char *truth_trajectory_file_path;
  double static_truth_ecef[3];
  bool constrain_to_truth;
  double truth_threshold;
} pvt_driver_insights_truth_config_t;

typedef struct {
  bool enable_insights_output;
  bool light_insights;
  const char *insights_file_path;
  pvt_driver_insights_truth_config_t truth_config;
} pvt_driver_insights_config_t;

/* arguably we could choose this to be smaller than MAX_CHANNELS, as 63
 * individual satellites are unlikely for starling, and create a MAX_SATS
 * version of this struct with less `ephemeris_t` structs */
typedef struct {
  size_t n;
  ephemeris_t ephemerides[MAX_CHANNELS];
} ephemeris_array_t;

typedef enum {
  PVT_DRIVER_ME_TYPE_GENERIC,
  PVT_DRIVER_ME_TYPE_ORION,
  PVT_DRIVER_ME_TYPE_PIKSI_MULTI,
  PVT_DRIVER_ME_TYPE_ANDROID,
  PVT_DRIVER_ME_TYPE_TESEO_V,
  PVT_DRIVER_ME_TYPE_UBLOX_M8L,
  PVT_DRIVER_ME_TYPE_UBLOX_F9,
} pvt_driver_me_type_t;

typedef enum {
  PVT_DRIVER_FREQ_BAND_L1,
  PVT_DRIVER_FREQ_BAND_L2,
  PVT_DRIVER_FREQ_BAND_L5,
  PVT_DRIVER_FREQ_BAND_COUNT
} pvt_driver_freq_band_t;

typedef enum {
  PVT_DRIVER_OBS_PSEUDORANGE,
  PVT_DRIVER_OBS_CARRIER_PHASE,
  PVT_DRIVER_OBS_MEASURED_DOPPLER,
  PVT_DRIVER_OBS_COMPUTED_DOPPLER,
  PVT_DRIVER_OBS_TYPE_COUNT
} pvt_driver_obs_type_t;

typedef struct {
  pvt_driver_obs_type_t obs_type;
  double maximum_stddev;
  double inflection_point_x_axis;
  double rate_of_decrease;
} pvt_driver_weight_fnct_params_t;

typedef struct {
  pvt_driver_obs_type_t obs_type;
  double obs_var;
} pvt_driver_obs_variance_t;

typedef struct {
  // Array holding variance factors per observation for elevation angle
  // weighting
  size_t num_variance_elevation_angles;
  pvt_driver_obs_variance_t
      variance_elevation_angles[PVT_DRIVER_OBS_TYPE_COUNT];

  // Array holding function params per observation type for C/No weighting
  size_t num_cno_weighting_functions;
  pvt_driver_weight_fnct_params_t
      cno_weighting_functions[PVT_DRIVER_OBS_TYPE_COUNT];
} pvt_driver_variance_model_t;

typedef struct {
  // Measurement engine type
  pvt_driver_me_type_t me_type;

  // Array which holds frequency bands for the ME
  size_t num_frequency_bands_enabled;
  pvt_driver_freq_band_t frequency_bands_enabled[PVT_DRIVER_FREQ_BAND_COUNT];
} pvt_driver_config_me_t;

typedef struct {
  pvt_driver_obs_type_t obs_type;
  size_t num_supported_codes;
  code_t supported_codes[CODE_COUNT];
} pvt_driver_supported_code_types_t;

typedef enum { UNKNOWN, UBX, RTCM } pvt_driver_data_origin_format_t;

typedef enum {
  PVT_DRIVER_AIDED_OUTLIER_DETECTION_DEFAULT,
  PVT_DRIVER_AIDED_OUTLIER_DETECTION_ENABLE,
  PVT_DRIVER_AIDED_OUTLIER_DETECTION_DISABLE,
} pvt_driver_aided_outlier_detection_option_t;

typedef struct {
  bool do_outlier_detection;
  double min_residual_for_outlier_detection;
  double min_sigma_for_outlier_detection;
  double max_percentage_of_outliers;
  int max_number_of_outliers;
  int max_outlier_failures_before_init;
  int mde_too_high_timeout_s;
  double dof_threshold;
  bool allow_half_cycle_slip;
  int min_pos_observability_rank;
  double max_allowable_position_error_meters;
  double max_allowable_velocity_error_meters_per_second;
  double pseudorange_outlier_threshold_for_all_signal_rejection;
} pvt_driver_robust_estimator_config_t;

typedef struct {
  bool is_robust_estimator_config_parsed;
  pvt_driver_robust_estimator_config_t robust_estimator_config;
} pvt_driver_robust_estimator_t;

typedef struct {
  double time_match_threshold;
  int min_number_of_rover_obs_for_matching;
  int max_number_of_missing_prior_positions;
  double max_prior_position_covariance;
} pvt_driver_aided_outlier_detection_config_t;

typedef struct {
  bool is_aided_outlier_detection_config_parsed;
  pvt_driver_aided_outlier_detection_config_t aided_outlier_detection_config;
} pvt_driver_aided_outlier_detection_t;

typedef struct {
  pvt_driver_aided_outlier_detection_option_t aided_outlier_detection_option;
  pvt_driver_robust_estimator_t aided_robust_estimator;
  pvt_driver_robust_estimator_t unaided_robust_estimator;
  pvt_driver_aided_outlier_detection_t aided_outlier_detection;
} pvt_driver_outlier_detection_t;

/**
 * Defines possible configurations to put the PVT driver into.
 *
 * These configurations can drastically effect the PVT engine performance
 * and should be matched to the ME being used. Generic configurations
 * are provided when the ME is unknown or the PVT engine hasn't been tuned
 * for the ME being used.
 *
 * @note: The settings can be used to modify aspects of the configuration,
 * but not all aspects of the configuration are modifiable after start up
 */
typedef struct {
  // Measurement engine type and frequency bands
  pvt_driver_config_me_t me_config;

  // Array which holds the enabled constellations
  size_t num_constellations_enabled;
  constellation_t constellations_enabled[CONSTELLATION_COUNT];

  // Enables/disables use of the legacy SSR processing code path
  bool use_legacy_ssr_processing;

  // Defines if SBAS corrections are applied
  bool sbas_enabled;
  bool sbas_enabled_is_parsed;

  double elevation_mask_deg_;
  bool elevation_mask_deg_parsed_;

  double min_modelled_baseline_len_km_;
  bool min_modelled_baseline_len_km_parsed_;

  // Defines if instantaneous velocity is estimated
  bool instantaneous_velocity_enabled;
  bool instantaneous_velocity_is_parsed;

  // Parameters for signal variance weighting
  pvt_driver_variance_model_t variance_model;

  // Array where each entry represents which code types are enabled
  // for each observation type
  size_t num_supported_codes_per_obs_type;
  pvt_driver_supported_code_types_t
      supported_codes_per_obs_type[PVT_DRIVER_OBS_TYPE_COUNT];

  // Array where each entry represents which code types are enabled
  // for SSR virtual obs generation for each observation type
  size_t num_ssr_supported_observations;
  pvt_driver_supported_code_types_t
      ssr_supported_observations[PVT_DRIVER_OBS_TYPE_COUNT];

  // Data stream format origin
  pvt_driver_data_origin_format_t data_origin_format;

  pvt_driver_outlier_detection_t outlier_detection;

} pvt_driver_config_t;

typedef struct {
  bool is_fix_mode_valid;
  bool is_gps_time_valid;
  bool is_ins_used;

  gps_time_t t;
  double position[3];
  float covariance[6];
} pvt_driver_prior_position_t;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // PVT_DRIVER_TYPES_H
