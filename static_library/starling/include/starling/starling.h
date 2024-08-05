/*
 * Copyright (C) 2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_CALC_PVT_H
#define STARLING_CALC_PVT_H

#include <libsbp/navigation.h>
#include <libsbp/orientation.h>
#include <libsbp/system.h>
#include <pvt_engine/solution_frequency.h>
#include <starling/config.h>
#include <starling/observation.h>
#include <starling/process_noise_motion.h>
#include <stdbool.h>
#include <swiftnav/common.h>
#include <swiftnav/glonass_phase_biases.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/pvt_result.h>
#include <swiftnav/sbas_raw_data.h>
#include <swiftnav/single_epoch_solver.h>

#ifdef __cplusplus
#define STARLING_NO_RETURN [[noreturn]]
#else
#define STARLING_NO_RETURN _Noreturn
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* arguably we could choose this to be smaller than MAX_CHANNELS, as 63
 * individual satellites are unlikely for starling, and create a MAX_SATS
 * version of this struct with less `ephemeris_t` structs */
typedef struct {
  size_t n;
  ephemeris_t ephemerides[MAX_CHANNELS];
} ephemeris_array_t;

/* same as obs_array_t but post-filtering to reduce memory usage */
typedef struct {
  u16 sender;
  gps_time_t t;
  size_t n;
  bool is_osr;
  starling_obs_t observations[MAX_CHANNELS];
} obs_core_t;

/* container for rover core observations and the associated SPP solution */
typedef struct {
  obs_core_t obs;
  pvt_engine_result_t soln;
} obs_rover_t;

/**
 * Base observation input type. Starling engine operates
 * on base observations provided in this format.
 *
 * TODO(kevin) for now...
 * (michele) fields ordered favoring memory alignment
 */
// NOLINTNEXTLINE(clang-analyzer-optin.performance.Padding)
typedef struct {
  /** Set of observations. */
  navigation_measurement_t nm[MAX_CHANNELS];
  /** Number of observations in the set. */
  u8 n;
  /** Observation Solution */
  pvt_engine_result_t soln;

  /** Are there valid measurement standard deviation values? */
  bool has_std;
  /** Measurement standard deviation values, if present there are `n` of them */
  measurement_std_t std[MAX_CHANNELS];

  /** Is the `pos_ecef` field valid? */
  bool has_pos;
  /** Approximate base station position.
   * This may be the position as reported by the base station itself or the
   * position obtained from doing a single point solution using the base
   * station observations. */
  double pos_ecef[3];

  /** GPS system time of the observation. */
  gps_time_t tor;

  /** sender */
  u16 sender_id;
} obss_t;

/**
 * Paired observation type.
 * A struct to hold a pair of base and rover observations
 * that have been time matched
 */
typedef struct {
  obs_core_t rover_obs;
  obs_core_t base_obs;
  pvt_engine_result_t rover_soln;
} paired_obss_t;

typedef enum {
  STARLINNG_PVT_CONFIG_PIKSI_MULTI,
  STARLINNG_PVT_CONFIG_ORION_NETWORK,
  STARLINNG_PVT_CONFIG_GENERIC_L1L2,
  STARLINNG_PVT_CONFIG_GENERIC_L1L5,
  STARLINNG_PVT_CONFIG_ANDROID_L1L5,
} starling_pvt_config_t;

/** Maximum difference between observation times to consider them matched. */
#define TIME_MATCH_THRESHOLD 2e-2

/* Warn on 15 second base station observation latency */
#define BASE_LATENCY_TIMEOUT 15

#define MIN_RANGE_VALUE 1.8e7

/*******************************************************************************
 * Starling Top-Level API
 ******************************************************************************/

/* All user configurable IO functions used by the Starling Engine.
 * Any functions which are either irrelevant, or not required may
 * be replaced with NULL. */
typedef struct StarlingIoFunctionTable StarlingIoFunctionTable;

/* User may optionally supply debug functions which will may be used to
 * instrument certain modes of operation. */
typedef struct StarlingDebugFunctionTable StarlingDebugFunctionTable;

/* Must be called before using any of the other API functions. */
void starling_initialize_api(void);

/* Run the starling engine on the current thread.
 * Returns once the main thread is started */
bool starling_run(starling_pvt_config_t pvt_config,
                  const StarlingDebugFunctionTable *debug_f);

/* Stops the starling engine in other threads.
 * Blocks until all threads have exited */
void starling_stop(void);

/* Return codes for read functions. */
#define STARLING_READ_OK 0

/* Blocking indicator values. */
#define STARLING_READ_NONBLOCKING 0
#define STARLING_READ_BLOCKING 1

typedef struct {
  /* Sample time. */
  gps_time_t t;
  /* 3-axis acceleration in m/s^2. */
  double acc_xyz[3];
  /* 3-axis angular velocity in rad/s. */
  double gyr_xyz[3];
} imu_data_t;

typedef bool (*sbas_has_corrections_t)(const gnss_signal_t *sid,
                                       const gps_time_t *epoch_time, u8 iode);

/**
 * These handler function singnatures intentionally don't make the data `const`,
 * this is to give the handlers the ability to mutate the data without having
 * to first copy it. The handlers do NOT take ownership of the data. The data
 * is immediately freed once the handler returns so references to the data
 * should not be kept once the handler returns.
 */
typedef void (*rover_obs_handler_f)(obs_array_t *obs_array);
typedef void (*base_obs_handler_f)(obs_array_t *obs_array);
typedef void (*sbas_data_handler_f)(sbas_raw_data_t *data);
typedef void (*ephemeris_array_handler_f)(ephemeris_array_t *eph_arr);
typedef void (*imu_data_handler_f)(imu_data_t *data);

enum ProfileDirective {
  PROFILE_BEGIN,
  PROFILE_END,
};

struct StarlingDebugFunctionTable {
  /* Will be called at the beginning and end of a low-latency thread iteration.
   */
  void (*profile_low_latency_thread)(enum ProfileDirective directive);
};

#define TIR_SAFE_STATE 0
#define TIR_LEVEL_1 1
#define TIR_LEVEL_2 2
#define TIR_LEVEL_3 3
#define TIR_LEVEL_MASK 0x07
#define TIR_LEVEL_OFFSET 0

typedef struct {
  uint8_t tir_level : 3;
} StarlingPLFlags;

typedef struct {
  bool valid;
  double protection_levels_ned[3];
  double pos_ecef[3];
  StarlingPLFlags flags;
} StarlingProtectionLevels;

/**
 * Filter result data type returned by various API functions.
 */
typedef struct {
  dops_t dops;
  pvt_engine_result_t result;
  double rover_ecef[3];
  StarlingProtectionLevels pl;
} StarlingFilterSolution;

typedef struct {
  /**
   * User handling of a low-latency solution.
   *
   * At every processing epoch, possible solutions include both
   * an SPP solution, and an RTK solution. Either one may be invalid,
   * (indicated by NULL pointer), although existence of an RTK
   * solution implies existence of an SPP solution.
   *
   * NOTE: The pointers are only valid within the enclosing scope.
   *       Any copies of the data must be deep copies.
   */
  void (*handle_low_latency)(const StarlingFilterSolution *spp_solution,
                             const StarlingFilterSolution *rtk_solution,
                             const gps_time_t *solution_epoch_time);

  /**
   * User handling of a time-matched solution.
   *
   * The solution pointer may optionally be NULL if there was no
   * valid solution for this epoch of processing. The observation
   * pointers are expected to always be valid.
   *
   * NOTE: The pointers are only valid within the enclosing scope.
   *       Any copies of the data must be deep copies.
   */
  void (*handle_time_matched)(const StarlingFilterSolution *solution,
                              const obs_core_t *obs_base,
                              const pvt_engine_result_t *rover_soln,
                              const double *spp_ecef);
} SolutionHandler;

/* Register any number of solution handlers. Starling Engine will
 * hold the reference you pass until it is removed, so be careful
 * to give a pointer to valid memory with sufficient lifetime.
 *
 * Returns non-zero on failure.
 */
int starling_add_solution_handler(const SolutionHandler *handler);

/* Remove a previously registered solution handler.
 *
 * Returns non-zero on failure.
 */
int starling_remove_solution_handler(const SolutionHandler *handler);

/*******************************************************************************
 * Starling Configuration API
 ******************************************************************************/

/**
 * Various solution output modes supported by the Starling Engine.
 *
 * LOW_LATENCY:  Output a solution immediately for every rover observation.
 * TIME_MATCHED: Output a solution whenever a base observation is received.
 * NO_DGNSS:     Output single-point low-latency solution.
 */
typedef enum {
  STARLING_SOLN_MODE_LOW_LATENCY,
  STARLING_SOLN_MODE_TIME_MATCHED,
  STARLING_SOLN_MODE_NO_DGNSS
} dgnss_solution_mode_t;

typedef enum {
  FILTER_FLOAT,
  FILTER_FIXED,
} dgnss_filter_t;

typedef enum {
  STARLING_ANT_NONE = 0,
  STARLING_ANT_GPS1000,
  STARLING_ANT_AMOTECH_L1L2_A14,
} starling_antenna_t;

/* Reset all filters. */
void starling_reset_all_filters(void);
/* Reset Time Matched filter. */
void starling_reset_time_matched_filter(void);
/* Enable glonass constellation in the Starling engine. */
void starling_set_is_glonass_enabled(bool is_glonass_enabled);
/* Enable galileo constellation in the Starling engine. */
void starling_set_is_galileo_enabled(bool is_galileo_enabled);
/* Enable beidou constellation in the Starling engine. */
void starling_set_is_beidou_enabled(bool is_beidou_enabled);
/* Enable fixed RTK mode in the Starling engine. */
void starling_set_is_fix_enabled(bool is_fix_enabled);
/* Enable klobuchar corrections in the time-matched filter. */
void starling_set_is_time_matched_klobuchar_enabled(bool is_klobuchar_enabled);
/* Enable/disable base obs interpolation */
void starling_set_base_obs_interpolation(bool interp);
/* Indicate for how long corrections should persist. */
void starling_set_max_correction_age(int max_age);
/* Modify the process noise variance values for the filter. */
void starling_set_process_noise_motion(
    PROCESS_NOISE_MOTION_TYPE process_noise_settings);
/* Modify the relative weighting of glonass observations. */
void starling_set_glonass_downweight_factor(float factor);
/* Set the elevation mask used to filter satellites from the solution. */
void starling_set_elevation_mask(float elevation_mask);

/* Get the max sats for the given requested frequency. */
bool starling_get_max_sats_from_requested_freq_and_solution_mode(
    const double requested_frequency_hz,
    const dgnss_solution_mode_t solution_mode, s32 *max_sats);
/* Get the max sats for the given observation rate. */
s32 starling_get_max_sats_from_freq_enum_and_solution_mode(
    observation_rate_t rate, const dgnss_solution_mode_t solution_mode);

/* Set the rate at which the filter calculates solutions, return true if
 * successful. */
bool starling_set_solution_frequency(double requested_frequency_hz);
/* Set a surveyed reference position for the base station. */
void starling_set_known_ref_pos(const double base_pos[3]);
/* Update the glonass biases. */
void starling_set_known_glonass_biases(const glo_biases_t biases);

/* Set the desired solution mode for the Starling engine. */
void starling_set_solution_mode(dgnss_solution_mode_t mode);
/* Get the current solution mode for the Starling engine. */
dgnss_solution_mode_t starling_get_solution_mode(void);

/* Set the antenna offset to compensate for */
void starling_set_antenna_offset(starling_antenna_t antenna);

#ifdef __cplusplus
}
#endif

#endif
