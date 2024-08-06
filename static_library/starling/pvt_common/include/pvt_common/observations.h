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

#ifndef PVT_DRIVER_OBSERVATIONS_H
#define PVT_DRIVER_OBSERVATIONS_H

#include <starling/build/config.h>
#include <swiftnav/nav_meas.h>
#include <swiftnav/pvt_result.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_INPUT_OBSERVATION_COUNT 250

/* Warn on 15 second base station observation latency */
#define BASE_LATENCY_TIMEOUT 15

typedef struct {
  gnss_signal_t sid;
  double pseudorange;
  double carrier_phase;
  float doppler;
  float cn0;
  float lock_time;         // Lock time in seconds.
  nav_meas_flags_t flags;  // Navigation measurement flags.

  /** OSR Corrections, optionally populated */
  u8 osr_flags;  // OSR corrections flags.
  float iono_std;
  float tropo_std;
  float range_std;
} starling_obs_t;

typedef struct {
  u16 sender;
  gps_time_t t;
  size_t n;  // Number of raw pseudorange and carrier phase observations for the
             // satellites being tracked by the device available
  bool is_osr;  // Do the individual observations have OSR corrections info
                // populated?
  starling_obs_t observations[MAX_INPUT_OBSERVATION_COUNT];
} obs_array_t;

/* same as obs_array_t but post-filtering to reduce memory usage */
typedef struct {
  u16 sender;
  gps_time_t t;
  size_t n;
  bool is_osr;
  starling_obs_t observations[MAX_CHANNELS];
} obs_core_t;

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

#ifdef __cplusplus
}
#endif

#endif  // PVT_DRIVER_OBSERVATIONS_H
