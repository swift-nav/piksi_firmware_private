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

#ifndef PVT_ENGINE_OBSS_H
#define PVT_ENGINE_OBSS_H

#include <starling/build/config.h>
#include <swiftnav/nav_meas.h>
#include <swiftnav/pvt_result.h>

#ifdef __cplusplus
extern "C" {
#endif

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

typedef struct {
  double pseudorange;      /**< Single differenced raw pseudorange [m] */
  double carrier_phase;    /**< Single differenced raw carrier phase [cycle]*/
  double measured_doppler; /**< Single differenced raw measured doppler [Hz] */
  double computed_doppler; /**< Single differenced raw computed doppler [Hz] */
  double sat_pos[3];       /**< ECEF XYZ of ephemeris satellite position [m]*/
  double sat_vel[3];       /**< ECEF XYZ of ephemeris satellite velocity [m]*/
  double cn0;              /**< The lowest C/N0 of the two single differenced
                            *   observations [dB Hz] */
  double rover_lock_time;  /**< The rover lock time [s] */
  double base_lock_time;   /**< The base lock time [s] */
  gnss_signal_t sid;       /**< SV signal identifier */
  nav_meas_flags_t flags;  /**< Measurement flags: see nav_meas.h */
} sdiff_t;

void apply_sat_clock_corrections(u8 n_channels,
                                 navigation_measurement_t *nav_meas[]);

#ifdef __cplusplus
}
#endif

#endif  // PVT_ENGINE_OBSS_H
