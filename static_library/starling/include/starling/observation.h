/*
 * Copyright (C) 2014 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_OBSERVATION_H
#define STARLING_OBSERVATION_H

#include <swiftnav/almanac.h>
#include <swiftnav/common.h>
#include <swiftnav/ephemeris.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/nav_meas.h>
#include <swiftnav/signal.h>

#define MAX_INPUT_OBSERVATION_COUNT 250

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

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
  size_t n;
  bool is_osr;  // Do the individual observations have OSR corrections info
                // populated?
  starling_obs_t observations[MAX_INPUT_OBSERVATION_COUNT];
} obs_array_t;

int cmp_sdiff(const void *a_, const void *b_);
int cmp_amb(const void *a_, const void *b_);
int cmp_amb_sdiff(const void *a_, const void *b_);
int cmp_sdiff_sid(const void *a_, const void *b_);
int cmp_amb_sid(const void *a_, const void *b_);

u8 single_diff(const u8 n_a, const navigation_measurement_t *m_a, const u8 n_b,
               const navigation_measurement_t *m_b, sdiff_t *sds);

bool has_mixed_l2_obs(u8 n, navigation_measurement_t *nav_meas);

typedef bool (*navigation_measurement_predicate_f)(navigation_measurement_t);

typedef bool (*navigation_measurement_predicate_extra_f)(
    navigation_measurement_t, void *);

typedef bool (*starling_obs_predicate_f)(starling_obs_t *);

typedef bool (*starling_obs_predicate_extra_f)(starling_obs_t *, void *);

bool only_gps_l2cm_sid(navigation_measurement_t a);
bool not_gps_l2cm_sid(navigation_measurement_t a);
void collapse_navmeas(u8 n, navigation_measurement_t *nav_meas);
void filter_navmeas(u8 *n, navigation_measurement_t nav_meas[],
                    bool prefer_l2c);

void filter_obs_array(obs_array_t *obsa);

u64 mask_secondary_codes(u64 found_codes);
u64 mark_found_code(const u64 found_codes, const code_t code);
bool is_code_in_mask(const u64 masked_codes, const code_t code);
code_t to_supported_code_t(const code_t code);

/* Given an array of measurements `nav_meas` containing `n` elements,
 * remove all elements for which the selection function `predicate`
 * returns `false`.  The remaining elements (for which `predicate`
 * returned `true`) will be in their original order.  Returns the
 * number of elements in the array after filtering.
 */
u8 filter_nav_meas(u8 n, navigation_measurement_t nav_meas[],
                   navigation_measurement_predicate_f predicate);

/* Given an array of measurements `nav_meas` containing `n` elements,
 * remove all elements for which the selection function `predicate`
 * returns `false`.  The remaining elements (for which `predicate`
 * returned `true`) will be in their original order. The given pointer
 * `extra_data` will be passed as the second argument to the selection
 * function on each call.  Returns the number of elements in the array
 * after filtering.
 */
u8 filter_nav_meas_extra(u8 n, navigation_measurement_t nav_meas[],
                         navigation_measurement_predicate_extra_f predicate,
                         void *extra_data);

u8 filter_starling_obs(obs_array_t *obss, starling_obs_predicate_f predicate);

u8 filter_starling_obs_extra(obs_array_t *obss,
                             starling_obs_predicate_extra_f predicate,
                             void *extra_data);

int cmp_sid_sdiff(const void *a, const void *b);

u8 make_propagated_sdiffs(const u8 n_local,
                          const navigation_measurement_t *m_local,
                          const u8 n_remote,
                          const navigation_measurement_t *m_remote,
                          const double remote_pos_ecef[3], sdiff_t *sds);

u8 filter_sdiffs(u8 num_sdiffs, sdiff_t *sdiffs, u8 num_sats_to_drop,
                 gnss_signal_t *sats_to_drop);

void debug_sdiff(sdiff_t sd);
void debug_sdiffs(u8 n, sdiff_t *sds);

void apply_sat_clock_corrections(u8 n_channels,
                                 navigation_measurement_t *nav_meas[]);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* STARLING_OBSERVATION_H */
