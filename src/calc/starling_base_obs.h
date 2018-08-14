/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Kevin Dade <kevin@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#ifndef STARLING_BASE_OBS_H_
#define STARLING_BASE_OBS_H_

#include <starling/starling.h>
#include <libswiftnav/gnss_time.h>
#include <libswiftnav/pvt_engine/firmware_binding.h>
#include <libswiftnav/nav_meas.h>

/**
 * Uncollapsed observation input type.
 * Remote observations may contain multiple useful signals for satellites.
 * This observation type can fit more observations than the obss_t.
 * Eventually signals in uncollapsed_obss_t are collapsed, and copied to obss_t.
 */
typedef struct {
  /** GPS system time of the observation. */
  gps_time_t tor;
  /** Approximate base station position.
   * This may be the position as reported by the base station itself or the
   * position obtained from doing a single point solution using the base
   * station observations. */
  double pos_ecef[3];
  /** Is the `pos_ecef` field valid? */
  u8 has_pos;
  /** Observation Solution */
  pvt_engine_result_t soln;

  /** Number of observations in the set. */
  u8 n;
  u8 sender_id;
  /** Set of observations. */
  navigation_measurement_t nm[STARLING_MAX_OBS_COUNT];
} uncollapsed_obss_t;

void convert_starling_obs_array_to_uncollapsed_obss(
    obs_array_t *obs_array, uncollapsed_obss_t *obss); 

void collapse_obss(uncollapsed_obss_t *uncollapsed_obss, obss_t *obss); 

int compare_starling_obs_by_sid(const void *a, const void *b); 

int convert_starling_obs_array_to_obss(obs_array_t *obs_array, obss_t *obss);

#endif
