/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef ME_OBSERVATION_THREAD_H
#define ME_OBSERVATION_THREAD_H

#include <libswiftnav/common.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/gnss_time.h>
#include <libswiftnav/nav_meas.h>
#include "utils/position/position.h"

typedef struct {
  u8 signals_tracked;
  u8 signals_useable;
} soln_stats_t;

/** Maximum time that an observation will be propagated for to align it with a
 * solution epoch before it is discarded.  */
#define OBS_PROPAGATION_LIMIT 10e-3

/* Maximum receiver clock error before it is adjusted back to GPS time.
 * The value of 1.01 ms keeps the receiver close enough to GPS time to
 * always round to the output time stamps to the correct 2 ms boundary.
 * Note that values smaller than 1.00 ms will cause oscillation because
 * clock jumps are always done by full milliseconds. */
#define MAX_CLOCK_ERROR_S 0.00101

extern u32 obs_output_divisor;
extern double soln_freq_setting;

void me_obs_setup(void);

u8 collect_nav_meas(u64 current_tc,
                    gps_time_t *current_time,
                    const last_good_fix_t *lgf,
                    navigation_measurement_t nav_meas[],
                    ephemeris_t e_meas[]);

u8 nav_meas_get_sat_count(u8 n_ready,
                          const navigation_measurement_t nav_meas[]);

soln_stats_t solution_last_stats_get(void);

#endif /* ME_OBSERVATION_THREAD_H */
