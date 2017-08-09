/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Measurement Engine team <michele@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef ME_CALC_PVT_H
#define ME_CALC_PVT_H

#include <libsbp/navigation.h>
#include <libsbp/observation.h>
#include <libsbp/system.h>
#include <libswiftnav/common.h>
#include <libswiftnav/observation.h>
#include <libswiftnav/pvt.h>
#include <libswiftnav/time.h>
#include <libswiftnav/track.h>

#include <ch.h>

typedef struct {
  u8 signals_tracked;
  u8 signals_useable;
} soln_stats_t;

typedef struct _me_msg_obs_t {
  size_t size;
  navigation_measurement_t obs[MAX_CHANNELS];
  ephemeris_t ephem[MAX_CHANNELS];
  gps_time_t obs_time;
  gnss_solution soln;
} me_msg_obs_t;

/** Maximum time that an observation will be propagated for to align it with a
 * solution epoch before it is discarded.  */
#define OBS_PROPAGATION_LIMIT 10e-3

#define OBS_N_BUFF 2

/* Maximum receiver clock error before it is adjusted back to GPS time.
 * The default value 0.5 ms keeps the receiver close enough to GPS time to
 * always round to the output time stamps to the correct millisecond.
 * Note that values smaller than 0.5 ms will cause oscillation because clock
 * jumps are always done by full milliseconds. */
#define MAX_CLOCK_ERROR_S 0.0005

/* If the residual in a pseudorange excluded by RAIM is larger than this, then
 * drop the channel */
#define RAIM_DROP_CHANNEL_THRESHOLD_M 1000

extern u32 obs_output_divisor;
extern memory_pool_t obs_buff_pool;
extern mailbox_t obs_mailbox;

soln_stats_t solution_last_stats_get(void);

void me_calc_pvt_setup(void);

#endif
