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

#include <libsbp/observation.h>
#include <libsbp/navigation.h>
#include <libsbp/system.h>
#include <libswiftnav/common.h>
#include <libswiftnav/pvt.h>
#include <libswiftnav/track.h>
#include <libswiftnav/time.h>
#include <libswiftnav/observation.h>

#define memory_pool_t MemoryPool
#include <ch.h>
#undef memory_pool_t

typedef struct {
  u8 signals_tracked;
  u8 signals_useable;
} soln_stats_t;

typedef struct {
  systime_t systime;
  u8 signals_used;
} soln_pvt_stats_t;

typedef struct _me_msg_obs_t {
  size_t size;
  navigation_measurement_t obs[MAX_CHANNELS];
  ephemeris_t ephem[MAX_CHANNELS];
} me_msg_obs_t;


/** Maximum time that an observation will be propagated for to align it with a
 * solution epoch before it is discarded.  */
#define OBS_PROPAGATION_LIMIT 10e-3

#define OBS_N_BUFF 2

#define OBS_BUFF_SIZE (OBS_N_BUFF * sizeof(obss_t))

extern u32 obs_output_divisor;
extern MemoryPool obs_buff_pool;
extern mailbox_t  obs_mailbox;

soln_stats_t solution_last_stats_get(void);
soln_pvt_stats_t solution_last_pvt_stats_get(void);

void me_calc_pvt_setup(void);

#endif
