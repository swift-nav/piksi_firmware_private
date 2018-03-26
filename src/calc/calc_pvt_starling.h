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
#include <libswiftnav/common.h>
#include <libswiftnav/gnss_time.h>
#include <libswiftnav/observation.h>
#include <libswiftnav/pvt_engine/firmware_binding.h>
#include <libswiftnav/single_epoch_solver.h>

#include "piksi_systime.h"
#include "starling_threads.h"
#include "starling_platform_shim.h"

typedef struct {
  piksi_systime_t systime;
  u8 signals_used;
} soln_pvt_stats_t;

/** Maximum time that an observation will be propagated for to align it with a
 * solution epoch before it is discarded.  */
#define OBS_PROPAGATION_LIMIT 10e-3

/* Make the buffer large enough to handle 15 second latency at 10Hz */
#define STARLING_OBS_N_BUFF BASE_LATENCY_TIMEOUT * 10

extern u32 max_age_of_differential;
extern bool send_heading;

double calc_heading(const double b_ned[3]);

soln_dgnss_stats_t solution_last_dgnss_stats_get(void);
void reset_rtk_filter(void);
void set_known_ref_pos(const double base_pos[3]);
void set_known_glonass_biases(const glo_biases_t biases);

soln_pvt_stats_t solution_last_pvt_stats_get(void);

#endif
