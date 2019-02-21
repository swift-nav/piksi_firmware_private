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
#ifndef STARLING_INTEGRATION_H_
#define STARLING_INTEGRATION_H_

/**
 * This file "integrates" the Starling engine into the firmware.
 *
 * The engine is operated via well-defined API calls (no shared global
 * variables). Ultimately, this should be the only header including the
 * starling_threads.h.
 */

#include <starling/config.h>
#include <starling/starling.h>
#include <stdbool.h>

#include "hal/piksi_systime.h"
#include "starling_sbp_settings.h"

typedef struct {
  piksi_systime_t systime;
  dgnss_filter_t mode;
} soln_dgnss_stats_t;

typedef struct {
  piksi_systime_t systime;
  u8 signals_used;
} soln_pvt_stats_t;

void starling_calc_pvt_setup(void);
soln_dgnss_stats_t solution_last_dgnss_stats_get(void);
soln_pvt_stats_t solution_last_pvt_stats_get(void);

#endif
