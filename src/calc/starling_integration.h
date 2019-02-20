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

#include "starling_sbp_settings.h"

#include <starling/config.h>
#include <starling/starling.h>
#include <stdbool.h>

#include "hal/piksi_systime.h"

void starling_calc_pvt_setup(void);

/* Hold information about the system time and type of most recent solutions. */
typedef struct piksi_solution_info_t {
  piksi_systime_t last_time_spp;
  piksi_systime_t last_time_rtk;
  bool            was_last_rtk_fix;
  size_t          num_spp_signals;
} piksi_solution_info_t;

void piksi_solution_info_get(piksi_solution_info_t *info);

#endif
