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

#ifndef SWIFTNAV_BASE_OBS_H
#define SWIFTNAV_BASE_OBS_H

#include <ch.h>

#include <libswiftnav/common.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/gnss_time.h>
#include <libswiftnav/single_epoch_solver.h>

#include <libswiftnav/pvt_engine/firmware_binding.h>

/** \addtogroup base_obs Base station observation handling
 * \{ */

/* Maximum distance between calculated and surveyed base station single point
 * position before we ignore the observation. In metres.
 */
#define BASE_STATION_RESET_THRESHOLD 1000.0

#define BASE_OBS_N_BUFF 5

/* \} */

extern bool base_pos_known;
extern double base_pos_ecef[3];

void base_obs_setup(void);

#endif
