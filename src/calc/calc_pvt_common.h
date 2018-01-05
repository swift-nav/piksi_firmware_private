/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Michele Bavaro <michele@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef COMMON_CALC_PVT_H
#define COMMON_CALC_PVT_H

/* RFT_TODO */

#include <libsbp/navigation.h>
#include <libsbp/system.h>
#include <libswiftnav/common.h>
#include <libswiftnav/gnss_time.h>
#include <libswiftnav/observation.h>
#include <libswiftnav/single_epoch_solver.h>

#include "calc_base_obs.h"
#include "ephemeris/ephemeris.h"
#include "main.h"
#include "manage.h"
#include "nav_msg/cnav_msg_storage.h"
#include "nmea/nmea.h"
#include "peripherals/leds.h"
#include "position/position.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "settings/settings.h"
#include "shm/shm.h"
#include "signal_db/signal_db.h"
#include "simulator.h"
#include "system_monitor/system_monitor.h"
#include "timing/timing.h"

#ifdef __cplusplus
extern "C" {
#endif

void send_observations(u8 n,
                       u32 msg_obs_max_size,
                       const navigation_measurement_t m[],
                       const gps_time_t *t);

bool gate_covariance(gnss_solution *soln);

void extract_covariance(double full_covariance[9],
                        double vel_covariance[9],
                        const gnss_solution *soln);

bool check_covariance(const double pos_accuracy, const double vel_accuracy);

#ifdef __cplusplus
}
#endif

#endif /* COMMON_CALC_PVT_H */
