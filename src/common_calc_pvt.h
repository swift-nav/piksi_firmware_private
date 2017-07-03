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
#include <libswiftnav/pvt.h>
#include <libswiftnav/track.h>
#include <libswiftnav/time.h>
#include <libswiftnav/observation.h>


#include "peripherals/leds.h"
#include "position.h"
#include "nmea.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "manage.h"
#include "simulator.h"
#include "settings.h"
#include "timing.h"
#include "base_obs.h"
#include "ephemeris.h"
#include "signal.h"
#include "system_monitor.h"
#include "main.h"
#include "cnav_msg_storage.h"
#include "shm.h"


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
                        const gnss_solution *soln) ;

#ifdef __cplusplus
}
#endif

#endif /* COMMON_CALC_PVT_H */
