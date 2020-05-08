/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_NAV_MEAS_CALC_H
#define SWIFTNAV_NAV_MEAS_CALC_H

#include <pvt_driver/observations.h>
#include <swiftnav/common.h>
#include <swiftnav/ephemeris.h>
#include <swiftnav/ionosphere.h>
#include <swiftnav/nav_meas.h>

#include "nav_msg/cnav_msg.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

s8 calc_navigation_measurement(const channel_measurement_t *meas,
                               starling_obs_t *starling_obs,
                               const gps_time_t *rec_time);

s8 calc_navigation_measurements(u8 n_channels,
                                const channel_measurement_t meas[],
                                obs_array_t *obs_array,
                                const gps_time_t *rec_time);

void apply_gps_cnav_isc(u8 n_channels,
                        navigation_measurement_t *nav_meas[],
                        const ephemeris_t ephe[]);

void apply_cons_time_offsets(u8 n_channels,
                             navigation_measurement_t *nav_meas[]);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* SWIFTNAV_NAV_MEAS_CALC_H */
