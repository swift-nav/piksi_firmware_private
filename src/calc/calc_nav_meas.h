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

#include "nav_msg/cnav_msg.h"

#include <swiftnav/common.h>
#include <swiftnav/ephemeris.h>
#include <swiftnav/ionosphere.h>
#include <swiftnav/nav_meas.h>
#include <starling/starling.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

s8 calc_navigation_measurement(u8 n_channels,
                               const channel_measurement_t *meas[],
                               navigation_measurement_t *nav_meas[],
                               const gps_time_t *rec_time);

void apply_gps_cnav_isc(u8 n_channels,
                        navigation_measurement_t *nav_meas[],
                        const ephemeris_t *p_ephe[]);

u8 tdcp_doppler(u8 n_new,
                navigation_measurement_t *m_new,
                u8 n_old,
                navigation_measurement_t *m_old,
                navigation_measurement_t *m_corrected,
                double dt);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* SWIFTNAV_NAV_MEAS_CALC_H */
