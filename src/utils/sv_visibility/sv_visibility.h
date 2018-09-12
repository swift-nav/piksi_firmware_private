/*
 * Copyright (C) 2016-2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_SV_VISIBILITY_H_
#define SWIFTNAV_SV_VISIBILITY_H_

#include <stdbool.h>
#include <swiftnav/common.h>
#include <swiftnav/constants.h>
#include <swiftnav/coord_system.h>
#include <swiftnav/ephemeris.h>
#include <swiftnav/signal.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** Polar Earth radius in meters */
#define SV_VIS_EARTH_MIN_RADIUS WGS84_B

/** Angle in deg. corresponds to max arc length from LGF when GPS SV visibility
 * status will be computed, equal to
 * arccos(SV_VIS_EARTH_MIN_RADIUS/(SV_VIS_EARTH_MIN_RADIUS + GPS Orbital
 * Height)) */
#define SV_VIS_MAX_ANGLE_FROM_LGF_GPS_DEG (76.14f)

/** Angle in deg. corresponds to max arc length from LGF when GLO SV visibility
 * status will be computed, equal to
 * arccos(SV_VIS_EARTH_MIN_RADIUS/(SV_VIS_EARTH_MIN_RADIUS + GLO Orbital
 * Height)) */
#define SV_VIS_MAX_ANGLE_FROM_LGF_GLO_DEG (75.54f)

/** Angle in deg. corresponds to max arc length from LGF when SBAS SV visibility
 * status will be computed, equal to
 * arccos(SV_VIS_EARTH_MIN_RADIUS/(SV_VIS_EARTH_MIN_RADIUS + SBAS Orbital
 * Height)),
 * where SBAS Orbital Height is 35786000 m as for geostationary objects */
#define SV_VIS_MAX_ANGLE_FROM_LGF_SBAS_DEG (81.32f)

/** Angle in deg. corresponds to max arc length from LGF when SV visibility
 * status will be marked as VISIBLE or NON-VISIBLE */
#define SV_VIS_MAX_UNKNOWN_ANGLE (5.f)

/** Configuration structure contains data needed for SV visibility computation
 * */
typedef struct {
  ephemeris_t *e;      /**< pointer to ephemeris for SV that is subject of
                                    visibility computation */
  double lgf_ecef[3];  /**< LGF X,Y,Z coordinates in meters array in ECEF */
  gps_time_t lgf_time; /**< LGF time stamp */
  float user_velocity; /**< worst-case user velocity in m/s */
  u32 time_delta;      /**< Time in sec since LGF */
} sv_vis_config_t;

void sv_visibility_status_get(const sv_vis_config_t *config,
                              bool *visible,
                              bool *known);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* SWIFTNAV_SV_VISIBILITY_H_ */
