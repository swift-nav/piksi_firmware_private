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
#include <assert.h>
#include <math.h>
#include <string.h>

#define DEBUG 0

#include <swiftnav/gnss_time.h>

#include "acq/manage.h"
#include "signal_db/signal_db.h"
#include "sv_visibility.h"

/**
 * The function will compute visibility and visibility known status for a SV
 *
 * \param[in] config pointer to configuration structure sv_config_t
 * \param[out] visible contains information if SV is visible (true)
 *                          or non-visible (false)
 * \param[out] known contains information if SV visibility known (true)
 *                         or unknown (false)
 */
void sv_visibility_status_get(const sv_vis_config_t *config,
                              bool *visible,
                              bool *known) {
  assert(visible != 0 && known != 0);

  if (config == NULL || config->e == NULL) {
    *visible = false;
    *known = false;
    return;
  }

  /* max distance in m (arc length) that user can cover during offline */
  float distance = config->user_velocity * (float)config->time_delta;

  /* calculate angle corresponds to arc length [rad] */
  double arc_angle = distance / SV_VIS_EARTH_MIN_RADIUS;
  log_debug_sid(config->e->sid, "arc_angle [deg] %lf", arc_angle * R2D);
  double sv_max_vis_angle_deg;
  me_gnss_signal_t mesid =
      construct_mesid(config->e->sid.code, config->e->sid.sat);
  switch ((s8)mesid.code) {
    case CODE_GPS_L1CA:
    case CODE_GPS_L2CM:
    case CODE_BDS2_B1:
    case CODE_BDS2_B2:
    case CODE_GAL_E1B:
    case CODE_GAL_E7I:
    case CODE_GAL_E5I:
      sv_max_vis_angle_deg = SV_VIS_MAX_ANGLE_FROM_LGF_GPS_DEG;
      break;
    case CODE_GLO_L1OF:
    case CODE_GLO_L2OF:
      sv_max_vis_angle_deg = SV_VIS_MAX_ANGLE_FROM_LGF_GLO_DEG;
      break;
    case CODE_SBAS_L1CA:
      sv_max_vis_angle_deg = SV_VIS_MAX_ANGLE_FROM_LGF_SBAS_DEG;
      break;
    default:
      log_error("Unknown GNSS code %d", mesid.code);
      assert(0);
  }

  if (arc_angle * R2D >= sv_max_vis_angle_deg) {
    /* user traveled so far even best SV (zenith) cannot be seen */
    *visible = false;
    *known = false;
    return;
  }

  /* determine user time */
  gps_time_t user_time = config->lgf_time;
  user_time.tow += config->time_delta;
  normalize_gps_time(&user_time);

  /* rewrite LGF ECEF for worst case -- altitude = 0 */
  double lgf_ecef_tmp[3] = {0};
  double lgf_llh_tmp[3] = {0};
  memcpy(lgf_ecef_tmp, config->lgf_ecef, sizeof(lgf_ecef_tmp));
  wgsecef2llh(lgf_ecef_tmp, lgf_llh_tmp);
  lgf_llh_tmp[2] = 0; /* set altitude of LGF to 0*/
  wgsllh2ecef(lgf_llh_tmp, lgf_ecef_tmp);
  /* calc SV elevation and azimuth for LGF at user time */
  double elevation_lgf = 0;
  double azimuth_lgf = 0;
  calc_sat_az_el(
      config->e, &user_time, lgf_ecef_tmp, &azimuth_lgf, &elevation_lgf, false);
  log_debug_sid(config->e->sid,
                "LGF: elev_deg, azi_deg = %lf, %lf",
                elevation_lgf * R2D,
                azimuth_lgf * R2D);

  /* Determine the worst case SV elevation
   * due to the worst case direction of user movement */
  double ned[3];
  ned[0] = sin(azimuth_lgf + M_PI) * sin(arc_angle) * SV_VIS_EARTH_MIN_RADIUS;
  ned[1] = cos(azimuth_lgf + M_PI) * sin(arc_angle) * SV_VIS_EARTH_MIN_RADIUS;
  ned[2] = (1 - cos(arc_angle)) * SV_VIS_EARTH_MIN_RADIUS;
  double worst_case_ecef[3] = {0};
  wgsned2ecef_d(ned, lgf_ecef_tmp, worst_case_ecef);

  double elevation_user = 0;
  double azimuth_user = 0;
  /* Calc SV elevation at worst-case reference point and
   * at user time */
  calc_sat_az_el(config->e,
                 &user_time,
                 worst_case_ecef,
                 &azimuth_user,
                 &elevation_user,
                 false);
  log_debug_sid(config->e->sid,
                "USER: elev_deg, azi_deg = %lf, %lf",
                elevation_user * R2D,
                azimuth_user * R2D);
  elevation_user *= R2D;
  elevation_lgf *= R2D;
  double elevation_delta = elevation_lgf - elevation_user;

  double elev_mask = get_solution_elevation_mask();
  if (elevation_user > elev_mask) {
    *visible = true;
    *known = true;
    return;
  }

  if ((elevation_user <= elev_mask) &&
      elevation_delta <= SV_VIS_MAX_UNKNOWN_ANGLE) {
    *visible = false;
    *known = true;
    return;
  }

  if ((elevation_user <= elev_mask) &&
      elevation_delta > SV_VIS_MAX_UNKNOWN_ANGLE) {
    *visible = false;
    *known = false;
    return;
  }
}
