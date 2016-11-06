/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Leith Bade <leith@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <string.h>

#include <libswiftnav/coord_system.h>
#include <libswiftnav/linear_algebra.h>

#include "soln_output.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "main.h"

typedef enum {
  GPS_TIME_NONE,
  GPS_TIME_PROPAGATED,
  GPS_TIME_SPP
} gps_time_source_t;

typedef enum {
  POSITION_NONE,
  POSITION_SPP,
  POSITION_RTK
} position_source_t;

void send_solutions(const send_solutions_t *in) {
  /* Choose GPS time source */
  /* Prefers SPP over propagated time */
  gps_time_t best_gps_time;
  gps_time_source_t best_gps_time_src = GPS_TIME_NONE;
  if (in->rec_time_valid) {
    best_gps_time = in->rec_time;
    best_gps_time_src = GPS_TIME_PROPAGATED;
  }
  if (in->spp_time_valid) {
    best_gps_time = in->spp_time;
    best_gps_time_src = GPS_TIME_SPP;
  }

  /* Send GPS time message first */
  if (best_gps_time_src != GPS_TIME_NONE) {
    msg_gps_time_t gps_time;
    sbp_make_gps_time(&gps_time, &best_gps_time, 0);
    sbp_send_msg(SBP_MSG_GPS_TIME, sizeof(gps_time), (u8 *) &gps_time);
  } else {
    /* Everything else needs a time stamp */
    return;
  }

  /* TODO(Leith): UTC conversion goes here */

  /* Choose position source */
  /* Prefers RTK pseudo-absolute over SPP */
  double best_pos_ecef[3];
  u8 best_pos_num_sats;
  position_source_t best_pos_src = POSITION_NONE;
  if (in->spp_position_valid) {
    memcpy(best_pos_ecef, in->spp_position, sizeof(best_pos_ecef));
    best_pos_src = POSITION_SPP;
    best_pos_num_sats = in->spp_position_num_sats;
  }
  if (in->base_position_valid && in->rtk_baseline_valid) {
    vector_add(3, in->base_position, in->rtk_baseline, best_pos_ecef);
    best_pos_src = POSITION_RTK;
    best_pos_num_sats = in->rtk_baseline_num_sats;
  }

  /* Send position messages */
  if (best_pos_src != POSITION_NONE) {
    double best_pos_llh[3];
    wgsecef2llh(best_pos_ecef, best_pos_llh);

    /* Encode the SBP flags value */
    u8 pos_flags = MSG_POS_SPP;
    if (POSITION_RTK) {
      if (in->rtk_baseline_fixed) {
        pos_flags = MSG_POS_RTK_FIXED;
      } else {
        pos_flags = MSG_POS_RTK_FLOAT;
      }
    }
    /* TODO(Leith): implement the RAIM flags? */

    msg_pos_llh_t pos_llh;
    sbp_make_pos_llh_vect(&pos_llh, best_pos_llh, &best_gps_time,
                          best_pos_num_sats, pos_flags);
    sbp_send_msg(SBP_MSG_POS_LLH, sizeof(pos_llh), (u8 *) &pos_llh);
    msg_pos_ecef_t pos_ecef;
    sbp_make_pos_ecef_vect(&pos_ecef, best_pos_ecef, &best_gps_time,
                           best_pos_num_sats, pos_flags);
    sbp_send_msg(SBP_MSG_POS_ECEF, sizeof(pos_ecef), (u8 *) &pos_ecef);
  }

  /* Send velocity messages */
  if (in->spp_velocity_valid) {
    /* Absolute position needed to convert to NED */
    if (best_pos_src != POSITION_NONE) {
      double v_ned[3];
      wgsecef2ned(in->spp_velocity, best_pos_ecef, v_ned);
      msg_vel_ned_t vel_ned;
      sbp_make_vel_ned_vect(&vel_ned, v_ned, &best_gps_time,
                            in->spp_velocity_num_sats, 0);
      sbp_send_msg(SBP_MSG_VEL_NED, sizeof(vel_ned), (u8 *) &vel_ned);
    }

    msg_vel_ecef_t vel_ecef;
    sbp_make_vel_ecef_vect(&vel_ecef, in->spp_velocity, &best_gps_time,
                           in->spp_velocity_num_sats, 0);
    sbp_send_msg(SBP_MSG_VEL_ECEF, sizeof(vel_ecef), (u8 *) &vel_ecef);
  }

  /* Send baseline messages */
  if (in->rtk_baseline_valid) {
    /* Encode the SBP flags value */
    u8 baseline_flags =
      in->rtk_baseline_fixed ? MSG_BASELINE_FIXED : MSG_BASELINE_FLOAT;
    /* TODO(Leith): implement the RAIM flags? */

    msg_baseline_ecef_t baseline_ecef;
    sbp_make_baseline_ecef(&baseline_ecef, &best_gps_time,
                           in->rtk_baseline_num_sats, in->rtk_baseline,
                           baseline_flags);
    sbp_send_msg(SBP_MSG_BASELINE_ECEF, sizeof(baseline_ecef),
                 (u8 *)&baseline_ecef);

    /* Absolute position needed to convert to NED */
    if (best_pos_src != POSITION_NONE) {
      double b_ned[3];
      wgsecef2ned(in->rtk_baseline, best_pos_ecef, b_ned);

      msg_baseline_ned_t baseline_ned;
      sbp_make_baseline_ned(&baseline_ned, &best_gps_time,
                            in->rtk_baseline_num_sats, b_ned, baseline_flags);
      sbp_send_msg(SBP_MSG_BASELINE_NED, sizeof(baseline_ned),
                   (u8 *)&baseline_ned);
    }
  }

  /* TODO(Leith): heading should be from velocity */
  /*if (send_heading) {
    double heading = calc_heading(b_ned);
    msg_baseline_heading_t sbp_heading;
    sbp_make_heading(&sbp_heading, t, heading, n_sats, flags);
    sbp_send_msg(SBP_MSG_BASELINE_HEADING, sizeof(sbp_heading), (u8 *)&sbp_heading);
  }*/

  /* Send DOPs message */
  if (in->spp_dops_valid) {
    DO_EVERY(10,
      msg_dops_t sbp_dops;
      sbp_make_dops(&sbp_dops, &in->spp_dops, &best_gps_time);
      sbp_send_msg(SBP_MSG_DOPS, sizeof(msg_dops_t), (u8 *) &sbp_dops);
    );
  }

  // TODO: NMEA
}
