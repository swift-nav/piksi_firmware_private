/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Kevin Dade <kevin@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "starling_sbp_output.h"

/*********************************************************************
 * External Dependencies -- TODO(kevin) remove these.
 ********************************************************************/
extern s8 sbp_send_msg(u16 msg, u8 n, u8 *data);
extern bool send_heading;

/*********************************************************************
 * Solution Message Generation Helpers
 ********************************************************************/

/**
 *
 * @param sbp_messages struct of sbp messages
 *
 */
void solution_send_pos_messages(const sbp_messages_t *sbp_messages) {
  dgnss_solution_mode_t dgnss_soln_mode = starling_get_solution_mode();
  if (sbp_messages) {
    sbp_send_msg(SBP_MSG_GPS_TIME,
                 sizeof(sbp_messages->gps_time),
                 (u8 *)&sbp_messages->gps_time);
    sbp_send_msg(SBP_MSG_UTC_TIME,
                 sizeof(sbp_messages->utc_time),
                 (u8 *)&sbp_messages->utc_time);
    sbp_send_msg(SBP_MSG_POS_LLH,
                 sizeof(sbp_messages->pos_llh),
                 (u8 *)&sbp_messages->pos_llh);
    sbp_send_msg(SBP_MSG_POS_ECEF,
                 sizeof(sbp_messages->pos_ecef),
                 (u8 *)&sbp_messages->pos_ecef);
    sbp_send_msg(SBP_MSG_VEL_NED,
                 sizeof(sbp_messages->vel_ned),
                 (u8 *)&sbp_messages->vel_ned);
    sbp_send_msg(SBP_MSG_VEL_ECEF,
                 sizeof(sbp_messages->vel_ecef),
                 (u8 *)&sbp_messages->vel_ecef);
    sbp_send_msg(SBP_MSG_DOPS,
                 sizeof(sbp_messages->sbp_dops),
                 (u8 *)&sbp_messages->sbp_dops);
    sbp_send_msg(SBP_MSG_POS_ECEF_COV,
                 sizeof(sbp_messages->pos_ecef_cov),
                 (u8 *)&sbp_messages->pos_ecef_cov);
    sbp_send_msg(SBP_MSG_VEL_ECEF_COV,
                 sizeof(sbp_messages->vel_ecef_cov),
                 (u8 *)&sbp_messages->vel_ecef_cov);
    sbp_send_msg(SBP_MSG_POS_LLH_COV,
                 sizeof(sbp_messages->pos_llh_cov),
                 (u8 *)&sbp_messages->pos_llh_cov);
    sbp_send_msg(SBP_MSG_VEL_NED_COV,
                 sizeof(sbp_messages->vel_ned_cov),
                 (u8 *)&sbp_messages->vel_ned_cov);

    if (dgnss_soln_mode != STARLING_SOLN_MODE_NO_DGNSS) {
      sbp_send_msg(SBP_MSG_BASELINE_ECEF,
                   sizeof(sbp_messages->baseline_ecef),
                   (u8 *)&sbp_messages->baseline_ecef);
    }

    if (dgnss_soln_mode != STARLING_SOLN_MODE_NO_DGNSS) {
      sbp_send_msg(SBP_MSG_BASELINE_NED,
                   sizeof(sbp_messages->baseline_ned),
                   (u8 *)&sbp_messages->baseline_ned);
    }

    sbp_send_msg(SBP_MSG_AGE_CORRECTIONS,
                 sizeof(sbp_messages->age_corrections),
                 (u8 *)&sbp_messages->age_corrections);

    if (dgnss_soln_mode != STARLING_SOLN_MODE_NO_DGNSS) {
      sbp_send_msg(SBP_MSG_DGNSS_STATUS,
                   sizeof(sbp_messages->dgnss_status),
                   (u8 *)&sbp_messages->dgnss_status);
    }

    if (send_heading && dgnss_soln_mode != STARLING_SOLN_MODE_NO_DGNSS) {
      sbp_send_msg(SBP_MSG_BASELINE_HEADING,
                   sizeof(sbp_messages->baseline_heading),
                   (u8 *)&sbp_messages->baseline_heading);
    }
  }
}

/******************************************************************************/
//void starling_connect_sbp_output(const SbpDuplexLink *sbp_output_link) {
//
//}
