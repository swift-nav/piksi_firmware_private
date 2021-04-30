/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "calc_pvt_common.h"

#include <pvt_engine/firmware_binding.h>

#include "sbp/sbp.h"
#include "sbp/sbp_utils.h"

void send_observations(const obs_array_t *obs_array, u32 msg_obs_max_size) {
  assert(obs_array);
  static u8 buff[SBP_FRAMING_MAX_PAYLOAD_SIZE + 1];
  msg_obs_t *msg = (msg_obs_t *)&buff;

  u8 n = obs_array->n;

  if (0 == n) {
    /* Send empty observations */
    pack_obs_header(&obs_array->t, 1, 0, &msg->header);
    sbp_send_msg(SBP_MSG_OBS, sizeof(observation_header_t), buff);
    return;
  }

  /* Upper limit set by SBP framing size, preventing underflow */
  u16 msg_payload_size =
      MAX(MIN((u16)MAX(msg_obs_max_size, 0), SBP_FRAMING_MAX_PAYLOAD_SIZE),
          sizeof(observation_header_t)) -
      sizeof(observation_header_t);

  /* Lower limit set by sending at least 1 observation */
  msg_payload_size = MAX(msg_payload_size, sizeof(packed_obs_content_t));

  /* Round down the number of observations per message */
  u16 obs_in_msg = msg_payload_size / sizeof(packed_obs_content_t);

  /* Round up the number of messages */
  u16 total = MIN((n + obs_in_msg - 1) / obs_in_msg, MSG_OBS_HEADER_MAX_SIZE);

  u8 obs_i = 0;
  for (u8 count = 0; count < total; count++) {
    u8 curr_n = MIN(n - obs_i, obs_in_msg);
    pack_obs_header(&obs_array->t, total, count, &msg->header);

    for (u8 i = 0; i < curr_n; i++, obs_i++) {
      const starling_obs_t *m = &obs_array->observations[obs_i];
      const navigation_measurement_t nm = {
          .raw_pseudorange = m->pseudorange,
          .raw_carrier_phase = m->carrier_phase,
          .raw_measured_doppler = m->doppler,
          .cn0 = m->cn0,
          .lock_time = m->lock_time,
          .flags = m->flags,
          .sid = m->sid};
      if (pack_obs_content(&nm, &msg->obs[i]) < 0) {
        /* Error packing this observation, skip it. */
        i--;
        curr_n--;
      }
    }

    sbp_send_msg(
        SBP_MSG_OBS,
        sizeof(observation_header_t) + curr_n * sizeof(packed_obs_content_t),
        buff);
  }
}

bool get_max_sats(double soln_freq_hz,
                  pvt_driver_solution_mode_t soln_mode,
                  s32 *max_sats) {
  static int NUM_OBSERVATIONS = 6;
  static double soln_freq_arr[] = {20.0, 10.0, 5.0, 4.0, 2.0, 1.0};
  static int max_sats_arr[] = {5, 15, 22, 22, 22, 22};
  static int max_sats_time_matched_arr[] = {5, 6, 13, 17, 22, 22};

  for (int i = 0; i < NUM_OBSERVATIONS; i++) {
    if (fabs(soln_freq_hz - soln_freq_arr[i]) < FLOAT_EQUALITY_EPS) {
      if (soln_mode == PVT_DRIVER_SOLN_MODE_TIME_MATCHED) {
        *max_sats = max_sats_time_matched_arr[i];
      } else {
        *max_sats = max_sats_arr[i];
      }
      return true;
    }
  }

  log_warn(
      "Input solution frequency %.2f Hz outside acceptable range [%.2f, %.2f]",
      soln_freq_hz,
      1.0 - FLOAT_EQUALITY_EPS,
      20.0 + FLOAT_EQUALITY_EPS);
  return false;
}
