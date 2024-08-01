/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Michele Bavaro <michele@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_INTEGRATION_CORE_SBP_IN_H
#define STARLING_INTEGRATION_CORE_SBP_IN_H

#include <libsbp/sbp.h>

namespace starling {
namespace integration {

enum class MsgSource {
  LOCAL,
  REMOTE,
};

struct ObsSbpNodes {
  sbp_msg_callbacks_node_t obs;
  sbp_msg_callbacks_node_t osr;
};

struct EphSbpNodes {
  sbp_msg_callbacks_node_t gps;
  sbp_msg_callbacks_node_t glo;
  sbp_msg_callbacks_node_t gal;
  sbp_msg_callbacks_node_t gal_dep_a;
  sbp_msg_callbacks_node_t bds;
};

struct PosSbpNodes {
  sbp_msg_callbacks_node_t llh;
  sbp_msg_callbacks_node_t ecef;
};

struct ImuSbpNodes {
  sbp_msg_callbacks_node_t aux;
  sbp_msg_callbacks_node_t raw;
};

struct SettingsSbpNodes {
  sbp_msg_callbacks_node_t read_by_index;
  sbp_msg_callbacks_node_t write;
  sbp_msg_callbacks_node_t reset_filters;
};

struct HeartbeatSbpNodes {
  sbp_msg_callbacks_node_t heartbeat;
};

struct SsrSbpNodes {
  sbp_msg_callbacks_node_t orbit_clock;
  sbp_msg_callbacks_node_t code_biases;
  sbp_msg_callbacks_node_t phase_biases;
  sbp_msg_callbacks_node_t stec_correction;
  sbp_msg_callbacks_node_t gridded_correction;
  sbp_msg_callbacks_node_t gridded_correction_no_std;
  sbp_msg_callbacks_node_t grid_definiton;
};

struct StationSbpNodes {
  ObsSbpNodes obs;
  EphSbpNodes eph;
  PosSbpNodes pos;
  ImuSbpNodes imu;
  SsrSbpNodes ssr;
};

void install_obs_sbp_callbacks(sbp_state_t *state, ObsSbpNodes *nodes,
                               MsgSource msg_source);
void install_eph_sbp_callbacks(sbp_state_t *state, EphSbpNodes *nodes,
                               MsgSource msg_source);
void install_pos_sbp_callbacks(sbp_state_t *state, PosSbpNodes *nodes,
                               MsgSource msg_source);
void install_imu_sbp_callbacks(sbp_state_t *state, ImuSbpNodes *nodes,
                               MsgSource msg_source);

void install_settings_sbp_callbacks(sbp_state_t *state, SettingsSbpNodes *nodes,
                                    MsgSource msg_source);

void install_heartbeat_sbp_callbacks(sbp_state_t *state,
                                     HeartbeatSbpNodes *nodes,
                                     MsgSource msg_source);

void install_sbp_callbacks(sbp_state_t *state, StationSbpNodes *nodes,
                           MsgSource msg_source);

void sbp_in(u16 sender_id, u16 msg_type, u8 len, u8 msg[],
            MsgSource msg_source);

}  // namespace integration
}  // namespace starling

#endif
