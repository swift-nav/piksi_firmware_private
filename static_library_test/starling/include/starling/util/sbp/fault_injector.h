//
// Copyright (C) 2019 Swift Navigation Inc.
// Contact: Swift Navigation <dev@swiftnav.com>
//
// This source is subject to the license found in the file 'LICENSE' which must
// be distributed together with this source. All other rights reserved.
//
// THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
// EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
//

#ifndef STARLING_FAULT_INJECTOR_H
#define STARLING_FAULT_INJECTOR_H

#include <libsbp/common.h>
#include <libsbp/cpp/message_handler.h>
#include <pvt_engine/optional.h>

#include <iomanip>
#include <iostream>
#include <utility>

#include "fault_chrono.h"
#include "fault_injector/dropouts_chrono.h"

namespace starling {
namespace fault_injector {

class Injector
    : private sbp::MessageHandler<msg_gps_time_t, msg_pos_ecef_cov_gnss_t,
                                  msg_imu_raw_t, msg_imu_aux_t,
                                  msg_vel_ecef_cov_gnss_t, msg_dops_t> {
 public:
  Injector(sbp::State *state)
      : sbp::MessageHandler<msg_gps_time_t, msg_pos_ecef_cov_gnss_t,
                            msg_imu_raw_t, msg_imu_aux_t,
                            msg_vel_ecef_cov_gnss_t, msg_dops_t>(state),
        state_{state} {}
  // ************************* SBP_MSG_GPS_TIME *********************
  void handle_sbp_msg(uint16_t sender_id, uint8_t message_length,
                      const msg_gps_time_t &msg) override;

  // ************************* SBP_POS_COV_ECEF *********************
  void handle_sbp_msg(uint16_t sender_id, uint8_t message_length,
                      const msg_pos_ecef_cov_gnss_t &msg) override;

  // ************************* SBP_MSG_IMU_RAW **********************
  void handle_sbp_msg(uint16_t sender_id, uint8_t message_length,
                      const msg_imu_raw_t &msg) override;

  // ************************* SBP_MSG_IMU_AUX **********************
  void handle_sbp_msg(uint16_t sender_id, uint8_t message_length,
                      const msg_imu_aux_t &msg) override;

  // ************************* SBP_VEL_ECEF_COV *********************
  void handle_sbp_msg(uint16_t sender_id, uint8_t message_length,
                      const msg_vel_ecef_cov_gnss_t &msg) override;

  // ************************* SBP_MSG_DOPS *************************
  void handle_sbp_msg(uint16_t sender_id, uint8_t message_length,
                      const msg_dops_t &msg) override;

 protected:
  template <typename T>
  void send_msg(uint16_t sender_id, uint8_t message_length, const T &msg) {
    constexpr uint16_t msg_type = sbp::MessageTraits<T>::id;
    state_->send_message(msg_type, sender_id, message_length,
                         reinterpret_cast<const u8 *>(&msg));
  }

 private:
  sbp::State *state_;
};

// if we start mixing dropouts with multiple periods, durations and multiple sbp
// messages we could build a jump table of DropOut<-->NotDropOut states using
// our FSM lib
class GnssDropoutInjector : public Injector {
 public:
  GnssDropoutInjector(sbp::State *state, DropoutsChrono *chrono)
      : Injector(state), chrono_{chrono} {}

  // ************************* SBP_MSG_GPS_TIME *********************
  void handle_sbp_msg(uint16_t sender_id, uint8_t message_length,
                      const msg_gps_time_t &msg) override;

  // ************************* SBP_POS_COV_ECEF *********************
  void handle_sbp_msg(uint16_t sender_id, uint8_t message_length,
                      const msg_pos_ecef_cov_gnss_t &msg) override;

 private:
  DropoutsChrono *chrono_;
};

class FilterResetInjector : public Injector {
 public:
  FilterResetInjector(sbp::State *state, double interval)
      : Injector(state), interval_{interval} {}

  // ************************* SBP_MSG_GPS_TIME *********************
  void handle_sbp_msg(uint16_t sender_id, uint8_t message_length,
                      const msg_gps_time_t &msg) override;

 private:
  double interval_;
  std::experimental::optional<gps_time_t> last_reset_;
};
}  // namespace fault_injector
}  // namespace starling

#endif  // STARLING_FAULT_INJECTOR_H
