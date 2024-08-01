//
// Copyright (C) 2019 Swift Navigation Inc.
// Contact: Swift Navigation <dev@swiftnav.com>
//
// This source is subject to the license found in the file 'LICENSE' which must
// be be distributed together with this source. All other rights reserved.
//
// THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
// EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
//

#include <libsbp/common.h>
#include <libsbp/cpp/message_handler.h>
#include <pvt_engine/optional.h>
#include <pvt_engine/vehicle_dynamics_filter.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/pvt_result.h>
#include <iomanip>
#include <iostream>
#include <utility>
#include "sbp_io_fuzer_helpers.h"

#ifndef STARLING_SBP_IO_FUZER_H
#define STARLING_SBP_IO_FUZER_H

namespace starling {
namespace fuzer {

constexpr int gPrimaryStreamId = 4096;
constexpr int gSecondaryStreamId = 61568;
constexpr int gFuzedStreamId = 123;
constexpr double gEHPEThreshold = 3.2;
constexpr bool gMakeFuzedOutputFlashyGreen = true;

class SbpIOFuzer : private sbp::MessageHandler<msg_gps_time_t, msg_pos_llh_t> {
 public:
  SbpIOFuzer(sbp::State *state, pvt_engine::VehicleDynamicsFilter *filter,
             const int32_t &output_sol)
      : sbp::MessageHandler<msg_gps_time_t, msg_pos_llh_t>(state),
        state_{state},
        filter_{filter},
        output_sol_{output_sol},
        current_gnss_time() {}

  // ************************* SBP_MSG_GPS_TIME *********************
  void handle_sbp_msg(uint16_t sender_id, uint8_t message_length,
                      const msg_gps_time_t &msg) override;

  // ************************* SBP_MSG_POS_LLH **********************
  void handle_sbp_msg(uint16_t sender_id, uint8_t message_length,
                      const msg_pos_llh_t &msg) override;

 public:
  sbp::State *state_;

  typedef struct {
    optional<pvt_engine_result_t> pos_llh_primary;
    optional<pvt_engine_result_t> pos_llh_secondary;

    void reset() {
      pos_llh_primary.reset();
      pos_llh_secondary.reset();
    }

    CircularMapBuffer<u32, pvt_engine_result_t, 25> primary_circular_buffer;
    CircularMapBuffer<u32, pvt_engine_result_t, 2> secondary_circular_buffer;

    void prepare_fuzer_data() {
      // circular buffer key is tow in ms
      // pick the first (oldest) secondary solution
      // note that some solutions do a round-to-nearest half-sec
      // This allows starling solutions to come even after their time-matched
      // secondary counterpart, but before the next secondary solution
      if (secondary_circular_buffer.size() < 1) {
        pos_llh_secondary.reset();
        return;
      } else {  // NOLINT
        pos_llh_secondary.emplace(secondary_circular_buffer.begin()->second);
      }

      detailed_log_debug(
          "%d  SEARCHING CIRC BUFFER",
          static_cast<u32>((*pos_llh_secondary).time.tow * 1000));

      auto maybe_primary_sol = primary_circular_buffer.find(round_nearest_100ms(
          static_cast<u32>((*pos_llh_secondary).time.tow * 1000)));
      if (maybe_primary_sol != primary_circular_buffer.end()) {
        detailed_log_debug(
            "%d  USING CIRC-BUFFERED PRIMARY",
            static_cast<u32>((*pos_llh_secondary).time.tow * 1000));

        pos_llh_primary.emplace(maybe_primary_sol->second);
      } else {
        detailed_log_debug(
            "%d  NOTHING IN CIRC-BUFFER",
            static_cast<u32>((*pos_llh_secondary).time.tow * 1000));

        pos_llh_primary.emplace(pos_llh_secondary.value());
      }
    }

    optional<std::array<pvt_engine_result_t, 2>> get_combined_input_data() {
      if (pos_llh_primary.has_value() && pos_llh_secondary.has_value()) {
        std::array<pvt_engine_result_t, 2> ret(
            {pos_llh_primary.value(), pos_llh_secondary.value()});
        return ret;
      } else {  // NOLINT
        return {};
      }
    };

  } fuzer_buffer;

  fuzer_buffer buffer_;

 protected:
  optional<msg_pos_llh_t> handle_secondary_sbp_msg(
      const msg_pos_llh_t &msg, const pvt_engine_result_t &pvt_engine_result);
  optional<msg_pos_llh_t> handle_primary_sbp_msg(
      const msg_pos_llh_t &msg, const pvt_engine_result_t &pvt_engine_result);

 private:
  pvt_engine::VehicleDynamicsFilter *filter_;
  const int32_t output_sol_;
  gps_time_t current_gnss_time;
};

}  // namespace fuzer
}  // namespace starling

#endif  // STARLING_SBP_IO_FUZER_H
