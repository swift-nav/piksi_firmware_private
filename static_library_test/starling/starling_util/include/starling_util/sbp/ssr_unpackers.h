/*
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_UTIL_SBP_SSR_UNPACKERS_H
#define STARLING_UTIL_SBP_SSR_UNPACKERS_H

#include <libsbp/v4/ssr.h>
#include <pvt_engine/RTKLib_apriori_models/rtklib_common_antenna.h>
#include <pvt_engine/ssr_corrections.h>
#include <starling/build/config.h>
#include <swiftnav/signal.h>

namespace starling {
namespace util {
namespace sbp {

static constexpr double ORBIT_RADIAL_MULTIPLIER = 1e-4;
static constexpr double ORBIT_ALONG_MULTIPLIER = 4e-4;
static constexpr double ORBIT_CROSS_MULTIPLIER = 4e-4;
static constexpr double ORBIT_DOT_RADIAL_MULTIPLIER = 1e-6;
static constexpr double ORBIT_DOT_ALONG_MULTIPLIER = 4e-6;
static constexpr double ORBIT_DOT_CROSS_MULTIPLIER = 4e-6;

static constexpr double CLOCK_C0_MULTIPLIER = 1e-4;
static constexpr double CLOCK_C1_MULTIPLIER = 1e-6;
static constexpr double CLOCK_C2_MULTIPLIER = 2e-8;

static constexpr double CODE_BIASES_MULTIPLIER = 1e-2;
static constexpr double PHASE_BIASES_MULTIPLIER = 1e-4;

static constexpr double YAW_MULTIPLIER = 256;

static constexpr u8 NUM_SSR_UPDATE_INTERVALS = 16;
static constexpr u16 INVALID_SSR_UPDATE_INTERVAL = 5;

static constexpr u16 MM_PER_METER = 1000;
static constexpr u16 DECI = 10;  // technically, this should be called DECA

static constexpr u16 ssr_update_interval_values[NUM_SSR_UPDATE_INTERVALS] = {
    1, 2, 5, 10, 15, 30, 60, 120, 240, 300, 600, 900, 1800, 3600, 7200, 10800,
};

struct SsrStecMessageTraits {
  struct Storage {
    static constexpr size_t MAX_SAT_ELEMENTS =
        SBP_MSG_SSR_STEC_CORRECTION_STEC_SAT_LIST_MAX;

    Storage() : sat_elements(), sat_count(0) {}

    explicit Storage(const sbp_msg_ssr_stec_correction_t &body)
        : sat_elements(), sat_count(0) {
      sat_count = body.n_stec_sat_list;
      assert(sat_count <= MAX_SAT_ELEMENTS);
      std::copy(&body.stec_sat_list[0], &body.stec_sat_list[sat_count],
                sat_elements);
    }

    Storage(const Storage &other) : sat_elements(), sat_count(other.sat_count) {
      std::copy(&other.sat_elements[0], &other.sat_elements[sat_count],
                sat_elements);
    }

    Storage &operator=(const Storage &other) = default;

    sbp_stec_sat_element_t sat_elements[MAX_SAT_ELEMENTS];
    size_t sat_count;
  };

  using Header = sbp_stec_header_t;
  using Message = sbp_msg_ssr_stec_correction_t;
  static constexpr int32_t kMaxMessageCount = cNumSat;
  static constexpr const char *kMessageName = "STEC";
};

struct SsrGriddedCorrectionMessageTraits {
  struct Storage {
    Storage()
        : index(0),
          tropo_delay_correction(),
          stec_residuals(),
          residual_count(0) {}

    explicit Storage(const sbp_msg_ssr_gridded_correction_t &body)
        : index(body.index),
          tropo_delay_correction(body.tropo_delay_correction),
          stec_residuals(),
          residual_count(0) {
      residual_count = body.n_stec_residuals;
      std::copy(&body.stec_residuals[0], &body.stec_residuals[residual_count],
                stec_residuals);
    }

    Storage(const Storage &other)
        : index(other.index),
          tropo_delay_correction(other.tropo_delay_correction),
          stec_residuals(),
          residual_count(other.residual_count) {
      std::copy(&other.stec_residuals[0], &other.stec_residuals[residual_count],
                stec_residuals);
    }

    Storage &operator=(const Storage &other) = default;

    uint16_t index;
    sbp_tropospheric_delay_correction_t tropo_delay_correction;
    sbp_stec_residual_t
        stec_residuals[SBP_MSG_SSR_GRIDDED_CORRECTION_STEC_RESIDUALS_MAX];
    size_t residual_count;
  };
  using Message = sbp_msg_ssr_gridded_correction_t;
  using Header = sbp_gridded_correction_header_t;
  static constexpr int32_t kMaxMessageCount = pvt_engine::MAX_SSR_GRID_POINTS;
  static constexpr const char *kMessageName = "Gridded Correction";
};

template <typename MessageTrait>
class SsrMessageAggregator {
 public:
  using Message = typename MessageTrait::Message;
  using MessageHeader = typename MessageTrait::Header;
  using MessageContents =
      pvt_common::containers::StaticVector<typename MessageTrait::Storage,
                                           MessageTrait::kMaxMessageCount>;

  SsrMessageAggregator()
      : expected_number_of_messages_(0),
        received_number_of_messages_(0),
        header_(),
        contents_(0) {}

  bool is_started() const { return (expected_number_of_messages_ != 0); }

  bool is_complete() const {
    return is_started() &&
           (expected_number_of_messages_ == received_number_of_messages_);
  }

  const MessageHeader &get_header() const {
    assert(header_.has_value());
    return *header_;
  }

  const MessageContents &get_message_bodies() const { return contents_; }

  u16 get_received_message_count() const {
    return received_number_of_messages_;
  }

  void reset() {
    expected_number_of_messages_ = 0;
    received_number_of_messages_ = 0;
    header_.reset();
    contents_.resize(0);
  }

  void add_message(const Message &msg) {
    if (is_started() && !is_complete() &&
        (msg.header.time.tow != header_->time.tow ||
         msg.header.time.wn != header_->time.wn ||
         msg.header.iod_atmo != header_->iod_atmo)) {
      log_warn(
          "Got a new %s message while the previous train is not complete, "
          "resetting. Expected %u messages, recieved %u. New time: {%u, %u} "
          "New IOD: %u, Prev time {%u, %u} Prev IOD: %u",
          MessageTrait::kMessageName,
          static_cast<unsigned>(expected_number_of_messages_),
          static_cast<unsigned>(received_number_of_messages_),
          static_cast<unsigned>(msg.header.time.wn),
          static_cast<unsigned>(msg.header.time.tow),
          static_cast<unsigned>(msg.header.iod_atmo),
          static_cast<unsigned>(header_->time.wn),
          static_cast<unsigned>(header_->time.tow),
          static_cast<unsigned>(header_->iod_atmo));
      reset();

      expected_number_of_messages_ = msg.header.num_msgs;
      header_ = msg.header;
    }

    if (expected_number_of_messages_ == 0 && msg.header.seq_num == 0) {
      header_ = msg.header;
      expected_number_of_messages_ = msg.header.num_msgs;
    } else if (msg.header.seq_num != received_number_of_messages_) {
      log_warn(
          "Got a %s message out of order resetting. Expected #%u but got #%u",
          MessageTrait::kMessageName, received_number_of_messages_ + 1,
          msg.header.seq_num);
      reset();
      return;
    }

    contents_.push_back(typename MessageTrait::Storage(msg));
    received_number_of_messages_++;
  }

 private:
  u16 expected_number_of_messages_;
  u16 received_number_of_messages_;
  optional<MessageHeader> header_;
  MessageContents contents_;
};

using SsrStecMessageAggregator = SsrMessageAggregator<SsrStecMessageTraits>;
using SsrGriddedCorrectionMessageAggregator =
    SsrMessageAggregator<SsrGriddedCorrectionMessageTraits>;

u16 get_ssr_interval_seconds(const u8 &index);

u8 get_rtcm_df391_index(const u16 &seconds);

pvt_engine::PRC is_valid_ssr_update_interval(const u16 &seconds);

void unpack_ssr_orbit_clock_content(
    const sbp_msg_ssr_orbit_clock_t &msg, gnss_signal_t *sid,
    pvt_engine::OrbitCorrection *orbit_correction,
    pvt_engine::ClockCorrection *clock_correction);

void unpack_ssr_code_biases_content(const sbp_msg_ssr_code_biases_t &msg,
                                    gnss_signal_t *sid,
                                    pvt_engine::CodeBiases *code_biases);

void unpack_ssr_phase_biases_content(const sbp_msg_ssr_phase_biases_t &msg,
                                     gnss_signal_t *sid,
                                     pvt_engine::PhaseBiases *phase_biases);

void unpack_ssr_gridded_atmo_content(
    const SsrStecMessageAggregator &stec_messages,
    const SsrGriddedCorrectionMessageAggregator &gridded_atmo_messages,
    const pvt_engine::SsrGrid &grid, pvt_engine::GriddedAtmo *gridded_atmo);

pvt_engine::SsrGrid unpack_ssr_tile_definition_content(
    const sbp_msg_ssr_tile_definition_t &msg);

bool ssr_headers_match(const sbp_stec_header_t &stec_header,
                       const sbp_gridded_correction_header_t &gridded_header,
                       const pvt_engine::SsrGrid &tile);

void unpack_ssr_satellite_apc_content(const sbp_msg_ssr_satellite_apc_t &msg,
                                      pvt_engine::SatellitePCVMap *sat_apc);

}  // namespace sbp
}  // namespace util
}  // namespace starling

#endif  // STARLING_UTIL_SBP_SSR_UNPACKERS_H
