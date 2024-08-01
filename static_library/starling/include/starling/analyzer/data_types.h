///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2019 Swift Navigation Inc.
// Contact: Swift Navigation <dev@swiftnav.com>
//
// This source is subject to the license found in the file 'LICENSE' which must
// be distributed together with this source. All other rights reserved.
//
// THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
// EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
///////////////////////////////////////////////////////////////////////////////

#ifndef STARLING_ANALYZER_DATA_TYPES_H
#define STARLING_ANALYZER_DATA_TYPES_H

#include <libsbp/navigation.h>
#include <libsbp/system.h>
#include <pvt_engine/eigen_types.h>
#include <iostream>
#include "metrics_errors.h"
#include "metrics_integrity.h"
#include "starling/analyzer/metric_interface.h"

// Metric Messages for Starling Analyzer, noted "MsG".
// Each MsG holds one metric. Typically, a MsG buffers necessary inputs, checks
// the buffer state, computes the metric when buffer's ready, stores the metrics
// and irreversibly flush-returns the metric to the caller. The typical flow is
// instantiate --> register inputs --> check buffer --> build message metric -->
// flush metric. When flushing is called but no metric was computed, "quiet"
// NaNs are returned and can be checked agaisnt further down the stream. Note
// there is a command-line argument available to ignore NaNs in the output
// stream. Every MsG holds a reference to its relevant Metrics class (e.g.
// MetricsError). Thus, MsG don't have the responsability for knowing where
// truth references are, only Metrics do.
namespace starling {
namespace analyzer {
namespace AnalyzerData {

constexpr auto ANALYZER_GPS_TIME_MSG_NAME = "analyzer_gnss_time";
constexpr auto ANALYZER_INS_STATUS_MSG_NAME = "analyzer_ins_status";
constexpr auto ANALYZER_ERROR_POS_ECEF_NAME = "AnalyzerErrorPosECEF";
constexpr auto ANALYZER_ERROR_POS_ECEF_COV_NAME = "analyzer_error_pos_ecef_cov";
constexpr auto ANALYZER_ERROR_POS_LLH_NAME = "analyzer_error_pos_llh";
constexpr auto ANALYZER_ERROR_POS_NED_NAME = "AnalyzerErrorPosNED";
constexpr auto ANALYZER_INTEGRITY_STATUS_NAME = "AnalyzerIntegrityStatus";
constexpr auto ANALYZER_ERROR_VEL_NED_NAME = "AnalyzerVelErrorNED";
constexpr auto ANALYZER_ERROR_VEL_NED_COV_NAME = "analyzer_error_vel_ned_cov";
constexpr auto ANALYZER_ZSCORE_VEL_NED = "AnalyzerZscoreVelNED";
constexpr auto ANALYZER_ERROR_COG_DEG_NAME = "AnalyzerErrorCogDeg";
constexpr auto ANALYZER_ERROR_HDG_DEG_NAME = "AnalyzerErrorHdgDeg";
constexpr auto ANALYZER_ZSCORE_HDG_NAME = "AnalyzerZscoreHdg";

constexpr int NAVIGATION_FIX_MODE_MASK = 0x7;

// TODO (guillaume) add constexpr for col names in MsG ctors

// Base class for MsG. Not super useful for now, only if we decide to build a
// version of starling_analyzer with ASIL-friendly dynamic polymorphism some day
struct AnalyzerMsg {
 public:
  AnalyzerMsg(const std::string &msg_name,
              const MetricInterface::metric_metadata_t &metric_metadata)
      : msg_name_(msg_name), metric_interface_(metric_metadata) {}

  explicit AnalyzerMsg(const std::string &msg_name_) : msg_name_(msg_name_) {}

  virtual ~AnalyzerMsg(){};

  virtual u16 build_message() noexcept = 0;

  // Every single inherited MsG must have a register overloaded for every input
  // type. Since we currently only work with SBP, they're all sbp-like. If we
  // switch to RTCM, add functions below for RTCM-like input types.
  virtual void register_to_buffer(
      const gps_time_t &ref_time,              // NOLINT
      const msg_gps_time_t &msg) noexcept {};  // NOLINT

  virtual void register_to_buffer(
      const gps_time_t &ref_time,              // NOLINT
      const msg_pos_ecef_t &msg) noexcept {};  // NOLINT

  virtual void register_to_buffer(
      const gps_time_t &ref_time,                // NOLINT
      const msg_ins_status_t &msg) noexcept {};  // NOLINT

  virtual void register_to_buffer(
      const gps_time_t &ref_time,                  // NOLINT
      const msg_pos_ecef_cov_t &msg) noexcept {};  // NOLINT

  virtual void register_to_buffer(
      const gps_time_t &ref_time,             // NOLINT
      const msg_pos_llh_t &msg) noexcept {};  // NOLINT

  virtual void register_to_buffer(
      const gps_time_t &ref_time,             // NOLINT
      const msg_vel_ned_t &msg) noexcept {};  // NOLINT

  virtual void register_to_buffer(
      const gps_time_t &ref_time,                 // NOLINT
      const msg_vel_ned_cov_t &msg) noexcept {};  // NOLINT

  virtual void register_to_buffer(
      const gps_time_t &ref_time,                  // NOLINT
      const msg_orient_euler_t &msg) noexcept {};  // NOLINT

  // Current GNSS epoch for the MsG, if available
  const std::experimental::optional<gps_time_t> get_message_time() const
      noexcept {
    return time_;
  };

  // Human-readable MsG name
  std::string get_msg_name() const { return msg_name_; }

  // Check if buffer is ready. Typically used by build_message() function
  bool is_ready() const { return is_ready_; }

  // Check if the metric interface has as many stored fields as field names in
  // its metadata
  bool check_metric_completeness() {
    if (metric_interface_.size() !=
        metric_interface_.num_nonempty_fields_metadata()) {
      std::cout << "===> Warning: missing fields in flushed metric "
                << metric_interface_.get_metric_name()
                << " may cause "
                   "rows with missing column values if Output to CSV"
                << std::endl;
      return false;
    }
    return true;
  }

 protected:
  MetricInterface metric_interface_;
  bool is_ready_ = false;
  const bool force_rebuild_ = false;

  const std::string msg_name_;
  std::experimental::optional<gps_time_t> time_;

 private:
  // Must implement specialized definitions
  // Make sure derived members are reset when flushing
  virtual const MetricInterface &flush_metric() noexcept = 0;
};

// This "metric" is simply gnss epoch time, for convenience when flushing all
// metrics into an output stream
struct AnalyzerGNSSMsg final : public AnalyzerMsg {
 public:
  explicit AnalyzerGNSSMsg()
      : AnalyzerMsg(ANALYZER_GPS_TIME_MSG_NAME,
                    {ANALYZER_GPS_TIME_MSG_NAME, {"wn", "tow"}}) {}

  u16 build_message() noexcept override;

  using AnalyzerMsg::register_to_buffer;

  void register_to_buffer(const gps_time_t &the_time,
                          const msg_gps_time_t &sbp_msg_) noexcept override;

  // important to call .reset() such that we don't have "value leaks" over
  // multiple epochs
  const MetricInterface &flush_metric() noexcept override;

 private:
  // input for the metric
  std::experimental::optional<msg_gps_time_t> msg_gps_time_;
};

// This "metric" is simply ins status, for convenience when flushing all
// metrics into an output stream
struct AnalyzerINSStatus final : public AnalyzerMsg {
 public:
  explicit AnalyzerINSStatus()
      : AnalyzerMsg(ANALYZER_INS_STATUS_MSG_NAME,
                    {ANALYZER_INS_STATUS_MSG_NAME, {"ins_status"}}) {}

  u16 build_message() noexcept override;

  using AnalyzerMsg::register_to_buffer;

  void register_to_buffer(const gps_time_t &the_time,
                          const msg_ins_status_t &sbp_msg_) noexcept override;

  // important to call .reset() such that we don't have "value leaks" over
  // multiple epochs
  const MetricInterface &flush_metric() noexcept override;

 private:
  // input for the metric
  std::experimental::optional<msg_ins_status_t> msg_ins_status_;
};

// MsG to compute 3D position error in ECEF.
// Inputs can be either an sbp msg_pos_ecef or msg_pos_llh
struct AnalyzerErrorPosECEF : public AnalyzerMsg {
 public:
  explicit AnalyzerErrorPosECEF(MetricsErrors &metrics_error)
      : AnalyzerMsg(ANALYZER_ERROR_POS_ECEF_NAME,
                    {ANALYZER_ERROR_POS_ECEF_NAME,
                     {"fix_mode", "error_pos_x", "error_pos_y", "error_pos_z",
                      "error_pos_norm"}}),
        msg_metric_(metrics_error) {}

  u16 build_message() noexcept override;

  // Help the compiler resolve names
  using AnalyzerMsg::register_to_buffer;

  void register_to_buffer(const gps_time_t &the_time,
                          const msg_pos_ecef_t &msg_ecef) noexcept override;

  // 3D position error in ECEF can also be computed from LLH position
  void register_to_buffer(const gps_time_t &the_time,
                          const msg_pos_llh_t &msg_llh) noexcept override;

  // Return fix mode (as per sbp standard) if available
  u8 get_fix_mode();

  // Again, important to reset values.
  const MetricInterface &flush_metric() noexcept override;

 private:
  std::experimental::optional<msg_pos_ecef_t> msg_pos_ecef_val;
  std::experimental::optional<msg_pos_llh_t> msg_pos_llh_val;
  // Convenient storage for 3D pos error
  std::experimental::optional<Eigen::Vector3d> msg_value_;

  MetricsErrors &msg_metric_;
};

// MsG to compute 3D position error in NED.
struct AnalyzerErrorPosNED : public AnalyzerMsg {
 public:
  explicit AnalyzerErrorPosNED(MetricsErrors &metrics_error)
      : AnalyzerMsg(ANALYZER_ERROR_POS_NED_NAME,
                    {ANALYZER_ERROR_POS_NED_NAME,
                     {"error_pos_n", "error_pos_e", "error_pos_d"}}),
        msg_metric_(metrics_error) {}

  u8 get_fix_mode();

  u16 build_message() noexcept override;

  using AnalyzerMsg::register_to_buffer;

  void register_to_buffer(const gps_time_t &the_time,
                          const msg_pos_llh_t &msg) noexcept override;

  void register_to_buffer(const gps_time_t &the_time,
                          const msg_pos_ecef_t &msg) noexcept override;

  const MetricInterface &flush_metric() noexcept override;

 private:
  // Stores input value (the "buffer")
  std::experimental::optional<msg_pos_llh_t> msg_pos_llh_;
  std::experimental::optional<msg_pos_ecef_t> msg_pos_ecef_;
  // Reference to the Metrics class used to call stateless computing functions
  MetricsErrors &msg_metric_;
  const std::string msg_name_;
};

struct AnalyzerIntegrityStatus : public AnalyzerMsg {
 public:
  explicit AnalyzerIntegrityStatus(const MetricsIntegrity &metrics_integrity)
      : AnalyzerMsg(ANALYZER_INTEGRITY_STATUS_NAME,
                    {ANALYZER_INTEGRITY_STATUS_NAME, {"", "", "", "", ""}}),
        msg_metric_(metrics_integrity) {}

  // TODO (guillaume) implement
  u16 build_message() noexcept override;

  const MetricInterface &flush_metric() noexcept override;

 private:
  const MetricsIntegrity &msg_metric_;
  const std::string msg_name_;
};

// MsG for velocity error in NED coordinate system
struct AnalyzerVelErrorNED : public AnalyzerMsg {
 public:
  explicit AnalyzerVelErrorNED(MetricsErrors &metrics_error)
      : AnalyzerMsg(ANALYZER_ERROR_VEL_NED_NAME,
                    {ANALYZER_ERROR_VEL_NED_NAME,
                     {"error_vel_n", "error_vel_e", "error_vel_d"}}),
        msg_metric_(metrics_error) {}

  u16 build_message() noexcept override;

  using AnalyzerMsg::register_to_buffer;

  void register_to_buffer(const gps_time_t &the_time,
                          const msg_vel_ned_t &msg) noexcept override;

  const MetricInterface &flush_metric() noexcept override;

 private:
  std::experimental::optional<msg_vel_ned_t> msg_vel_ned_;
  std::experimental::optional<Eigen::Vector3d> vel_error_ned_;

  MetricsErrors &msg_metric_;
};

// Build a MsG with a z-score estimate of vel_ned
struct AnalyzerZscoreVelNED : public AnalyzerMsg {
 public:
  explicit AnalyzerZscoreVelNED(MetricsErrors &metrics_error)
      : AnalyzerMsg(ANALYZER_ZSCORE_VEL_NED,
                    {ANALYZER_ZSCORE_VEL_NED,
                     {"zscore_vel_n", "zscore_vel_e", "zscore_vel_d"}}),
        msg_metric_(metrics_error) {}

  // Check if buffer is ready and if so, build the MsG metric
  u16 build_message() noexcept override;

  using AnalyzerMsg::register_to_buffer;

  void register_to_buffer(const gps_time_t &the_time,
                          const msg_vel_ned_t &msg) noexcept override;

  void register_to_buffer(const gps_time_t &the_time,
                          const msg_vel_ned_cov_t &msg) noexcept override;

  const MetricInterface &flush_metric() noexcept override;

 private:
  std::experimental::optional<msg_vel_ned_t> msg_vel_ned_;
  std::experimental::optional<msg_vel_ned_cov_t> msg_vel_ned_cov_;
  std::experimental::optional<Eigen::Vector3d> msg_value_;

  MetricsErrors &msg_metric_;
};

// Metric is an estimate of the ratio of pos error over estimated error
struct AnalyzerZscorePosECEF : public AnalyzerMsg {
 public:
  explicit AnalyzerZscorePosECEF(MetricsErrors &metrics_error)
      : AnalyzerMsg(ANALYZER_ERROR_POS_ECEF_COV_NAME,
                    {ANALYZER_ERROR_POS_ECEF_COV_NAME,
                     {"zscore_pos_x", "zscore_pos_y", "zscore_pos_z"}}),
        msg_metric_(metrics_error) {}

  // Check if buffer is ready and if so, build the MsG metric
  u16 build_message() noexcept override;

  using AnalyzerMsg::register_to_buffer;

  void register_to_buffer(const gps_time_t &the_time,
                          const msg_pos_ecef_t &msg) noexcept override;

  void register_to_buffer(const gps_time_t &the_time,
                          const msg_pos_ecef_cov_t &msg) noexcept override;

  const MetricInterface &flush_metric() noexcept override;

 private:
  std::experimental::optional<msg_pos_ecef_t> msg_pos_ecef_;
  std::experimental::optional<msg_pos_ecef_cov_t> msg_pos_ecef_cov_;
  std::experimental::optional<Eigen::Vector3d> msg_value_;

  MetricsErrors &msg_metric_;
};

struct AnalyzerErrorCogDeg : public AnalyzerMsg {
 public:
  explicit AnalyzerErrorCogDeg(MetricsErrors &metrics_error)
      : AnalyzerMsg(ANALYZER_ERROR_COG_DEG_NAME,
                    {ANALYZER_ERROR_COG_DEG_NAME, {"error_cog_deg"}}),
        msg_metric_(metrics_error) {}

  u16 build_message() noexcept override;

  using AnalyzerMsg::register_to_buffer;

  void register_to_buffer(const gps_time_t &the_time,
                          const msg_vel_ned_t &msg) noexcept override;

  const MetricInterface &flush_metric() noexcept override;

 private:
  std::experimental::optional<msg_vel_ned_t> msg_vel_ned_;
  MetricsErrors &msg_metric_;
};

struct AnalyzerErrorHdgDeg : public AnalyzerMsg {
 public:
  explicit AnalyzerErrorHdgDeg(MetricsErrors &metrics_error)
      : AnalyzerMsg(ANALYZER_ERROR_HDG_DEG_NAME,
                    {ANALYZER_ERROR_HDG_DEG_NAME, {"error_hdg_deg"}}),
        msg_metric_(metrics_error) {}

  u16 build_message() noexcept override;

  using AnalyzerMsg::register_to_buffer;

  void register_to_buffer(const gps_time_t &the_time,
                          const msg_orient_euler_t &msg) noexcept override;

  const MetricInterface &flush_metric() noexcept override;

 private:
  std::experimental::optional<msg_orient_euler_t> msg_orient_euler_;
  MetricsErrors &msg_metric_;
};

struct AnalyzerZscoreHdg : public AnalyzerMsg {
 public:
  explicit AnalyzerZscoreHdg(MetricsErrors &metrics_error)
      : AnalyzerMsg(ANALYZER_ZSCORE_HDG_NAME,
                    {ANALYZER_ZSCORE_HDG_NAME, {"zscore_hdg"}}),
        msg_metric_(metrics_error) {}

  u16 build_message() noexcept override;

  using AnalyzerMsg::register_to_buffer;

  void register_to_buffer(const gps_time_t &the_time,
                          const msg_orient_euler_t &msg) noexcept override;

  const MetricInterface &flush_metric() noexcept override;

 private:
  std::experimental::optional<msg_orient_euler_t> msg_orient_euler_;
  MetricsErrors &msg_metric_;
};

}  // namespace AnalyzerData
}  // namespace analyzer
}  // namespace starling

#endif  // STARLING_ANALYZER_DATA_TYPES_H
