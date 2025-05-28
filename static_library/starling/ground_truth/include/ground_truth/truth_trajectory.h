/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef GROUND_TRUTH_TRAJECTORY_H
#define GROUND_TRUTH_TRAJECTORY_H

#include <algorithm>
#include <vector>

#include <swiftnav/constants.h>
#include <swiftnav/coord_system.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/logging.h>

#include <pvt_common/eigen_custom.h>
#include <pvt_common/optional.h>

#include "ground_truth/truth_epoch.h"
#include "ground_truth/truth_file_and_format.h"

namespace ground_truth {

constexpr double
    TRUTH_TRAJECTORY_DEFAULT_MAX_ACCEPTED_HORIZONTAL_POSITION_ERROR = 0.1;

struct ZuptConfiguration {
  enum ZuptSource { TRUTH, SPEED, NONE };
  ZuptSource zupt_source;
  double zupt_disable_threshold;
  double zupt_enable_threshold;
  double zupt_time_window;

  /**
   * Empty Constructor, defaults to no ZUPT source
   */
  ZuptConfiguration()
      : zupt_source(ZuptSource::NONE),
        zupt_disable_threshold(0),
        zupt_enable_threshold(0),
        zupt_time_window(0) {}

  /**
   * Constructor
   *
   * @param zupt_source source for ZUPT reference. 'truth' to use iFlag
   * directly. 'speed' to use  speed to determine ZUPT, otherwise don't populate
   * ZUPT.
   * @param zupt_enable_threshold velocity >= this are considered for ZUPT
   * missed detection. Only used when zupt_source is 'speed'.
   * @param zupt_disable_threshold velocity >= this are considered for ZUPT
   * false positive. Only used when zupt_source is 'speed'.
   * @param zupt_time_window window of to check for criteria before marking ZUPT
   * error. If results in window are ambiguous do not mark false
   * positive/negatives
   */
  ZuptConfiguration(const std::string &source, double disable_threshold,
                    double enable_threshold, double time_window)
      : zupt_source(ZuptSource::NONE),
        zupt_disable_threshold(disable_threshold),
        zupt_enable_threshold(enable_threshold),
        zupt_time_window(time_window) {
    if (source == "truth") {
      zupt_source = ZuptSource::TRUTH;
    } else if (source == "speed") {
      zupt_source = ZuptSource::SPEED;
    }
  }
};

class TruthTrajectory : public Truth {
 public:
  explicit TruthTrajectory(const TruthFileAndFormat &truth_log,
                           bool is_truth_static = false);

  void parse_and_filter_est_hoz_error(
      const double &max_accepted_horizontal_position_accuracy,
      const ZuptConfiguration &zupt_config = {});

  optional<TruthEpoch> get_truth_at(const gps_time_t &time) override;
  optional<TruthEpoch> get_truth_at(const gps_time_t &time_now,
                                    const gps_time_t &time_prev) override;

 private:
  TruthFileAndFormat truth_log_;
  std::vector<TruthEpoch> truth_trajectory_;
  bool is_truth_static_;
};

void filter_estimated_hoz_error(
    std::vector<TruthEpoch> *truth,
    const double &max_accepted_hozizontal_position_accuracy);

std::vector<TruthEpoch> parse_truth_log(const TruthFileAndFormat &truth_log,
                                        const ZuptConfiguration &zupt_config);

std::vector<TruthEpoch> parse_standard_format_truth_log_line(
    const std::string &file_path, const ZuptConfiguration &zupt_config);

std::vector<TruthEpoch> parse_alternate_format_truth_log_line(
    const std::string &file_path, const ZuptConfiguration &zupt_config);

std::vector<TruthEpoch> parse_unknown_format_truth_log_line(
    const std::string &file_path, const ZuptConfiguration &zupt_config);

optional<TruthEpoch> find_truth_epoch(
    const std::vector<TruthEpoch> &truth_trajectory, const gps_time_t &time);

}  // namespace ground_truth

#endif  // GROUND_TRUTH_TRAJECTORY_H
