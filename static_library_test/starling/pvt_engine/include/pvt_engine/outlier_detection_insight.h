/*
 * Copyright (C) 2021 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_OUTLIER_DETECTION_INSIGHT_H
#define STARLING_OUTLIER_DETECTION_INSIGHT_H

namespace pvt_engine {

enum class OutlierDetectionResult {
  kOutlierDetectionNotDone,
  kNoOutlierFound,
  kPosMdeTooHigh,
  kVelMdeTooHigh,
  kObservabilityTooPoor,
  kTooManyOutliers,
};

class OutlierDetectionInsight {
 public:
  OutlierDetectionInsight()
      : outlier_detection_return_type_(
            OutlierDetectionResult::kOutlierDetectionNotDone) {}

  void set_outlier_detection_return(OutlierDetectionResult return_type) {
    outlier_detection_return_type_ = return_type;
  }

  OutlierDetectionResult get_outlier_detection_return_type() const {
    return outlier_detection_return_type_;
  }

 private:
  OutlierDetectionResult outlier_detection_return_type_;
};

}  // namespace pvt_engine

#endif  // STARLING_OUTLIER_DETECTION_INSIGHT_H
