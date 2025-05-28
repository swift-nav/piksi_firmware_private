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

#ifndef STARLING_PVT_ENGINE_MEAN_AND_VARIANCE_H
#define STARLING_PVT_ENGINE_MEAN_AND_VARIANCE_H

namespace pvt_engine {

class MeanAndVariance {
 public:
  MeanAndVariance();
  MeanAndVariance(const double mean, const double variance);

  bool operator==(const MeanAndVariance &rhs) const;

  void set_mean(const double mean);
  void set_variance(const double variance);
  void set_standard_deviation(const double standard_deviation);

  double get_mean() const;
  double get_variance() const;
  double get_standard_deviation() const;

 private:
  double mean_;
  double variance_;
};

}  // namespace pvt_engine

#endif  // STARLING_PVT_ENGINE_MEAN_AND_VARIANCE_H
