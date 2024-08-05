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

#ifndef LIBSWIFTNAV_PVT_ENGINE_GAUSSIAN_H
#define LIBSWIFTNAV_PVT_ENGINE_GAUSSIAN_H

#include <optional.hpp>

namespace pvt_engine {

struct Gaussian {
  double mean;
  double variance;

  Gaussian(double mu, double sigmasq) : mean(mu), variance(sigmasq) {}

  Gaussian() : mean(1e222), variance(-1e222) {}
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_GAUSSIAN_H
