/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_PVT_ENGINE_LABELED_ESTIMATE_DIAGONAL_H
#define LIBSWIFTNAV_PVT_ENGINE_LABELED_ESTIMATE_DIAGONAL_H

#include <swiftnav/signal.h>

namespace pvt_engine {

namespace ambiguities {

struct LabeledEstimateDiagonal {
  gnss_signal_t sid;
  double ambiguity_mean;
  double ambiguity_variance;

  LabeledEstimateDiagonal(gnss_signal_t label, double amb, double var)
      : sid(label), ambiguity_mean(amb), ambiguity_variance(var){};
};

}  // namespace ambiguities

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_LABELED_ESTIMATE_DIAGONAL_H
