/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_PVT_ENGINE_IONO_UTILS_H
#define LIBSWIFTNAV_PVT_ENGINE_IONO_UTILS_H

#include <swiftnav/signal.h>

namespace pvt_engine {

enum class Units : bool { Cycles, Meters };

double get_iono_scale_factor(const gnss_signal_t &sid,
                             const Units &output_units);

double get_mapping_coefficient(const double &elev_rad);

double get_decorrelation_factor(const double &baseline_length_km,
                                const double &decorrelation_distance);

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_IONO_UTILS_H
