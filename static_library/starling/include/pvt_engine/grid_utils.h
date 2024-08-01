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

#ifndef STARLING_GRID_UTILS_H
#define STARLING_GRID_UTILS_H

#include <pvt_common/eigen_custom.h>
#include <pvt_engine/optional.h>
#include <pvt_engine/ssr_corrections.h>
#include <swiftnav/coord_system.h>

namespace pvt_engine {
namespace grid {

MeanAndVariance interpolate_grid_values(
    const pvt_engine::SsrGrid &grid, const pvt_engine::LatLonDeg &point,
    const pvt_engine::GridValues &grid_values);

}  // namespace grid
}  // namespace pvt_engine

#endif  // STARLING_GRID_UTILS_H
