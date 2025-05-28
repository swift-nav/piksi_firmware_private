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

#ifndef LIBSWIFTNAV_PVT_ENGINE_FINITY_H
#define LIBSWIFTNAV_PVT_ENGINE_FINITY_H

#include <pvt_engine/eigen_types.h>
#include <pvt_engine/obss.h>

namespace pvt_engine {

bool validate_double(double in);

bool validate_3_vector(const double in[3], const char *prefix);

bool validate_3_vector(const Eigen::Vector3d &in, const char *prefix);

template <typename matrix_base_type>
bool validate_double_matrix_base(const matrix_base_type &matrix_base) {
  return Eigen::isfinite(matrix_base.array()).all();
}

template <typename T>
bool validate_in_range(const T &value, const T &lower_bound,
                       const T &upper_bound) {
  return (lower_bound <= value) && (value <= upper_bound);
}

template <typename T>
bool validate_greater_or_equal(const T &value, const T &lower_bound) {
  return (lower_bound <= value);
}

template <typename T>
bool validate_lower_or_equal(const T &value, const T &upper_bound) {
  return (value <= upper_bound);
}

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_FINITY_H
