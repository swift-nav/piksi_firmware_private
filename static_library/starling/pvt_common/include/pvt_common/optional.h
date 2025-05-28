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

#ifndef PVT_COMMON_OPTIONAL_H
#define PVT_COMMON_OPTIONAL_H

#include <optional.hpp>

// To improve readability, we alias `std::experimental::optional` to `optional`.
template <typename T>
using optional = std::experimental::optional<T>;
constexpr std::experimental::in_place_t in_place;

template <typename T>
optional<typename std::remove_cv<T>::type> ref_to_value(optional<T &> opt) {
  if (!opt.has_value()) {
    return {};
  }

  return *opt;
}

#endif  // PVT_COMMON_OPTIONAL_H
