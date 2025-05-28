/**
 * Copyright (C) 2020 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_PVT_ENGINE_RANGES_REVERSE_H
#define LIBSWIFTNAV_PVT_ENGINE_RANGES_REVERSE_H

#include <iterator>

namespace pvt_engine {
namespace ranges {

template <typename Iter>
class ReverseView {
 public:
  using iterator = std::reverse_iterator<Iter>;

  ReverseView(Iter begin, Iter end) : begin_(begin), end_(end) {}

  iterator begin() { return std::make_reverse_iterator(end_); }

  iterator end() { return std::make_reverse_iterator(begin_); }

 private:
  Iter begin_;
  Iter end_;
};

template <typename Range>
auto reverse(Range &range) {
  using Iter = decltype(std::declval<Range>().begin());
  return ReverseView<Iter>(range.begin(), range.end());
}

}  // namespace ranges
}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_RANGES_REVERSE_H
