///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2019 Swift Navigation Inc.
// Contact: Swift Navigation <dev@swiftnav.com>
//
// This source is subject to the license found in the file 'LICENSE' which must
// be distributed together with this source. All other rights reserved.
//
// THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
// EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
///////////////////////////////////////////////////////////////////////////////

#ifndef SENSORFUSION_CORE_ORDERED_MULTIBUFFER_INTERNAL_H_
#define SENSORFUSION_CORE_ORDERED_MULTIBUFFER_INTERNAL_H_

#include <assert.h>

#include <array>
#include <functional>
#include <optional.hpp>
#include <tuple>
#include <utility>

#include "pvt_common/containers/circular_buffer.h"
#include "pvt_common/containers/static_vector.h"
#include "sensorfusion/core/error_types.h"
#include "sensorfusion/core/traits.h"

namespace sensorfusion {
namespace omb_internal {

// A handle is bound to a particular set of types.
template <class... Streams>
class ReaderHandle {
 public:
  ReaderHandle(size_t id) : id_{id} {}
  size_t id() const { return id_; }

 private:
  size_t id_;
};

template <class Key, class... Streams>
struct OrderedMultiBufferBase {
  static_assert(
      traits::has_relational_operators_v<Key>,
      "cannot define OrderedMultiBuffer with non-comparable Key type");
  static_assert(
      traits::are_types_unique_v<Streams...>,
      "cannot create OrderedMultiBuffer with non-unique stream types");
};

}  // namespace omb_internal
}  // namespace sensorfusion

#endif
