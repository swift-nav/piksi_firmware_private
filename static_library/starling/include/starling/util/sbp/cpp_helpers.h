/**
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_CPP_HELPERS_H
#define STARLING_CPP_HELPERS_H

#include <cassert>
#include <cstdint>

namespace starling {
namespace util {
namespace sbp {

namespace details {
template <typename ClassT, typename ArgT>
using MemFn = void (ClassT::*)(uint16_t, const ArgT &);
}

// A helper function that casts an SBP msg buffer into a SBP type and calls a
// specified member function This is useful for calling a member function from
// an SBP callback The context should be a pointer to the class instance to
// callq
template <typename ArgT, typename ClassT, details::MemFn<ClassT, ArgT> func>
void sbp_cb_passthrough(uint16_t sender_id, uint8_t len, uint8_t msg[],
                        void *context) {
  assert(nullptr != context);
  auto instance = static_cast<ClassT *>(context);

  assert(len == sizeof(ArgT));
  auto val = reinterpret_cast<ArgT *>(msg);  // NOLINT

  ((*instance).*(func))(sender_id, *val);
}

}  // namespace sbp
}  // namespace util
}  // namespace starling

#endif  // STARLING_CPP_HELPERS_H
