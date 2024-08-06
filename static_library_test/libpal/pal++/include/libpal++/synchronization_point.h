/**
 * Copyright (C) 2021 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef PAL_CPP_SYNCHRONIZATION_POINT_H
#define PAL_CPP_SYNCHRONIZATION_POINT_H

#include <libpal++/condition_variable.h>
#include <libpal++/mutex.h>
#include <libpal/error.h>

namespace pal {

class SynchronizationPoint {
  pal::Mutex mutex_;
  pal::ConditionVariable cond_;
  bool triggered_;

 public:
  SynchronizationPoint();
  SynchronizationPoint(const SynchronizationPoint &) = delete;
  SynchronizationPoint(SynchronizationPoint &&) = delete;

  ~SynchronizationPoint() { deinit(); }

  SynchronizationPoint &operator=(const SynchronizationPoint &) = delete;
  SynchronizationPoint &operator=(SynchronizationPoint &&) = delete;

  pal_error init() noexcept;
  pal_error deinit() noexcept;

  template <typename Rep, typename Period>
  pal_error wait_for(
      const std::chrono::duration<Rep, Period> &duration) noexcept {
    if (!mutex_.is_valid() || !cond_.is_valid()) {
      return PAL_INVALID;
    }

    pal::ScopedLock<pal::Mutex> lock(mutex_);
    if (triggered_) {
      triggered_ = false;
      return PAL_SUCCESS;
    }

    pal_error result = cond_.wait_for(lock, duration);
    if (result == PAL_SUCCESS && triggered_) {
      triggered_ = false;
    }
    return result;
  }

  pal_error wait() noexcept;

  pal_error signal() noexcept;

  bool is_valid() const noexcept {
    return mutex_.is_valid() && cond_.is_valid();
  }
};

}  // namespace pal

#endif  // PAL_CPP_SYNCHRONIZATION_POINT_H
