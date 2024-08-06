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

#ifndef LIBPAL_CPP_LOCKED_RESOURCE_H
#define LIBPAL_CPP_LOCKED_RESOURCE_H

#include <libpal++/mutex.h>
#include <libpal++/scoped_lock.h>
#include <cassert>
#include <utility>

namespace pal {

template <typename T>
class LockedResource {
 private:
  mutable Mutex mutex_;
  T instance_;
  bool inited_;

  template <typename U, typename = void>
  class GenericLockGuard {
    ScopedLock<Mutex> lock_;
    U &reference_;

   public:
    GenericLockGuard(Mutex &mutex, U &reference)
        : lock_(mutex), reference_(reference) {}
    GenericLockGuard(const GenericLockGuard &) = delete;
    GenericLockGuard(GenericLockGuard &&) noexcept = default;

    ~GenericLockGuard() = default;

    GenericLockGuard &operator=(const GenericLockGuard &) = delete;
    GenericLockGuard &operator=(GenericLockGuard &&) noexcept = default;

    U &operator*() { return reference_; }

    U *operator->() { return &reference_; }
  };

 public:
  using WrappedType = T;

  using LockGuard = GenericLockGuard<T>;
  using ConstLockGuard = GenericLockGuard<const T>;

  template <typename... Args>
  explicit LockedResource(Args &&... args)
      : mutex_(), instance_(std::forward<Args>(args)...), inited_(false) {}
  LockedResource(const LockedResource &) = delete;
  LockedResource(LockedResource &&) noexcept = delete;

  ~LockedResource() { mutex_.deinit(); }

  LockedResource &operator=(const LockedResource &) = delete;
  LockedResource &operator=(LockedResource &&) noexcept = delete;

  pal_error init() {
    inited_ = true;
    return mutex_.init();
  }

  pal_error deinit() {
    inited_ = false;
    return mutex_.deinit();
  }

  LockGuard get_lock() {
    assert(inited_);
    return LockGuard(mutex_, instance_);
  }

  ConstLockGuard get_lock() const {
    assert(inited_);
    return ConstLockGuard(mutex_, instance_);
  }

  T *unsafe_get() { return &instance_; }
};

}  // namespace pal

#endif  // LIBPAL_CPP_LOCKED_RESOURCE_H
