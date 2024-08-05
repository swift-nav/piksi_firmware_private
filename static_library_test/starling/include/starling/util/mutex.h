/**
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_UTIL_MUTEX_H
#define STARLING_UTIL_MUTEX_H

#include <libpal/synch/mutex.h>
#include <swiftnav/logging.h>

namespace starling {
namespace util {

/**
 * A simple wrapper around the platform_mutex functions
 */
class Mutex {
 public:
  /* Mutex IDs used in this implementation. */
  enum ID {
    GLOBAL_SETTINGS = 0,
    GLONASS_BIASES,
    REFERENCE_POSITION,
    TM_FILTER,
    LL_FILTER,
    SPP_FILTER,
    IONO_PARAMS,
    PRIMARY_QUEUE,
    PAIRED_OBS_MEMPOOL,
    DATA_PROCESSED,
    TIME_MATCHED_DONE,
    kMutexCount,
  };
  static size_t mutex_count() { return kMutexCount; };

  static void InitAll() noexcept;
  static void Lock(ID mutex_id) noexcept;
  static void Unlock(ID mutex_id) noexcept;
  static pal_mutex_t GetMutex(ID mutex_id) noexcept;

  Mutex() = delete;
  ~Mutex() = delete;

 private:
  static pal_mutex_t mutexes[kMutexCount];
};

/**
 * A similar class to `std::lock_guard`, it locks the specified Mutex upon
 * construction and unlocks the same mutex upon destruction.
 */
class LockGuard {
 public:
  explicit LockGuard(Mutex::ID mutex_id) noexcept;

  ~LockGuard() noexcept;

  LockGuard() = delete;
  LockGuard(const LockGuard &) = delete;
  LockGuard(LockGuard &&) = delete;

  LockGuard &operator=(const LockGuard &) = delete;
  LockGuard &operator=(LockGuard &&) = delete;

 private:
  Mutex::ID mutex_id_;
};

}  // namespace util
}  // namespace starling

#endif /* STARLING_UTIL_MUTEX_H */
