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

#include "starling/util/mutex.h"

#include <swiftnav/logging.h>

namespace starling {
namespace util {

pal_mutex_t Mutex::mutexes[kMutexCount];

void Mutex::InitAll() noexcept {
  for (unsigned int i = 0; i < kMutexCount; ++i) {
    if (pal_mutex_alloc(&mutexes[i]) != PAL_SUCCESS) {
      log_error("STARLING: unable to initialize mutex: %u.", i);
    }
  }
}

void Mutex::Lock(ID mutex_id) noexcept { pal_mutex_lock(mutexes[mutex_id]); }

void Mutex::Unlock(ID mutex_id) noexcept {
  pal_mutex_unlock(mutexes[mutex_id]);
}

pal_mutex_t Mutex::GetMutex(ID mutex_id) noexcept { return mutexes[mutex_id]; }

LockGuard::LockGuard(Mutex::ID mutex_id) noexcept : mutex_id_(mutex_id) {
  Mutex::Lock(mutex_id);
}

LockGuard::~LockGuard() noexcept { Mutex::Unlock(mutex_id_); }

}  // namespace util
}  // namespace starling
