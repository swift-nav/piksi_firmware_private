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

#ifndef LIBSWIFTNAV_PVT_ENGINE_NEWLIB_WORKAROUNDS_H
#define LIBSWIFTNAV_PVT_ENGINE_NEWLIB_WORKAROUNDS_H

#include <string>

#if defined(__ARM_EABI__)
#include <sstream>
#endif  // defined(__ARM_EABI__)

#if defined(__ARM_EABI__)
// This is awful!
#define int32_t int
#endif  // defined(__ARM_EABI__)

namespace std_support {

template <typename T>
std::string to_string(T value) {
#if defined(__ARM_EABI__)
  std::ostringstream TempStream;
  TempStream << value;
  return TempStream.str();
#else   // defined(__ARM_EABI__)
  return std::to_string(value);
#endif  // defined(__ARM_EABI__)
}

}  // namespace std_support

#endif  // LIBSWIFTNAV_PVT_ENGINE_NEWLIB_WORKAROUNDS_H
