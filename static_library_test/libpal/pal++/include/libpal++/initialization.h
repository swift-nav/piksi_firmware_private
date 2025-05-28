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

#ifndef LIBPAL_CPP_INITIALIZATION_H
#define LIBPAL_CPP_INITIALIZATION_H

#include <libpal/error.h>
#include <utility>

namespace pal {

/**
 * Helper class to simultaneously initialize multiple libpal objects.
 *
 * You probably want to use `pal::init_all` instead of using this directly.
 */
template <typename FirstArgT, typename... ArgsT>
struct Initializer {
  static pal_error init_all(FirstArgT &&first_arg, ArgsT &&... args) noexcept {
    pal_error init_result = first_arg.init();
    if (init_result != PAL_SUCCESS) {
      return init_result;
    }

    return Initializer<ArgsT...>::init_all(std::forward<ArgsT>(args)...);
  }
};

template <typename ArgT>
struct Initializer<ArgT> {
  static pal_error init_all(ArgT &&arg) noexcept {
    pal_error init_result = arg.init();
    if (init_result != PAL_SUCCESS) {
      return init_result;
    }

    return PAL_SUCCESS;
  }
};

/**
 * @brief Helper function to initialize multiple libpal objects in one pass.
 *
 * This is usually helpful when you are initializing multiple libpal objects
 * at once at startup and want to detect any initialization errors.
 *
 * This function will call `.init()` on all of the objects you pass
 * into it in sequence. It will return the first non-success error code it
 * encounters, skipping initializing any remaining objects.
 */
template <typename... Ts>
pal_error init_all(Ts &&... args) noexcept {
  return Initializer<Ts...>::init_all(std::forward<Ts>(args)...);
}

/**
 * Helper class to simultaneously deinitialize multiple libpal objects.
 *
 * You probably want to use `pal::deinit_all` instead of using this directly.
 */
template <typename FirstArgT, typename... ArgsT>
struct Deinitializer {
  static pal_error deinit_all(FirstArgT &&first_arg,
                              ArgsT &&... args) noexcept {
    pal_error deinit_result = first_arg.deinit();
    pal_error other_deinit_result =
        Deinitializer<ArgsT...>::deinit_all(std::forward<ArgsT>(args)...);

    if (deinit_result != PAL_SUCCESS) {
      return deinit_result;
    }

    return other_deinit_result;
  }
};

template <typename ArgT>
struct Deinitializer<ArgT> {
  static pal_error deinit_all(ArgT &&arg) noexcept { return arg.deinit(); }
};

/**
 * @brief Helper function to deinitialize multiple libpal objects in one pass.
 *
 * This is usually helpful when you are deinitializing multiple libpal objects
 * at once at shut down and want to detect any errors.
 *
 * This function will call `.deinit()` on all of the objects you pass
 * into it in sequence. It will return the first non-success error code it
 * encounters, but it will continue to deinitialize all remaining objects
 * ignoring any subsequent errors.
 */
template <typename... Ts>
pal_error deinit_all(Ts &&... args) noexcept {
  return Deinitializer<Ts...>::deinit_all(std::forward<Ts>(args)...);
}

}  // namespace pal

#endif  // LIBPAL_CPP_INITIALIZATION_H
