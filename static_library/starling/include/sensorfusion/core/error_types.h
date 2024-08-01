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

#ifndef SENSORFUSION_CORE_ERROR_TYPES_H_
#define SENSORFUSION_CORE_ERROR_TYPES_H_

//-----------------------------------------------------------------------------
// About:
//
// In order to make interfaces as clear as possible, a couple return types are
// provided for cases where functions can either:
//  1. succeed and generate a value
//  2. fail
// As a developer, you don't need to use these return types for *every single
// function*, but for high-level APIs they should be helpful for communicating
// the possible outcomes of a function/method call.
//
// In general it should be very clear what is going on.
//
//    MaybeError myfunction();
//
//    auto ret = myfunction();
//    if (ret.is_error()) {
//       cout << ret.error_message();
//    }
//
//    ValueOrError<int> myfunction();
//
//    auto ret = myfunction();
//    if (ret.is_ok()) {
//      cout << "number is" << *ret;
//    }
//
// When implementing functions, there are also convenient semantics:
//
//    MaybeError myfunction() {
//      if (bad_times)  return Error("Bad Times!!!");
//      if (good_times) return Ok();
//    }
//-----------------------------------------------------------------------------

#include <cassert>
#include <optional.hpp>
#include <type_traits>
#include <utility>

namespace sensorfusion {

class ErrorMessage {
 private:
  const char *const error_message_;

 public:
  explicit constexpr ErrorMessage(const char *error_message)
      : error_message_(error_message) {}

  constexpr const char *error_message() const { return error_message_; }
};

constexpr ErrorMessage Error() { return ErrorMessage("Unspecified"); }

constexpr ErrorMessage Error(const char *err_msg) {
  return ErrorMessage(err_msg);
}

//-----------------------------------------------------------------------------
class MaybeError {
 private:
  bool is_error_;
  const char *error_message_;

 public:
  constexpr MaybeError(bool is_error, const char *error_message)
      : is_error_{is_error}, error_message_{error_message} {}

  constexpr MaybeError(const ErrorMessage &err)
      : is_error_(true), error_message_(err.error_message()) {}

  constexpr bool is_error() const { return is_error_; }

  constexpr bool is_ok() const { return !is_error(); };

  constexpr const char *error_message() const {
    const char *msg = nullptr;
    if (is_error_) {
      msg = error_message_;
    }
    return msg;
  };
};

//-----------------------------------------------------------------------------
constexpr MaybeError Ok() { return MaybeError(false, nullptr); }

using in_place_t = std::experimental::in_place_t;

//-----------------------------------------------------------------------------
template <typename T>
class ValueOrError final : public MaybeError {
  static_assert(!std::is_convertible<T, MaybeError>::value,
                "NESTED_ERROR_TYPES_NOT_ALLOWED");
  static_assert(!std::is_reference<T>::value,
                "REFERENCE_ERROR_TYPES_NOT_ALLOWED"
                " (consider using std::reference_wrapper)");

 private:
  std::experimental::optional<T> data_;

 public:
  constexpr ValueOrError(const ErrorMessage &err)
      : MaybeError(true, err.error_message()), data_() {}

  template <typename... Args>
  explicit constexpr ValueOrError(in_place_t tag, Args &&... args)
      : MaybeError(false, nullptr),
        data_(in_place_t{}, std::forward<Args>(args)...) {}

  constexpr ValueOrError(const ValueOrError &other)
      : MaybeError(other), data_(other.data_) {}

  ValueOrError(T &&value)
      : MaybeError(false, nullptr),
        data_(in_place_t{}, std::forward<T>(value)) {}

  ~ValueOrError() = default;

  T &operator*() {
    assert(is_ok() && data_);
    return data_.value();
  }

  const T &operator*() const {
    assert(is_ok() && data_);
    return data_.value();
  }

  T *operator->() {
    assert(is_ok() && data_);
    return &data_.value();
  }

  const T *operator->() const {
    assert(is_ok() && data_);
    return &data_.value();
  }

  constexpr MaybeError to_maybe_error() const {
    return MaybeError(!is_ok(), error_message());
  }
};

//-----------------------------------------------------------------------------
template <typename T, typename... Args>
constexpr auto Ok(Args &&... args) {
  return ValueOrError<T>(in_place_t{}, std::forward<Args>(args)...);
}

}  // namespace sensorfusion

#endif
