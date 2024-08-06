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

#ifndef PVT_COMMON_FUNCTIONAL_FOLD_H
#define PVT_COMMON_FUNCTIONAL_FOLD_H

#include <utility>

namespace pvt_common {
namespace functional {

/*
 * These functions attempt to implement functionality similar to
 * fold expressions in C++17, but with C++14 compliant code. These
 * fold functions apply a given function to the argument lists and
 * return the value from the final invocation of the given function.
 * The function must accept two arguments and return a value. The
 * function is first invoked for a pair of given values as the
 * arguments, then it is invoked again with the return value and the
 * next given value. This continues until all of the given values have
 * been used. The order that the arguments are used is based on the
 * fold direction, either left to right or right to left.
 *
 * Folding from left to right is equivalent to
 *   func(func(...func(arg0, arg1), arg2), .... argN);
 * If the given function is std::plus it is equivalent to
 *   (((arg0 + arg1) + arg2) + ...) + argN)
 * Fold from left to right means the function's return type must be
 * implicitly convertible to the first argument's type.
 *
 * Folding from right to left is equivalent to
 *   func(arg0, func(arg1, func(..., func(argN-1, argN)))
 * If the given function is std::plus it is equivalent to
 *   (arg0 + arg1 + (... + (argN-1 + argN)))
 */

template <typename F, typename TLeft, typename TRight>
auto fold_right_to_left(F &&func, TLeft &&lhs, TRight &&rhs) {
  return func(std::forward<TLeft>(lhs), std::forward<TRight>(rhs));
}

template <typename F, typename T, typename... Ts>
auto fold_right_to_left(F &&func, T &&arg, Ts &&... others) {
  return func(
      std::forward<T>(arg),
      fold_right_to_left(std::forward<F>(func), std::forward<Ts>(others)...));
}

template <typename F, typename TLeft, typename TRight>
auto fold_left_to_right(F &&func, TLeft &&lhs, TRight &&rhs) {
  return func(std::forward<TLeft>(lhs), std::forward<TRight>(rhs));
}

template <typename F, typename TLeft, typename TRight, typename... Ts>
auto fold_left_to_right(F &&func, TLeft &&lhs, TRight &&rhs, Ts &&... others) {
  return fold_left_to_right(
      std::forward<F>(func),
      func(std::forward<TLeft>(lhs), std::forward<TRight>(rhs)),
      std::forward<Ts>(others)...);
}

/*
 * Helper tag types and wrapper functions
 *
 * `fold(LeftToRight, ...)` will call `fold_left_to_right(...)`
 * Similar for `RightToLeft`
 */
struct RightToLeftTag {};
static constexpr RightToLeftTag RightToLeft;
struct LeftToRightTag {};
static constexpr LeftToRightTag LeftToRight;

template <typename F, typename... Ts>
auto fold(RightToLeftTag, F &&func, Ts &&... args) {
  return fold_right_to_left(std::forward<F>(func), std::forward<Ts>(args)...);
}

template <typename F, typename... Ts>
auto fold(LeftToRightTag, F &&func, Ts &&... args) {
  return fold_left_to_right(std::forward<F>(func), std::forward<Ts>(args)...);
}

};  // namespace functional
}  // namespace pvt_common

#endif  // PVT_COMMON_FUNCTIONAL_FOLD_H
