#ifndef LIBPAL_CPP_TYPE_TRAITS_H
#define LIBPAL_CPP_TYPE_TRAITS_H

/**
 * This header file offers standard library type trait metafunctions which were
 * introduced in C++17 and up.
 */

#include <functional>
#include <type_traits>

namespace pal {
// obtained from
// https://www.boost.org/doc/libs/1_65_1/boost/poly_collection/detail/is_invocable.hpp
template <typename F, typename... Args>
struct is_invocable
    : std::is_constructible<
          std::function<void(Args...)>,
          std::reference_wrapper<typename std::remove_reference<F>::type>> {};

// obtained from
// https://www.boost.org/doc/libs/1_65_1/boost/poly_collection/detail/is_invocable.hpp
template <typename R, typename F, typename... Args>
struct is_invocable_r
    : std::is_constructible<
          std::function<R(Args...)>,
          std::reference_wrapper<typename std::remove_reference<F>::type>> {};
}  // namespace pal

#endif  // LIBPAL_CPP_TYPE_TRAITS_H