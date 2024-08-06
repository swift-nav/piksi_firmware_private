#ifndef LIBPAL_CPP_TUPLE_H
#define LIBPAL_CPP_TUPLE_H

/**
 * Header offers libpal++ developers the ability to use various
 * template functions available in "#include <tuple>" which aren't
 * available in C++14 standard. When we eventually move over to higher
 * versions which offers the same functionality we can start refactoring
 * the code to use them over this.
 */

#include <tuple>
#include <type_traits>
#include <utility>

#include <libpal++/functional.h>

namespace pal {

/**
 * Implementation for std::apply, code used below is a modified version of what
 * is presented in cppreference.com.
 *
 * https://en.cppreference.com/w/cpp/utility/apply#Possible_implementation
 */

namespace detail {
template <class Callable, class Tuple, std::size_t... I>
constexpr decltype(auto) apply(Callable &&callable, Tuple &&tuple,
                               std::index_sequence<I...> /*unused*/) {
  return pal::invoke(std::forward<Callable>(callable),
                     std::get<I>(std::forward<Tuple>(tuple))...);
}
}  // namespace detail

template <class Callable, class Tuple>
constexpr decltype(auto) apply(Callable &&callable, Tuple &&tuple) {
  return detail::apply(
      std::forward<Callable>(callable), std::forward<Tuple>(tuple),
      std::make_index_sequence<
          std::tuple_size<std::remove_reference_t<Tuple>>::value>{});
}
}  // namespace pal

#endif  // LIBPAL_CPP_TUPLE_H
