#ifndef LIBPAL_CPP_SIZE_H_
#define LIBPAL_CPP_SIZE_H_

#include <cstddef>

namespace pal {

/**
 * C++14 implementation of std::size for container types
 *
 * @tparam T container type
 * @param t container instance
 * @return size of the container
 */
template <typename T>
constexpr auto size(const T &t) noexcept(noexcept(t.size()))
    -> decltype(t.size()) {
  return t.size();
}

/**
 * C++14 implementation of std::size for primitive arrays
 *
 * @tparam T array data type
 * @tparam N size of array
 * @param array primitive array instance
 * @return size of the array
 */
template <typename T, size_t N>
constexpr size_t size(const T (&array)[N]) noexcept {
  static_cast<void>(array);
  return N;
}

}  // namespace pal

#endif  // LIBPAL_CPP_SIZE_H_
