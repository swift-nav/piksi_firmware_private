#ifndef LIBPAL_CPP_CHRONO_H_
#define LIBPAL_CPP_CHRONO_H_

#include <type_traits>

#include <libpal++/monotonic_clock.h>

/**
 * Header offers an easy way for developers to include all the chrono related
 * classes offered within libpal++.
 *
 * It also offers libpal++ developers the ability to use various template
 * functions available in "#include <chrono>" which aren't available in C++14
 * standard. When we eventually move over to higher versions which offers the
 * same functionality we can start refactoring the code to use them over this.
 */

namespace pal {
namespace chrono {
/**
 * Implementation for std::chrono::ceil, code used below is a modified version
 * of what is presented in cppreference.com.
 *
 * https://en.cppreference.com/w/cpp/chrono/duration/ceil#Possible_implementation
 */

template <class T>
struct is_duration : std::false_type {};
template <class Rep, class Period>
struct is_duration<std::chrono::duration<Rep, Period>> : std::true_type {};

template <class To, class Rep, class Period,
          class = std::enable_if_t<is_duration<To>{}>>
constexpr To ceil(const std::chrono::duration<Rep, Period> &d) {
  To t = std::chrono::duration_cast<To>(d);

  if (t < d) {
    return t + To{1};
  }

  return t;
}
}  // namespace chrono
}  // namespace pal

#endif  // LIBPAL_CPP_CHRONO_H_
