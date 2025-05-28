#ifndef LIBPAL_CPP_MONOTONIC_CLOCK_H
#define LIBPAL_CPP_MONOTONIC_CLOCK_H

#include <libpal/chrono/monotonic_clock.h>

#include <chrono>
#include <type_traits>

namespace pal {
namespace chrono {
/**
 * A std::chrono clock whose epoch starts when the machine
 * starts up and steadily increments its timer monotonically.
 *
 * This class works identical to std::chrono::steady_clock.
 */
class MonotonicClock final {
 public:
  using rep = int64_t;
  using period = std::nano;
  using duration = std::chrono::duration<rep, period>;
  using time_point = std::chrono::time_point<MonotonicClock, duration>;

 public:
  static constexpr bool is_steady = true;
  static time_point now() noexcept;
};
}  // namespace chrono
}  // namespace pal

#endif  // LIBPAL_CPP_MONOTONIC_CLOCK_H