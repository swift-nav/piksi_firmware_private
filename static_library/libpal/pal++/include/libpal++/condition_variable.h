#ifndef LIBPAL_CPP_CONDITION_VARIABLE_H
#define LIBPAL_CPP_CONDITION_VARIABLE_H

#include <libpal++/chrono.h>
#include <libpal++/mutex.h>
#include <libpal++/scoped_lock.h>
#include <libpal++/validation.h>
#include <libpal/error.h>
#include <libpal/require.h>
#include <libpal/synch/condition_var.h>

#include <chrono>
#include <utility>

namespace pal {
/**
 * Similar class to std::condition_variable, except that it uses ScopedLock
 * over std::unique_lock. It attempts to mirror the methods of the C++ standard
 * library, the only difference is that all methods will return a platform error
 * code, and if must be explicitly initialized before use (#init). Other
 * differences will be documented.
 */
class ConditionVariable final {
 public:
  ConditionVariable() noexcept : pal_handle_() {}

  ConditionVariable(const ConditionVariable &other) = delete;
  ConditionVariable(ConditionVariable &&other) noexcept : pal_handle_() {
    std::swap(pal_handle_, other.pal_handle_);
  }

  ConditionVariable &operator=(const ConditionVariable &other) = delete;
  ConditionVariable &operator=(ConditionVariable &&other) noexcept {
    std::swap(pal_handle_, other.pal_handle_);
    return *this;
  }

  ~ConditionVariable() { deinit(); }

  /**
   * Initialize condition variable ready for use
   *
   * Allocate resources from libpal. Once this class has been instantiated it
   * must be initialized by calling this function. Only after this function
   * returns PAL_SUCCESS will the condition variable be available to use by
   * calling other member functions.
   *
   * @return PAL error code
   */
  pal_error init() noexcept;

  /**
   * Deinitialize condition variable
   *
   * Release all PAL resources used by this instance. After this function
   * returns PAL_SUCCESS the condition variable must not be used unless it is
   * reinitialized by a call to init().
   *
   * This function will be called automatically when the instance of
   * ConditionVariable is destroyed.
   *
   * @return PAL error code
   */
  pal_error deinit() noexcept;

  /**
   * Test whether this instance is valid, whether it is available for use
   *
   * A condition variable is valid (ie, ready to be used) once it has been
   * successfully initialized by a call to init(). When this function returns
   * true it is safe to use other functions in the class. If this function
   * returns false this instance must not be used.
   *
   * @return true if the instance is valid (ready to use), false otherwise
   */
  bool is_valid() const noexcept { return pal_handle_ != nullptr; }

  pal_error notify_one() noexcept;
  pal_error notify_all() noexcept;

  pal_error wait(ScopedLock<Mutex> &scoped_lock) noexcept;

  template <typename Predicate>
  pal_error wait(ScopedLock<Mutex> &scoped_lock,
                 Predicate &&predicate) noexcept(noexcept(predicate())) {
    static_assert(
        std::is_same<typename std::result_of<Predicate()>::type, bool>::value,
        "Predicate callable must return a boolean value");

    pal_error error = pal_require(is_valid() && pal_has_impl_cv());

    while (error == PAL_SUCCESS && !predicate()) {
      error = wait(scoped_lock);
    }

    return error;
  }

  template <typename Rep, typename Period>
  pal_error wait_for(
      ScopedLock<Mutex> &scoped_lock,
      const std::chrono::duration<Rep, Period> &duration) noexcept {
    if (duration <= std::chrono::microseconds::zero()) {
      pal_error err = pal_require(is_valid() && pal_has_impl_cv());
      if (err == PAL_SUCCESS) {
        err = PAL_TIMEOUT;
      }
      return err;
    }

    auto microseconds = pal::chrono::ceil<std::chrono::microseconds>(duration);

    return ::pal_cv_wait_for(pal_handle_, scoped_lock.mutex().pal_handle(),
                             static_cast<uint64_t>(microseconds.count()));
  }

  template <typename Rep, typename Period, typename Predicate>
  pal_error wait_for(ScopedLock<Mutex> &scoped_lock,
                     const std::chrono::duration<Rep, Period> &duration,
                     Predicate &&predicate) noexcept(noexcept(predicate())) {
    static_assert(
        std::is_same<typename std::result_of<Predicate()>::type, bool>::value,
        "Predicate callable must return a boolean value");

    pal_error error = pal_require(is_valid() && pal_has_impl_cv());

    if (error != PAL_SUCCESS) {
      return error;
    }

    if (duration <= std::chrono::microseconds::zero()) {
      return PAL_TIMEOUT;
    }

    pal::chrono::MonotonicClock::duration timeout = duration;
    pal::chrono::MonotonicClock::time_point end =
        pal::chrono::MonotonicClock::now() + duration;

    while (!predicate()) {
      if (timeout <= std::chrono::microseconds::zero()) {
        return PAL_TIMEOUT;
      }

      auto microseconds = pal::chrono::ceil<std::chrono::microseconds>(timeout);

      error = ::pal_cv_wait_for(pal_handle_, scoped_lock.mutex().pal_handle(),
                                static_cast<uint64_t>(microseconds.count()));
      if (error != PAL_SUCCESS) {
        return error;
      }

      timeout = end - pal::chrono::MonotonicClock::now();
    }

    return error;
  }

 private:
  pal_cv_t pal_handle_;
};
}  // namespace pal

#endif  // LIBPAL_CPP_CONDITION_VARIABLE_H
