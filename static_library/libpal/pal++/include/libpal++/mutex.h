#ifndef LIBPAL_CPP_MUTEX_H
#define LIBPAL_CPP_MUTEX_H

#include <libpal/synch/mutex.h>

#include <utility>

namespace pal {
/**
 * Class offers users the ability to create a mutex object.
 *
 * The class interface mirrors mostly how std::mutex does things, however the
 * locking/unlocking methods have been restricted from user access to prevent
 * bad practices and triggering platform non-deterministic behaviour. Please
 * use pal::ScopedLock to manage locking and unlocking of pal::Mutex object.
 *
 * Differing from std::mutex functions in this class return libpal error codes
 * and an instance must be explicitly initialized before use (#init)
 */
class Mutex final {
  friend class ConditionVariable;

  template <typename T>
  friend class ScopedLock;

 public:
  Mutex() noexcept : pal_handle_() {}

  Mutex(const Mutex &other) = delete;
  Mutex(Mutex &&other) noexcept : pal_handle_() {
    std::swap(pal_handle_, other.pal_handle_);
  }

  Mutex &operator=(const Mutex &other) = delete;
  Mutex &operator=(Mutex &&other) noexcept {
    std::swap(pal_handle_, other.pal_handle_);
    return *this;
  }

  ~Mutex() { deinit(); }

  /**
   * Initialize mutex ready for use
   *
   * Allocate resources from libpal. Once this class has been instantiated it
   * must be initialized by calling this function. Only after this function
   * returns PAL_SUCCESS will the mutex be available to use by calling other
   * member functions.
   *
   * @return PAL error code
   */
  pal_error init() noexcept;

  /**
   * Deinitialize mutex
   *
   * Release all PAL resources used by this instance. After this function
   * returns PAL_SUCCESS the mutex must not be used unless it is reinitialized
   * by a call to init().
   *
   * This function will be called automatically when the instance of Mutex is
   * destroyed.
   *
   * @return PAL error code
   */
  pal_error deinit() noexcept;

  /**
   * Test whether this instance is valid, whether it is available for use
   *
   * A mutex is valid (ie, ready to be used) once it has been initialized by a
   * call to init(). When this function returns true it is safe to use other
   * functions in the class. If this function returns false this instance must
   * not be used.
   *
   * @return true if the instance is valid (ready to use), false otherwise
   */
  bool is_valid() const noexcept { return pal_handle_ != nullptr; }

 private:
  pal_error lock() noexcept;
  pal_error unlock() noexcept;

  inline pal_mutex_t &pal_handle() noexcept { return pal_handle_; };

 private:
  pal_mutex_t pal_handle_;
};
}  // namespace pal

#endif  // LIBPAL_CPP_MUTEX_H
