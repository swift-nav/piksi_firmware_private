#ifndef LIBPAL_CPP_SCOPED_LOCK_H
#define LIBPAL_CPP_SCOPED_LOCK_H

#include <type_traits>
#include <utility>

namespace pal {
/**
 * Similar class to std::scoped_lock, however it doesn't handle multiple
 * mutex objects just yet, otherwise it works in the same manner. Below is
 * a quick example code to use:
 *
 * \code
 * pal::Mutex mutex;
 * {  // start of critical section
 *   pal::ScopedLock<pal::Mutex> lock(mutex);
 *   // do critical section here
 * } // end of critical section
 * \endcode
 *
 * If you'd like, you can even transfer the critical work across to another
 * thread like in this example;
 *
 * \code
 * pal::Mutex mutex;
 * pal::ScopedLock<pal::Mutex> lock(mutex);
 *
 * pal::DynamicThread thread;
 * thread.create([](pal::ScopedLock<pal::Mutex> lock) {
 *   // perform critical section work
 * }, std::move(lock));
 * thread.join();
 * \endcode
 *
 * @tparam T mutex type, currently it only works with pal++::Mutex
 */
template <typename T>
class ScopedLock final {
  static_assert(std::is_same<T, class ::pal::Mutex>::value,
                "pal::ScopedLock currently only works with pal::Mutex");

  friend class ConditionVariable;

 public:
  explicit ScopedLock(T &mutex) noexcept : mutex_(&mutex) {
    if (mutex_) {
      mutex_->lock();
    }
  }

  ScopedLock(const ScopedLock &other) = delete;
  ScopedLock(ScopedLock &&other) noexcept : ScopedLock() {
    std::swap(mutex_, other.mutex_);
  }

  ScopedLock &operator=(const ScopedLock &other) = delete;
  ScopedLock &operator=(ScopedLock &&other) noexcept {
    std::swap(mutex_, other.mutex_);
    return *this;
  }

  ~ScopedLock() {
    if (mutex_) {
      mutex_->unlock();
    }
  }

 private:
  ScopedLock() noexcept : mutex_(nullptr) {}

  T &mutex() noexcept { return *mutex_; }

 private:
  T *mutex_;
};
}  // namespace pal

#endif  // LIBPAL_CPP_SCOPED_LOCK_H
