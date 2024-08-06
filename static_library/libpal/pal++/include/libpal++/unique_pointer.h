#ifndef LIBPAL_CPP_UNIQUE_POINTER_H
#define LIBPAL_CPP_UNIQUE_POINTER_H

#include <functional>
#include <new>
#include <type_traits>
#include <utility>

#include <libpal/mem/mem.h>

namespace pal {
namespace detail {
/**
 * Serves the purpose of making sure that a platform abstraction layer memory
 * allocated  pointer is properly released back to the platform on the event
 * that an exception is thrown or variable goes out of scope.
 */
class PointerScopeGuard final {
 public:
  explicit PointerScopeGuard(void *pointer) noexcept : pointer_(pointer) {}

  PointerScopeGuard(const PointerScopeGuard &) = delete;
  PointerScopeGuard(PointerScopeGuard &&) = delete;

  PointerScopeGuard &operator=(const PointerScopeGuard &) = delete;
  PointerScopeGuard &operator=(PointerScopeGuard &&) = delete;

  ~PointerScopeGuard() { ::pal_mem_free(&pointer_); }

  void *release() noexcept {
    void *pointer = pointer_;
    pointer_ = nullptr;
    return pointer;
  }

 private:
  void *pointer_;
};
}  // namespace detail

/**
 * Class operates in more or less the same nature that std::unique_ptr does, in
 * that it manages the scope of a pointer. The main driving difference to having
 * a platform abstraction layer implementation over using std::unique_ptr is
 * that the standard library implementation uses the delete operator directly to
 * release its memory back to the platform, and this library has no control over
 * the implementation that delete offers. As such the need for this class.
 *
 * Having said that, users will need to allocate the pointer manually using
 * pal_mem_alloc() function if they wish to explicitly pass it onto this
 * classes constructor. Otherwise, the better and recommended approach is to use
 * the pal::make_unique<T, Args...>(Args&&...) implementation as it does the
 * allocation and construction of the underlying pointer for you.
 *
 * @note there is no specialization for UniquePointer<T[]>
 *
 * @tparam T pointer type to manage
 */
template <typename T>
class UniquePointer final {
 public:
  /**
   * Constructs an empty class that owns nothing.
   */
  UniquePointer() noexcept : UniquePointer(nullptr) {}

  /**
   * Constructs an instance which which owns the platform abstraction layer
   * allocated @p pointer.
   *
   * @param pointer pointer which was allocated via pal_mem_alloc() function
   */
  explicit UniquePointer(T *pointer) noexcept : pointer_(pointer) {}

  UniquePointer(const UniquePointer &) noexcept = delete;
  UniquePointer &operator=(const UniquePointer &) noexcept = delete;

  /**
   * Constructs an instance by transferring ownership of underlying pointer from
   * @p other to *this and stores the null pointer in @p other.
   *
   * @note the pointer held by @p other must either be identical to the one
   * managed by *this, or it must be of a derived class type.
   *
   * @tparam U pointer type managed by @p other
   * @param other unique pointer to transfer ownership from
   */
  template <typename U>
  UniquePointer(UniquePointer<U> &&other) noexcept  // NOLINT
      : UniquePointer() {
    static_assert(std::is_same<T, U>::value || std::is_base_of<T, U>::value,
                  "The pointer types are incompatible");

    pointer_ = other.release();
  }

  /**
   * Transfers ownership from of underlying pointer from @p other to *this.
   *
   * @note the pointer held by @p other must either be identical to the one
   * managed by *this, or it must be of a derived class type.
   *
   * @tparam U pointer type managed by @p other
   * @param other unique pointer to transfer ownership from
   */
  template <typename U>
  UniquePointer &operator=(UniquePointer<U> &&other) noexcept {
    static_assert(std::is_same<T, U>::value || std::is_base_of<T, U>::value,
                  "The pointer types are incompatible");

    reset();
    pointer_ = other.release();

    return *this;
  }

  /**
   * Deconstructor deallocates managed underlying pointer.
   */
  ~UniquePointer() { reset(); }

  /**
   * Releases the ownership of the managed pointer.
   *
   * @return pointer to the managed object
   */
  T *release() noexcept {
    T *pointer = pointer_;
    pointer_ = nullptr;
    return pointer;
  }

  /**
   * Deallocates the underlying managed pointer, and makes the instance into
   * an empty class that owns nothing.
   */
  void reset() noexcept {
    if (pointer_ == nullptr) {
      return;
    }

    pointer_->~T();

    void *void_pointer = pointer_;
    ::pal_mem_free(&void_pointer);

    pointer_ = nullptr;
  }

  /**
   * Swaps the managed objects
   *
   * @param other unique pointer to swap ownership with
   */
  void swap(UniquePointer &other) noexcept {
    std::swap(pointer_, other.pointer_);
  }

  /**
   * @return pointer to the managed object, nullptr if it owns nothing
   */
  T *get() const noexcept { return pointer_; }

  /**
   * @return reference to the managed object
   */
  T &operator*() const noexcept { return *get(); }

  /**
   * @return pointer to the managed object, nullptr if it owns nothing
   */
  T *operator->() const noexcept { return get(); }

  /**
   * @return true if *this owns an object, false otherwise
   */
  explicit operator bool() const noexcept { return get() != nullptr; }

 private:
  T *pointer_;
};

/**
 * Allocates a memory region to store the an object of type @p T. Operates in
 * the same way that std::make_unique<T, Args...> does.
 *
 * @note the request for memory allocation is made to the platform abstraction
 * layer library.
 *
 * @tparam T data type to allocated memory for
 * @tparam Args arguments types that are passed to the constructor of T
 * @param args arguments that are passed to the constructor of T
 * @return unique pointer which holds the data type. it may return an empty
 * unique pointer if there was an error during memory allocation.
 */
template <typename T, typename... Args>
UniquePointer<T> make_unique(Args &&... args) noexcept(
    std::is_nothrow_constructible<T, Args &&...>().value) {
  void *void_pointer;
  pal_error error = ::pal_mem_alloc(&void_pointer, sizeof(T));

  if (error != PAL_SUCCESS) {
    return UniquePointer<T>();
  }

  detail::PointerScopeGuard scope_guard(void_pointer);

  T *pointer = static_cast<T *>(void_pointer);
  new (pointer) T(std::forward<Args>(args)...);

  return UniquePointer<T>(static_cast<T *>(scope_guard.release()));  // NOLINT
}

template <typename T, typename U>
inline bool operator==(const UniquePointer<T> &lhs,
                       const UniquePointer<U> &rhs) noexcept {
  return lhs.get() == rhs.get();
}

template <typename T, typename U>
inline bool operator!=(const UniquePointer<T> &lhs,
                       const UniquePointer<U> &rhs) noexcept {
  return lhs.get() != rhs.get();
}

template <typename T, typename U>
inline bool operator<(const UniquePointer<T> &lhs,
                      const UniquePointer<U> &rhs) noexcept {
  return lhs.get() < rhs.get();
}

template <typename T, typename U>
inline bool operator<=(const UniquePointer<T> &lhs,
                       const UniquePointer<U> &rhs) noexcept {
  return lhs.get() <= rhs.get();
}

template <typename T, typename U>
inline bool operator>(const UniquePointer<T> &lhs,
                      const UniquePointer<U> &rhs) noexcept {
  return lhs.get() > rhs.get();
}

template <typename T, typename U>
inline bool operator>=(const UniquePointer<T> &lhs,
                       const UniquePointer<U> &rhs) noexcept {
  return lhs.get() >= rhs.get();
}

template <typename T>
inline bool operator==(const UniquePointer<T> &lhs,
                       std::nullptr_t rhs) noexcept {
  (void)rhs;
  return lhs.get() == nullptr;
}

template <typename T>
inline bool operator==(std::nullptr_t lhs,
                       const UniquePointer<T> &rhs) noexcept {
  (void)lhs;
  return lhs == rhs.get();
}

template <typename T>
inline bool operator!=(const UniquePointer<T> &lhs,
                       std::nullptr_t rhs) noexcept {
  (void)rhs;
  return lhs.get() != nullptr;
}

template <typename T>
inline bool operator!=(std::nullptr_t lhs,
                       const UniquePointer<T> &rhs) noexcept {
  (void)lhs;
  return lhs != rhs.get();
}

template <typename T>
inline bool operator<(const UniquePointer<T> &lhs,
                      std::nullptr_t rhs) noexcept {
  (void)rhs;
  return std::less<const T *>()(lhs.get(), nullptr);
}

template <typename T>
inline bool operator<(std::nullptr_t lhs,
                      const UniquePointer<T> &rhs) noexcept {
  (void)lhs;
  return std::less<const T *>()(nullptr, rhs.get());
}

template <typename T>
inline bool operator<=(const UniquePointer<T> &lhs,
                       std::nullptr_t rhs) noexcept {
  (void)rhs;
  return !(nullptr < lhs);
}

template <typename T>
inline bool operator<=(std::nullptr_t lhs,
                       const UniquePointer<T> &rhs) noexcept {
  (void)lhs;
  return !(rhs < nullptr);
}

template <typename T>
inline bool operator>(const UniquePointer<T> &lhs,
                      std::nullptr_t rhs) noexcept {
  (void)rhs;
  return nullptr < lhs;
}

template <typename T>
inline bool operator>(std::nullptr_t lhs,
                      const UniquePointer<T> &rhs) noexcept {
  (void)lhs;
  return rhs < nullptr;
}

template <typename T>
inline bool operator>=(const UniquePointer<T> &lhs,
                       std::nullptr_t rhs) noexcept {
  (void)rhs;
  return !(lhs < nullptr);
}

template <typename T>
inline bool operator>=(std::nullptr_t lhs,
                       const UniquePointer<T> &rhs) noexcept {
  (void)lhs;
  return !(nullptr < rhs);
}

}  // namespace pal

#endif  // LIBPAL_CPP_UNIQUE_POINTER_H
