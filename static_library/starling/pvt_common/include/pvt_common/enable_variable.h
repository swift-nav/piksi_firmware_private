#ifndef PVT_COMMON_ENABLE_VARIABLE_H
#define PVT_COMMON_ENABLE_VARIABLE_H

#include <cassert>
#include <type_traits>
#include <utility>

namespace pvt_common {

/**
 * This class aims to provide the ability to enable/disable a variable through
 * compile time condition.
 *
 * The class can be though of as a std::optional, in the sense that there is
 * either a value or no value at all, and whether it has a value or not is
 * entirely determined at compile time. If there is a value, the class will
 * internally hold the data type specified, otherwise the class will hold no
 * data type, thereby reducing the class size to a single byte.
 *
 * This class hopes to assist users that are trying to reduce their code's
 * text/stack/heap usage.
 *
 * Below is a code snippet to show how one might use the class:
 *
 * @code
 *  class Point {
 *   public:
 *    constexpr Point() = default;
 *    constexpr Point(int x, int y, int z) : x_(x), y_(y), z_(z) {}
 *
 *    int x() const { return x_; }
 *    int y() const { return y_; }
 *    int z() const { return z_; }
 *
 *   private:
 *    int x_{};
 *    int y_{};
 *    int z_{};
 *  };
 *
 *  constexpr bool enable = false;
 *
 *  EnableVariable<Point, enable> point;
 *  printf("Point size: %zu\n", sizeof(point));
 *
 *  if (point) {
 *    assert(sizeof(point) == sizeof(Point));
 *    printf("point.x: %d\n", point->x());
 *    printf("point.y: %d\n", point->y());
 *    printf("point.z: %d\n", point->z());
 *  } else {
 *    assert(sizeof(point) <= 1);
 *  }
 * @endcode
 *
 * If a user sets the `enable` variable to `true`, the code will print out the
 * indivudal `Point` coordintes and the `point` class's size is guaranteed to be
 * equal to that of `Point`.
 *
 * If the user sets the `enable` variable to `false` the code will guarantee
 * that `point`'s class size will consume at most 1 byte. If the variable is
 * disabled and users try and call on the internal type's data (ex. if the user
 * were to have invoked `point->x()` within the else statement), the program
 * will assert, so make sure to place if blocks whenever trying to use the
 * variable as you would with using pointers or optional types.
 *
 * Since the class is compile time deterministic, it helps the compiler in
 * determining if a block of code can be removed entirely from the binary
 * produced. For instance, in the above code, if `enable` is false, the `if`
 * statement and its contents can be removed entirely from the generated binary,
 * this can greatly help remove unnecessary code bits. Note that this only is
 * possible under certain conditions for the class because the `operator bool()`
 * member function depends on the construction's constexpr'ness. To bypass that
 * one can use `decltype(point)::enable` which is always a compile time
 * statement, although the notation isn't all that nice.
 *
 * @note since the class is templated, and the disabled specialization of the
 * class does not call the internal class's constructor directly, the compiled
 * code might work when the variable is disabled, but than fail when enabled.
 * the class will try and do everything it can to try mimic the enabled
 * behaviour, therefore it is advices to make sure you always compile with both
 * enabled/disabled options and test that it work.
 *
 * @tparam T data type to enable/disable variable
 * @tparam Enable compile time conditional, a true value enables the variable,
 * otherwise it doesn't
 */
template <typename T, bool Enable>
class EnableVariable;

template <typename T>
class EnableVariable<T, true> {
 public:
  static constexpr bool enabled = true;

 public:
  /**
   * Default Constructor
   */
  constexpr EnableVariable() : content_() {}

  /**
   * Copy Constructor
   *
   * @note might seem strange that there is non const version of a copy
   * constructor, but the reason why this is needed is because if you have
   * something like this:
   *
   * @code
   *  EnableVariable<int, true> var1(1);
   *  EnableVariable<int, true> var2(var1);
   * @endcode
   *
   * The second line will hit the `EnableVariable(Args &&... args)` (aka
   * Universal Value Constructor) over the copy constructor with the const
   * qualifier.
   *
   * @param other object to copy from
   */
  constexpr EnableVariable(EnableVariable &other) : content_(other.content_) {}

  /**
   * Copy Constructor
   *
   * @param other object to copy from
   */
  constexpr EnableVariable(const EnableVariable &other)
      : content_(other.content_) {}

  /**
   * Move Constructor
   *
   * @param other object to move from
   */
  constexpr EnableVariable(EnableVariable &&other) noexcept
      : content_(std::move(other.content_)) {}

  /**
   * Universal Value Constructor
   *
   * @tparam Args list of argument types
   * @param args list of arguments
   */
  template <typename... Args>
  constexpr explicit EnableVariable(Args &&... args)
      : content_{std::forward<Args>(args)...} {}

  /**
   * Copy Assignment
   *
   * @note much like the EnableVariable(EnableVariable&) copy constructor, we
   * need a none const version of the copy assignment for much the same reason.
   *
   * @param other object to assign from
   * @return this object
   */
  // NOLINTNEXTLINE
  constexpr EnableVariable &operator=(EnableVariable &other) {
    content_ = other.content_;
    return *this;
  }

  /**
   * Copy Assignment
   *
   * @param other object to assign from
   * @return this object
   */
  constexpr EnableVariable &operator=(const EnableVariable &other) {
    content_ = other.content_;
    return *this;
  }

  /**
   * Move Assignment
   *
   * @param other object to assign from
   * @return this object
   */
  constexpr EnableVariable &operator=(EnableVariable &&other) noexcept {
    content_ = std::move(other.content_);
    return *this;
  }

  /**
   * Universal Value Assignment
   *
   * @tparam U object to assign this from
   * @param other object to assign from
   * @return this object
   */
  template <typename U>
  constexpr EnableVariable &operator=(U &&other) {
    content_ = std::forward<U>(other);
    return *this;
  }

  constexpr explicit operator bool() const { return enabled; }

  constexpr T &value() { return content_; }
  constexpr const T &value() const { return content_; }

  constexpr T *get() { return &content_; }
  constexpr const T *get() const { return &content_; }

  constexpr T &unsafe_value() { return content_; }
  constexpr const T &unsafe_value() const { return content_; }

  constexpr T *unsafe_get() { return &content_; }
  constexpr const T *unsafe_get() const { return &content_; }

  constexpr T &operator*() { return content_; }
  constexpr const T &operator*() const { return content_; }

  constexpr T *operator->() { return &content_; }
  constexpr const T *operator->() const { return &content_; }

 private:
  T content_;
};

template <typename T>
class EnableVariable<T, false> {
 public:
  static constexpr bool enabled = false;

 public:
  /**
   * Default Constructor
   */
  constexpr EnableVariable() {
    static_assert(std::is_default_constructible<T>::value,
                  "Underlying type does not define a default constructor");
  }

  /**
   * Copy Constructor
   *
   * @param other object to copy from
   */
  constexpr EnableVariable(const EnableVariable & /*unused*/) {
    static_assert(std::is_copy_constructible<T>::value,
                  "Underlying type does not define a copy constructor");
  }

  /**
   * Move Constructor
   *
   * @param other object to move from
   */
  constexpr EnableVariable(EnableVariable && /*unused*/) noexcept {
    static_assert(std::is_move_constructible<T>::value,
                  "Underlying type does not define a move constructor");
  }

  /**
   * Universal Value Constructor
   *
   * @tparam Args list of argument types
   * @param args list of arguments
   */
  template <typename... Args>
  constexpr explicit EnableVariable(Args &&... /*unused*/) {}

  /**
   * Copy Assignment
   *
   * @param other object to assign from
   * @return this object
   */
  constexpr EnableVariable &operator=(const EnableVariable & /*unused*/) {
    static_assert(std::is_copy_assignable<T>::value,
                  "Underlying type does not define a copy assignment operator");
    return *this;
  }

  /**
   * Move Assignment
   *
   * @param other object to assign from
   * @return this object
   */
  // NOLINTNEXTLINE
  constexpr EnableVariable &operator=(EnableVariable && /*unused*/) {
    static_assert(std::is_move_assignable<T>::value,
                  "Underlying type does not define a move assignment operator");
    return *this;
  }

  /**
   * Universal Value Assignment
   *
   * @tparam U object to assign this from
   * @return this object
   */
  template <typename U>
  constexpr EnableVariable &operator=(U && /*unused*/) {
    return *this;
  }

  constexpr explicit operator bool() const { return enabled; }

  constexpr T &value() {
    assert(false && "variable is disabled");
    // This line was previously constructing a NULL reference but has been
    // changed to this hack to try to avoid build errors with a compiler which
    // enforces code safety issues like dereferencing NULL. Casting this back to
    // T is obviously as wrong as dereferencing NULL but should allow the build
    // to continue.
    return *reinterpret_cast<T *>(this);  // NOLINT
  }
  constexpr const T &value() const {
    assert(false && "variable is disabled");
    // This line was previously constructing a NULL reference but has been
    // changed to this hack to try to avoid build errors with a compiler which
    // enforces code safety issues like dereferencing NULL. Casting this back to
    // T is obviously as wrong as dereferencing NULL but should allow the build
    // to continue.
    return *reinterpret_cast<const T *>(this);  // NOLINT
  }

  constexpr T *get() {
    assert(false && "variable is disabled");
    return nullptr;
  }
  constexpr const T *get() const {
    assert(false && "variable is disabled");
    return nullptr;
  }

  constexpr T &unsafe_value() {
    // This line was previously constructing a NULL reference but has been
    // changed to this hack to try to avoid build errors with a compiler which
    // enforces code safety issues like dereferencing NULL. Casting this back to
    // T is obviously as wrong as dereferencing NULL but should allow the build
    // to continue.
    return *reinterpret_cast<T *>(this);  // NOLINT
  }
  constexpr const T &unsafe_value() const {
    // This line was previously constructing a NULL reference but has been
    // changed to this hack to try to avoid build errors with a compiler which
    // enforces code safety issues like dereferencing NULL. Casting this back to
    // T is obviously as wrong as dereferencing NULL but should allow the build
    // to continue.
    return *reinterpret_cast<const T *>(this);  // NOLINT
  }

  constexpr T *unsafe_get() { return nullptr; }
  constexpr const T *unsafe_get() const { return nullptr; }

  constexpr T &operator*() {
    assert(false && "variable is disabled");
    // This line was previously constructing a NULL reference but has been
    // changed to this hack to try to avoid build errors with a compiler which
    // enforces code safety issues like dereferencing NULL. Casting this back to
    // T is obviously as wrong as dereferencing NULL but should allow the build
    // to continue.
    return *reinterpret_cast<T *>(this);  // NOLINT
  }
  constexpr const T &operator*() const {
    assert(false && "variable is disabled");
    // This line was previously constructing a NULL reference but has been
    // changed to this hack to try to avoid build errors with a compiler which
    // enforces code safety issues like dereferencing NULL. Casting this back to
    // T is obviously as wrong as dereferencing NULL but should allow the build
    // to continue.
    return *reinterpret_cast<const T *>(this);  // NOLINT
  }

  constexpr T *operator->() {
    assert(false && "variable is disabled");
    return nullptr;
  }
  constexpr const T *operator->() const {
    assert(false && "variable is disabled");
    return nullptr;
  }
};

}  // namespace pvt_common

#endif  // PVT_COMMON_ENABLE_VARIABLE_H
