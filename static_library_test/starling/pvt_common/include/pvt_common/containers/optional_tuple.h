#ifndef PVT_COMMON_CONTAINERS_OPTIONAL_TUPLE_H
#define PVT_COMMON_CONTAINERS_OPTIONAL_TUPLE_H

#include <bitset>
#include <cstddef>
#include <type_traits>
#include <utility>

#include <pvt_common/optional.h>

namespace pvt_common {
namespace containers {
namespace detail {
/**
 * Backport of C++17 `std::conjunction`, implementation from
 * https://en.cppreference.com/w/cpp/types/conjunction
 */
template <class...>
struct conjunction : std::true_type {};
template <class B1>
struct conjunction<B1> : B1 {};
template <class B1, class... Bn>
struct conjunction<B1, Bn...>
    : std::conditional_t<bool(B1::value), conjunction<Bn...>, B1> {};
}  // namespace detail

/**
 * Normally holding an array of optional variables is expensive in terms of
 * memory usage. The reason for this is due to data alignment constraints
 * imposed on most common CPU architectures. For instance, on an x86_64 CPU,
 * an optional<double> variable would occupy 16 bytes of memory (as opposed to
 * the expected 9 bytes, 8 for the double, 1 for the bool).
 *
 * This container aims to provide the ability to intantiate a tuple of optional
 * values in a more memory efficient manner.
 *
 * @note the types are stored in memory in the reverse order specified, keep
 * this in mind when trying to efficiently pack your data. so for example:
 *
 * @code
 *   OptionalTupleImpl<double, int, bool, bool> var;
 * @endcode
 *
 * Would be logically equivalent to:
 *
 * @code
 *   optional<bool> var1;
 *   optional<bool> var2;
 *   optional<int> var3;
 *   optional<double> var4;
 * @endcode
 *
 * @tparam Types list of optional types to hold
 */
template <typename... Types>
struct OptionalTuple;

/**
 * Class holds a "view" to an entry within OptionalTuple. Its interface it
 * designed in such a way that it acts like `optional` but internally its
 * value and stored/managed by the OptionalTuple.
 *
 * @note by design, its currently impossible to move the content from an
 * OptionalView to another, this is because OptionalTuple::get<>() will
 * always return a RValue type, which makes it difficult to distinguish between
 * what the user is trying to do, so by default it will always copy.
 *
 * @tparam Type optional types held by class
 */
template <typename Type>
class OptionalView {
 public:
  using value_type = Type;

 public:
  OptionalView(Type &value, bool (*has_value_get)(const void *, size_t),
               void (*has_value_set)(void *, size_t, bool),
               void *has_value_context, size_t has_value_index)
      : value_(value),
        has_value_get_(has_value_get),
        has_value_set_(has_value_set),
        has_value_context_(has_value_context),
        has_value_index_(has_value_index) {}

  OptionalView(const OptionalView &) = default;
  // NOLINTNEXTLINE
  OptionalView(OptionalView &&other) noexcept : OptionalView(other) {}

  OptionalView &operator=(const OptionalView<value_type> &other) {
    if (has_value_context_ == other.has_value_context_ &&
        has_value_index_ == other.has_value_index_) {
      return *this;
    }

    if (other) {
      operator=(other.value());
    } else {
      reset();
    }
    return *this;
  }
  OptionalView &operator=(OptionalView<value_type> &&other) noexcept {
    return operator=(other);  // NOLINT
  }

  OptionalView &operator=(const optional<value_type> &other) {
    if (other) {
      operator=(*other);
    } else {
      reset();
    }
    return *this;
  }
  OptionalView &operator=(optional<value_type> &&other) {
    if (other) {
      operator=(std::move(*other));
    } else {
      reset();
    }
    return *this;
  }

  OptionalView &operator=(const value_type &other) {
    if (has_value()) {
      value_ = other;
    } else {
      new (&value_) value_type(other);
      has_value_set_(has_value_context_, has_value_index_, true);
    }
    return *this;
  }
  OptionalView &operator=(value_type &&other) {
    if (has_value()) {
      value_ = std::move(other);
    } else {
      new (&value_) value_type(std::move(other));
      has_value_set_(has_value_context_, has_value_index_, true);
    }
    return *this;
  }

  explicit operator bool() const noexcept {
    return has_value_get_(has_value_context_, has_value_index_);
  }
  bool has_value() const noexcept {
    return has_value_get_(has_value_context_, has_value_index_);
  }

  const value_type *operator->() const {
    assert(has_value());
    return &value_;
  }
  value_type *operator->() {
    assert(has_value());
    return &value_;
  }
  const value_type &operator*() const {
    assert(has_value());
    return value_;
  }
  value_type &operator*() {
    assert(has_value());
    return value_;
  }
  const value_type &value() const {
    assert(has_value());
    return value_;
  }
  value_type &value() {
    assert(has_value());
    return value_;
  }

  template <class U>
  constexpr value_type value_or(U &&default_value) const & {
    if (has_value()) {
      return value_;
    }
    return std::forward<U>(default_value);
  }

  void reset() noexcept {
    if (has_value()) {
      value_.~value_type();
      has_value_set_(has_value_context_, has_value_index_, false);
    }
  }

  template <class... Args>
  value_type &emplace(Args &&... args) {
    reset();
    new (&value_) value_type(std::forward<Args>(args)...);
    has_value_set_(has_value_context_, has_value_index_, true);
    return value_;
  }

  optional<Type> std_optional() const {
    optional<Type> ret;

    if (has_value()) {
      ret.emplace(value());
    }

    return ret;
  }

 private:
  Type &value_;
  bool (*const has_value_get_)(const void *, size_t);
  void (*const has_value_set_)(void *, size_t, bool);
  void *const has_value_context_;
  const size_t has_value_index_;
};

template <typename T>
bool operator==(const OptionalView<T> &left, const OptionalView<T> &right) {
  if (!left && !right) {
    return true;
  }

  if (left && right) {
    return left.value() == right.value();
  }

  return false;
}

template <typename T>
bool operator!=(const OptionalView<T> &left, const OptionalView<T> &right) {
  return !operator==(left, right);
}

template <typename... Types>
class OptionalTupleImpl {
 private:
  template <size_t Index>
  using tuple_element_t = std::tuple_element_t<Index, std::tuple<Types...>>;

 private:
  /**
   * Helper element to for_each_value which expands out the index values.
   */
  template <class Tuple, class F, std::size_t... I>
  static constexpr void for_each_value_impl(
      Tuple &&t, F &&f, std::index_sequence<I...> /*unused*/) {
    (void)std::initializer_list<int>{(
        std::forward<F>(f)(
            I, *static_cast<std::add_pointer_t<
                   typename std::tuple_element<I, std::tuple<Types...>>::type>>(
                   static_cast<void *>(&std::get<I>(std::forward<Tuple>(t))))),
        0)...};
  }

  /**
   * Helper method that allows us to iterator through each tuple element in
   * order.
   *
   * @tparam Tuple list of tuple types
   * @tparam F lambda function type
   * @param t reference to the class's values_ member variable
   * @param f lambda function which accepts the tuple element's index value as
   * its first parameter, followed by a reference to the tuple element's value.
   */
  template <class Tuple, class F>
  static constexpr void for_each_value(Tuple &&t, F &&f) {
    for_each_value_impl(
        std::forward<Tuple>(t), std::forward<F>(f),
        std::make_index_sequence<
            std::tuple_size<std::remove_reference_t<Tuple>>::value>{});
  }

  /**
   * Obtains reference to tuple element. Normally one could call
   * `std::get<Index>(values_)` to obtain reference to tuple element, but since
   * the tuple is a set of `std::aligned_storage_t` elements, we need to
   * reinterpret cast them to their underlying abstracted types.
   *
   * @tparam Index tuple index value
   * @return reference to the tuple element
   */
  template <size_t Index>
  constexpr const tuple_element_t<Index> &get_element() const {
    return *static_cast<const tuple_element_t<Index> *>(
        static_cast<const void *>(&std::get<Index>(values_)));
  }

  /**
   * Obtains reference to tuple element. Normally one could call
   * `std::get<Index>(values_)` to obtain reference to tuple element, but since
   * the tuple is a set of `std::aligned_storage_t` elements, we need to
   * reinterpret cast them to their underlying abstracted types.
   *
   * @tparam Index tuple index value
   * @return reference to the tuple element
   */
  template <size_t Index>
  constexpr tuple_element_t<Index> &get_element() {
    return *static_cast<tuple_element_t<Index> *>(
        static_cast<void *>(&std::get<Index>(values_)));
  }

  /**
   * Indirect way to call `OptionalTupleImpl::has_value_.operator[](size_t)`,
   * needed in order to allow for type erasure access from `OptionalView`.
   *
   * @param context pointer to this class (OptionalTupleImpl)
   * @param index index within tuple to access
   * @return result of has_value_[index]
   */
  static bool has_value_get(const void *context, size_t index) {
    return static_cast<const OptionalTupleImpl *>(context)->has_value_[index];
  }

  /**
   * Indirect way to call `OptionalTupleImpl::has_value_.set(size_t, bool)`,
   * needed in order to allow for type erasure access from `OptionalView`.
   *
   * @param context pointer to this class (OptionalTupleImpl)
   * @param index index within tuple to access
   * @return result of has_value_.set(index, value)
   */
  static void has_value_set(void *context, size_t index, bool value) {
    static_cast<OptionalTupleImpl *>(context)->has_value_.set(index, value);
  }

 public:
  /**
   * Constructs all optional types as empty
   */
  OptionalTupleImpl() : values_(), has_value_() {}

  /**
   * Obtain view to the `Index` element of the tuple.
   *
   * @tparam Index index value of the optional view of interest
   * @return optional view to the tuple element
   */
  template <size_t Index>
  const OptionalView<tuple_element_t<Index>> get() const {
    return OptionalView<tuple_element_t<Index>>(
        // NOLINTNEXTLINE
        const_cast<tuple_element_t<Index> &>(get_element<Index>()),
        // NOLINTNEXTLINE
        has_value_get, has_value_set, const_cast<OptionalTupleImpl *>(this),
        Index);
  }

  /**
   * Obtain view to the `Index` element of the tuple.
   *
   * @tparam Index index value of the optional view of interest
   * @return optional view to the tuple element
   */
  template <size_t Index>
  OptionalView<tuple_element_t<Index>> get() {
    return OptionalView<tuple_element_t<Index>>(
        get_element<Index>(), has_value_get, has_value_set, this, Index);
  }

 protected:
  /**
   * Destorys all tuple elements within the class that have values to them. This
   * will mean that all OptionalView's are invalidated, that is why this is a
   * private method that should only be called conditionally during
   * deconstruction of this class.
   */
  void reset() {
    for_each_value(values_, [this](size_t index, auto &value) {
      using value_type = std::decay_t<decltype(value)>;

      if (has_value_[index]) {
        value.~value_type();
      }
    });
  }

 private:
  std::tuple<std::aligned_storage_t<sizeof(Types), alignof(Types)>...> values_;
  std::bitset<sizeof...(Types)> has_value_;
};

template <typename... Types>
struct OptionalTupleImplNoDestuctor : public OptionalTupleImpl<Types...> {};

template <typename... Types>
struct OptionalTupleImplDestructible : public OptionalTupleImpl<Types...> {
  ~OptionalTupleImplDestructible() { OptionalTupleImpl<Types...>::reset(); }
};

template <typename... Types>
struct OptionalTuple
    : public std::conditional_t<
          std::is_trivially_destructible<std::tuple<Types...>>::value,
          OptionalTupleImplNoDestuctor<Types...>,
          OptionalTupleImplDestructible<Types...>> {};

}  // namespace containers
}  // namespace pvt_common
#endif  // PVT_COMMON_CONTAINERS_OPTIONAL_LIST_H
