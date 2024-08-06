#ifndef LIBPAL_CPP_FUNCTIONAL_H
#define LIBPAL_CPP_FUNCTIONAL_H

/**
 * Header offers libpal++ developers the ability to use various
 * template functions available in "#include <functional>" which aren't
 * available in C++14 standard. When we eventually move over to higher
 * versions which offers the same functionality we can start refactoring
 * the code to use them over this.
 */

#include <type_traits>

namespace pal {

/**
 * Implementation for std::invoke, code used below is a modified version of what
 * is presented in cppreference.com.
 *
 * https://en.cppreference.com/w/cpp/utility/functional/invoke#Possible_implementation
 */

namespace detail {
template <class>
struct is_reference_wrapper : public std::false_type {};

template <class T>
struct is_reference_wrapper<std::reference_wrapper<T>> : public std::true_type {
};

struct InvokeSubCategoryIsBaseOf {};
struct InvokeSubCategoryIsReferenceWrapper {};
struct InvokeSubCategoryBaseCase {};

template <bool IsPointerToMemberFunction, class>
struct InvokeCondition;

template <>
struct InvokeCondition<true, InvokeSubCategoryIsBaseOf> {
  template <class T, class Type, class T1, class... Args>
  static constexpr decltype(auto) invoke(Type T::*f, T1 &&t1, Args &&... args) {
    return (std::forward<T1>(t1).*f)(std::forward<Args>(args)...);
  }
};

template <>
struct InvokeCondition<true, InvokeSubCategoryIsReferenceWrapper> {
  template <class T, class Type, class T1, class... Args>
  static constexpr decltype(auto) invoke(Type T::*f, T1 &&t1, Args &&... args) {
    return (t1.get().*f)(std::forward<Args>(args)...);
  }
};

template <>
struct InvokeCondition<true, InvokeSubCategoryBaseCase> {
  template <class T, class Type, class T1, class... Args>
  static constexpr decltype(auto) invoke(Type T::*f, T1 &&t1, Args &&... args) {
    return ((*std::forward<T1>(t1)).*f)(std::forward<Args>(args)...);
  }
};

template <>
struct InvokeCondition<false, InvokeSubCategoryIsBaseOf> {
  template <class T, class Type, class T1, class... Args>
  static constexpr decltype(auto) invoke(Type T::*f, T1 &&t1, Args &&... args) {
    static_assert(std::is_member_object_pointer<decltype(f)>::value, "");
    static_assert(sizeof...(args) == 0, "");

    return std::forward<T1>(t1).*f;
  }
};

template <>
struct InvokeCondition<false, InvokeSubCategoryIsReferenceWrapper> {
  template <class T, class Type, class T1, class... Args>
  static constexpr decltype(auto) invoke(Type T::*f, T1 &&t1, Args &&... args) {
    static_assert(std::is_member_object_pointer<decltype(f)>::value, "");
    static_assert(sizeof...(args) == 0, "");

    return t1.get().*f;
  }
};

template <>
struct InvokeCondition<false, void> {
  template <class T, class Type, class T1, class... Args>
  static constexpr decltype(auto) invoke(Type T::*f, T1 &&t1, Args &&... args) {
    static_assert(std::is_member_object_pointer<decltype(f)>::value, "");
    static_assert(sizeof...(args) == 0, "");

    return (*std::forward<T1>(t1)).*f;
  }
};

template <class T, class Type, class T1, class... Args>
constexpr decltype(auto) INVOKE(Type T::*f, T1 &&t1, Args &&... args) {
  constexpr bool isPointerToMemberFunction =
      std::is_member_function_pointer<decltype(f)>::value;
  using InvokeSubcategory = std::conditional_t<
      std::is_base_of<T, std::decay_t<T1>>::value, InvokeSubCategoryIsBaseOf,
      std::conditional_t<is_reference_wrapper<std::decay_t<T1>>::value,
                         InvokeSubCategoryIsReferenceWrapper,
                         InvokeSubCategoryBaseCase>>;

  return InvokeCondition<isPointerToMemberFunction, InvokeSubcategory>::invoke(
      std::forward<decltype(f)>(f), std::forward<T1>(t1),
      std::forward<Args>(args)...);
}

template <class F, class... Args>
constexpr decltype(auto) INVOKE(F &&f, Args &&... args) {
  return std::forward<F>(f)(std::forward<Args>(args)...);
}
}  // namespace detail

template <class Callable, class... Args>
constexpr std::result_of_t<Callable(Args...)> invoke(Callable &&callable,
                                                     Args &&... args) {
  return detail::INVOKE(std::forward<Callable>(callable),
                        std::forward<Args>(args)...);
}
}  // namespace pal

#endif  // LIBPAL_CPP_FUNCTIONAL_H
