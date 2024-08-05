//
// Copyright (C) 2019 Swift Navigation Inc.
// Contact: Swift Navigation <dev@swiftnav.com>
//
// This source is subject to the license found in the file 'LICENSE' which must
// be be distributed together with this source. All other rights reserved.
//
// THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
// EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
//

#ifndef SENSORFUSION_CORE_TRAITS_H
#define SENSORFUSION_CORE_TRAITS_H

#include <tuple>
#include <type_traits>

namespace sensorfusion {
namespace traits {

// checks to see if all booleans in an initializer list are true.
template <bool...>
struct bool_pack {};
template <bool... B>
using boolean_all = std::is_same<bool_pack<B..., true>, bool_pack<true, B...>>;
template <bool... B>
constexpr bool boolean_all_v = boolean_all<B...>::value;

// Checks to see if a type is a std::tuple
template <typename T>
struct is_tuple : public std::false_type {};
template <typename... Ts>
struct is_tuple<std::tuple<Ts...>> : public std::true_type {};

// Applies `MatcherTrait` to all types of a tuple `TupleT` (from last to first),
// comparing against `KeyT`, until `MatcherTrait` finds a match, or all types
// have been checked
template <template <typename, typename> class MatcherTrait, typename TupleT,
          typename KeyT, size_t I>
struct tuple_search {
  static constexpr bool value =
      MatcherTrait<typename std::tuple_element<I, TupleT>::type, KeyT>::value ||
      tuple_search<MatcherTrait, TupleT, KeyT, I - 1>::value;
};

template <template <typename, typename> class MatcherTrait, typename TupleT,
          typename KeyT>
struct tuple_search<MatcherTrait, TupleT, KeyT, 0> {
  static constexpr bool value =
      MatcherTrait<typename std::tuple_element<0, TupleT>::type, KeyT>::value;
};

// Checks if a type TupleT is both a tuple, and contains the type KeyT
template <typename TupleT, typename KeyT>
struct is_in_tuple {
  static constexpr bool value =
      is_tuple<TupleT>::value &&
      tuple_search<std::is_same, TupleT, KeyT,
                   std::tuple_size<TupleT>::value - 1>::value;
};

// Check if a type TupleT is both a tuple and contains all of the types KeyT and
// OtherKeysT...
template <typename TupleT, typename KeyT, typename... OtherKeysT>
struct are_all_types_in_tuple {
  static constexpr bool value =
      is_in_tuple<TupleT, KeyT>::value &&
      are_all_types_in_tuple<TupleT, OtherKeysT...>::value;
};

template <typename TupleT, typename KeyT>
struct are_all_types_in_tuple<TupleT, KeyT> {
  static constexpr bool value = is_in_tuple<TupleT, KeyT>::value;
};

// Checks to see if one tuple is a subset of the other. The base case compares a
// Superset tuple against any non-tuple type which is false.
template <typename SupersetTupleT, typename SubsetTupleT>
struct is_tuple_a_subset : std::false_type {};
// When comparing one tuple type to another actually check is all the types in
// the subset are in the superset
template <typename SupersetTupleT, typename... SubsetTs>
struct is_tuple_a_subset<SupersetTupleT, std::tuple<SubsetTs...>>
    : public are_all_types_in_tuple<SupersetTupleT, SubsetTs...> {};
// The empty tuple is a subset of all tuples
template <typename SupersetTupleT>
struct is_tuple_a_subset<SupersetTupleT, std::tuple<>> : public std::true_type {
};

// Checks if all of the types in the set of types given are all unique
template <typename T, typename... OtherTs>
struct are_types_unique_impl {
  static constexpr bool value =
      (!traits::is_in_tuple<std::tuple<OtherTs...>, T>::value) &&
      are_types_unique_impl<OtherTs...>::value;
};

// Specialization, a single type is considered unique
template <typename T>
struct are_types_unique_impl<T> : public std::true_type {};

template <typename... Ts>
struct are_types_unique : public are_types_unique_impl<Ts...> {};

template <>
struct are_types_unique<> : public std::true_type {};

template <typename... Ts>
constexpr bool are_types_unique_v = are_types_unique<Ts...>::value;

// Sums the GetterTrait::values for a range of types in a std::tuple, going from
// from index I to IEnd (exclusive)
template <template <typename> class GetterTrait, typename TupleT, size_t I,
          size_t IEnd>
struct sum_tuple_range_values {
  static constexpr size_t value =
      GetterTrait<typename std::tuple_element<I, TupleT>::type>::value +
      sum_tuple_range_values<GetterTrait, TupleT, I + 1, IEnd>::value;
};

// Specialization of sum_tuple_range_annotation_size for the index past the end
template <template <typename> class GetterTrait, typename TupleT, size_t I>
struct sum_tuple_range_values<GetterTrait, TupleT, I, I> {
  static constexpr size_t value = 0;
};

// Implementation details of get_tuple_index, returns the current index if this
// index's type matches the KeyT otherwise searches the next element
template <typename TupleT, typename KeyT, size_t I>
struct get_tuple_index_impl {
  static_assert(I < std::tuple_size<TupleT>::value,
                "Could not find type in tuple");
  using check_type = typename std::conditional<
      std::is_same<typename std::tuple_element<I, TupleT>::type, KeyT>::value,
      std::integral_constant<size_t, I>,
      get_tuple_index_impl<TupleT, KeyT, I + 1>>::type;
  static constexpr size_t value = check_type::value;
};

// Gets the first index of a type KeyT that is a part of the tuple type TupleT
template <typename TupleT, typename KeyT>
struct get_tuple_index {
  static constexpr size_t value = get_tuple_index_impl<TupleT, KeyT, 0>::value;
};

// Get the size of a parameter pack.
template <typename... Ts>
constexpr size_t pack_size_v = std::tuple_size<std::tuple<Ts...>>::value;

// Get the index of a type in a parameter pack.
template <typename T, typename... Ts>
constexpr size_t pack_index_v =
    traits::get_tuple_index<std::tuple<Ts...>, T>::value;

// Check if a type is in a parameter pack.
template <typename T, typename... Ts>
constexpr bool is_in_pack_v = traits::is_in_tuple<std::tuple<Ts...>, T>::value;

// Check if a type has defined for it the four relational operators.
template <typename T, class = bool, class = bool, class = bool, class = bool>
struct has_relational_operators : std::false_type {};
template <typename T>
struct has_relational_operators<
    T, decltype(std::declval<T>() < std::declval<T>()),
    decltype(std::declval<T>() > std::declval<T>()),
    decltype(std::declval<T>() <= std::declval<T>()),
    decltype(std::declval<T>() >= std::declval<T>())> : std::true_type {};
template <typename T>
constexpr bool has_relational_operators_v = has_relational_operators<T>::value;

// Implementation needed to do for_each on all std::tuple elements, see
// https://www.fluentcpp.com/2019/03/08/stl-algorithms-on-tuples/
template <class Tuple, class F, std::size_t... I>
constexpr void for_each_impl(Tuple &&t, F &&f, std::index_sequence<I...>) {
  (void)std::initializer_list<int>{
      (std::forward<F>(f)(std::get<I>(std::forward<Tuple>(t))), 0)...};
}

// Call the callable f on each element of the tuple
template <class Tuple, class F>
constexpr void for_each(Tuple &&t, F &&f) {
  for_each_impl(std::forward<Tuple>(t), std::forward<F>(f),
                std::make_index_sequence<
                    std::tuple_size<std::remove_reference_t<Tuple>>::value>{});
}

}  // namespace traits
}  // namespace sensorfusion

#endif  // SENSORFUSION_CORE_TRAITS_H
