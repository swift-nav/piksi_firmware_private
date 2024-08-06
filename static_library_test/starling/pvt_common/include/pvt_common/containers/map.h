/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE'
 * which must be distributed together with this source. All other
 * rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF
 * ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_PVT_ENGINE_CONTAINERS_MAP_H
#define LIBSWIFTNAV_PVT_ENGINE_CONTAINERS_MAP_H

#include <pvt_common/optional.h>
#include <swiftnav/common.h>
#include <swiftnav/macros.h>
#include "set.h"
#include "shared.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <tuple>

namespace pvt_common {

namespace containers {

// Doxygen seems to encounter issues with a lot of the return types on
// member function declarations in this file, so we suppress it.

/// @cond CAN_DOXYGEN_CORRECTLY_PARSE_TEMPLATES_YET

template <typename T>
constexpr T const &max(T const &a, T const &b) {
  return a > b ? a : b;
}

template <typename K, typename V>
optional<V> values_equal(const K &key SWIFT_ATTR_UNUSED, const V &left,
                         const V &right) {
  if (left == right) {
    return left;
  }
  return {};
}

template <typename K, typename V, typename C, s32 max_map_size>
struct FilteredMapView;

// This class is a custom pair type which contains corresponding
// elements of type K and type V on the stack; in particular,
// comparison operators are all defined on the `key` member.  It is
// used in many places in the interface of the `Map` class, including
// in higher-order functions and in the iterators.
template <typename K, typename V>
struct MapElement {
  K key;
  V value;

  MapElement(const K &key_in, const V &value_in)
      : key(key_in), value(value_in) {}
  MapElement() : key(), value() {}

  bool operator<(const MapElement &other) const { return key < other.key; }

  bool operator>(const MapElement &other) const { return key > other.key; }

  bool operator<=(const MapElement &other) const { return key <= other.key; }

  bool operator>=(const MapElement &other) const { return key >= other.key; }

  bool operator==(const MapElement &other) const { return key == other.key; }

  bool operator!=(const MapElement &other) const { return !(*this == other); }

  bool kv_equals(const MapElement &other) const {
    return key == other.key && value == other.value;
  }
};

// This class lives on the stack and keeps an array of key-value
// mappings from type K to type V; it can only store one of each
// unique key.  It is a container, like `std::map`, but it has a more
// mathematical interface and encourages doing things inline instead
// of using iterators and other STL features, many of which require a
// heap.
//
//                          REQUIREMENTS
//
// To put values of type K in this container, K must be equipped with
//
//     bool operator<(const K &other)
//
//   and
//
//     bool operator==(const K &other)
//
// `operator<()` must induce a self-consistent total ordering on
// elements of K.  A map contains unique keys, so if you have two
// values of type V corresponding to key K, this container will only
// hold one of them at a time.  See the documentation for
// constructors, `extend()`, `add()` etc. if you need to understand
// how the container will behave with respect to such data.
//
//                      NAMING CONVENTIONS
//
// Member functions whose names are participles (`intersected()`,
// `filtered()`, `grown()`) return a new map object, with the
// operation applied to the original (`this`) map.  Members whose
// names are imperative verbs (`intersect()`, `filter()`, `add()`)
// modify the object they're called on and return a reference to
// enable method chaining.
//
//                    FUNCTIONS AS ARGUMENTS
//
// To make it easier to use higher-order functions without a heap,
// many member functions have overloads that are templatized over a
// helper value of type `C` (e.g. `intersect_on()`, `transformed()`).
// When you use these functions, especially if you pass them a lambda
// function, you may need to explicitly specify the type C as a
// template argument to the function.  In the below example, the
// helper value `threshold` is of type `s32`, so we explicitly write
// `filter<s32>()`.
//
//     s32 cutoff_value = 22;
//
//     const auto threshold_selector =
//         [](const MapElement<gnss_signal_t, s32> &el, s32 threshold) {
//       return el.value > threshold;
//     };
//
//     my_map.filter<s32>(threshold_selector, cutoff_value);
//
// The same caveat applies to a few other situations:
//
//   * the type-changing `transformed()` member functions return a
//     `Map` with the same key type `K` but a new value type `U` by
//     applying a user-provided function `transform_f` to each value
//     of the current `Map`.  Especially if you are giving this
//     function a lambda, you may need to explicitly specify the new
//     value type `U` as a template argument.
//
//   * a few flavors of `intersect_on()` accept as arguments map
//     objects with different value types than the map on which the
//     function is being called.  You may need to explicitly specify
//     not only the argument map's value types but also their maximum
//     sizes.
//
// If you are passing a lambda to a member function of a map, you may
// need to explicitly specify the return type of the lambda.  This can
// happen with functions like `intersect_on()`, which expects a
// function returning `optional<V>`.  If your lambda always returns a
// `V` (i.e. can never return an empty object `{}`), the compiler will
// not come up with the right return type, and the errors may not be
// helpful.  For example, instead of
//
//     my_map.intersect_on(
//         [](const key_t &key SWIFT_ATTR_UNUSED, const value_t &a,
//            const value_t &b) { return a + b; },
//         your_map);
//
// you may need to write
//
//     my_map.intersect_on(
//         [](const key_t &key SWIFT_ATTR_UNUSED, const value_t &a,
//            const value_t &b) -> optional<value_t> {
//           return a + b;
//         },
//         your_map);
//
// Sorry about this.
//
//                  PERFORMANCE OF CHAINED FUNCTIONS
//
// Many member functions that return a new map object,
// e.g. `intersected()` or `inserted()`, have multiple overloads: one
// where `*this` is a `const Map &` and one where `*this` is a `Map
// &&`.  The additional overloads are purely for performance reasons;
// they will kick in automatically where possible, and you can
// completely ignore them if you just want to use the container.  If
// you're worried about performance, the best thing you can do is
// chain several calls together in one rvalue, e.g.
//
//    const auto a = b
//        .intersected_on(containers::values_equal, c)
//        .filtered(is_a_foo)
//        .intersected(d);
//
// This should result in only one copy of `b`'s data being made and
// all three operations (intersect, filter, intersect with user merge
// function) being performed in-place on that copy.
//
// There is some more detailed discussion for implementors in the
// comments accompanying the out-of-line implementations of these
// overloads.
template <typename K, typename V, s32 max_map_size>
class Map {
  static_assert(std::is_trivially_destructible<MapElement<K, V>>::value,
                "Map requires a trivially destructible template class for "
                "its elements.");

 public:
  using Element = MapElement<K, V>;
  using ConstElement = MapElement<const K, V>;
  // Construct an empty map.
  Map() : elems_() {}

  // Construct a map from two arrays `keys` and `values`, each
  // containing at least `n` elements; the map will contain pairings
  // of elements at identical positions in `keys` and `values`.  The
  // first occurrence of any pairings for which the elements in `keys`
  // are equal according to `operator==()` will be preserved.  `n`
  // must be no greater than the compile-time maximum map size
  // `max_map_size` of this container.
  Map(s32 n, const K keys[], const V values[]);

  // Construct a map consisting of a single pairing `key : value`.
  Map(const K &key, const V &value);

  // Construct a map using an initializer list.
  Map(std::initializer_list<Element> initlist);

  // Construct a map from beginning and ending key iterators and a
  // value iterator.  The resulting map will contain the number of
  // keys spanned by the key iterators.  It is a programming error if
  // the value iterator can't be incremented and dereferenced that
  // many times.
  template <typename KIter, typename VIter>
  Map(KIter kbegin, KIter kend, VIter vbegin);

  // Construct a map from beginning and ending element iterators.  The
  // resulting map will contain the number of keys spanned by the
  // iterators.
  template <typename EIter>
  Map(EIter ebegin, EIter eend);

  // Construct a map from two arrays `keys` and `values`, each
  // containing `M` elements; the map will contain pairings of
  // elements at identical positions in `keys` and `values`.  The
  // first occurrence of any pairings for which the elements in `keys`
  // are equal according to `operator==()` will be preserved.  `M`
  // must be no greater than the compile-time maximum map size
  // `max_map_size` of this container.
  template <std::size_t M>
  Map(const std::array<K, M> &keys, const std::array<V, M> &values);

  // Construct a map from an array of `MapElement` key-value pairings
  // containing `M` elements.  The elements may occur in any order.
  // The first occurrence of any pairings for which the elements in
  // `keys` are equal according to `operator==()` will be preserved.
  // `M` must be no greater than the compile-time maximum map size
  // `max_map_size` of this container.
  template <std::size_t M>
  Map(s32 n, const std::array<Element, M> &elems);

  // Construct a map from a vector `v` of `MapElement` key-value
  // pairings.  The elements of `v` may occur in any order.  The first
  // occurrence of any pairings for which the elements in `keys` are
  // equal according to `operator==()` will be preserved.  `v` must
  // contain no more elements than the compile-time maximum map size
  // `max_map_size` of this container.
  explicit Map(const std::vector<Element> &v);

  Map(const Map &other) : n_(other.n_), elems_() {
    for (s32 i = 0; i < n_; ++i) {
      new (get_ptr(i)) Element(other.at(i));
    }
  }

  Map(Map &&other) noexcept : n_(other.n_), elems_() {
    for (s32 i = 0; i < n_; ++i) {
      new (get_ptr(i)) Element(std::move(other.at(i)));
    }
  }

  Map &operator=(const Map &other) {
    n_ = other.n_;
    std::copy(other.get_ptr(0), other.get_ptr(n_), get_ptr(0));
    return *this;
  }

  Map &operator=(Map &&other) noexcept {
    n_ = other.n_;
    std::copy(other.get_ptr(0), other.get_ptr(n_), get_ptr(0));
    return *this;
  }

  // Compare `this` map with an`other` for equality of both keys and
  // values.
  template <s32 other_max_map_size>
  bool operator==(const Map<K, V, other_max_map_size> &other) const;

  // Compare `this` map with an`other` for inequality of both keys and
  // values.
  bool operator!=(const Map &other) const { return !(*this == other); }

  // Return the number of key-value pairings stored in this map.
  s32 size() const { return n_; }

  // Return the maximum number of elements `this` map can contain.
  constexpr s32 max_size() const { return max_map_size; }

  // Test whether `this` map's keys are equal to an`other` map's keys.
  bool keys_equal(const Map &other) const {
    return differenced(other) == other.differenced(*this);
  }

  // Test whether `this` map contains a value corresponding to the
  // given `key`.
  bool contains(const K &key) const { return lookup(key).has_value(); }

  // Test whether `value` is associated with `key` in `this` map.  If
  // `this` map contains nothing associated with `key` or a different
  // value is associated with `key`, returns false.
  bool contains_key_value_pair(const K &key, const V &value) const {
    return lookup(key) == value;
  }

  // Return the set of keys for which `this` map contains associated
  // values.
  Set<K, max_map_size> keys() const;

  using EigenArrayOfValues =
      Eigen::Array<V, Eigen::Dynamic, 1, 0, max_map_size, 1>;

  // Return the set of values contained in `this` map.  The values are
  // ordered according to their corresponding keys.
  EigenArrayOfValues values() const;

  // Compute the intersection of `this` map with the `other` by
  // comparing keys.  On return, `this` set will contain the resulting
  // subset: the keys for which both `this` map and the `other` map
  // contained associated values paired with the values from `this`
  // map.
  Map &intersect(const Map &other);

  // This is like `intersect(const Map &other)`, but on return, `this`
  // map will contain the intersection of `a` and `b` (in that order).
  Map &intersect(const Map &a, const Map &b);

  // Compute the intersection of `this` map with the `other` by
  // comparing keys, using the given merge function `merge_f()` used
  // to resolve conflicting values.  On return, `this` set will
  // contain the resulting subset: the keys for which both `this` map
  // and the `other` map contained associated values, paired with the
  // non-empty values resulting from calling `merge_f()` on the
  // resulting intersection.  If `merge_f()` returns an empty object
  // on an input, the resulting map will contain no value associated
  // with that key.
  //
  // This function can be used, for example, to implement set
  // intersection on both keys and values by returning `this_value` if
  // it is equal to `other_value` and an empty object otherwise:
  //
  //     const auto ambs_equal = [](const gnss_signal_t &sid,
  //                                const s32 &this_amb,
  //                                const s32 &other_amb) {
  //       if (this_amb == other_amb) {
  //         return this_amb;
  //       }
  //       return {};
  //     };
  //
  //     my_map.intersect_on(ambs_equal, other_map);
  //
  // For another example, this function can also be used to do
  // aggregation, e.g. in the case of maps containing counters from
  // consecutive timesteps:
  //
  //     const auto sum_values = [](const gnss_signal_t &sid,
  //                                const u32 &this_counter,
  //                                const u32 &old_counter) {
  //       return this_counter + old_counter;
  //     };
  //
  //     my_map.intersect_on(sum_values, other_map);
  //
  // Please check out FUNCTIONS_AS_ARGUMENTS in the top-level
  // documentation for this class if you are getting compiler errors
  // trying to use this function.
  Map &intersect_on(optional<V> (*merge_f)(const K &key, const V &this_value,
                                           const V &other_value),
                    const Map &other);

  // This is like `intersect_on(merge_f, other)`, but on return,
  // `this` map will contain the intersection of `a` and `b` (in that
  // order) in place of `this` and `other`.
  Map &intersect_on(optional<V> (*merge_f)(const K &key, const V &a_value,
                                           const V &b_value),
                    const Map &a, const Map &b);

  // This is like `intersect_on(merge_f, other)`, but it allows an
  // arbitrary helper value `helper` to be passed to the merge
  // function `merge_f()` on each call.  `merge_f()` is guaranteed to
  // be called only once per overlapping key.
  template <typename C>
  Map &intersect_on(optional<V> (*merge_f)(const K &key, const V &this_value,
                                           const V &other_value, C helper),
                    const Map &other, C helper);

  // This is like `intersect_on(merge_f, a, b)`, but it allows an
  // arbitrary helper value to be passed to the merge function
  // `merge_f()` on each call.  `merge_f()` is guaranteed to be called
  // only once per key in the intersection.
  template <typename C>
  Map &intersect_on(optional<V> (*merge_f)(const K &key, const V &a_value,
                                           const V &b_value, C helper),
                    const Map &a, const Map &b, C helper);

  // This is like `intersect_on(merge_f, a, b)`, but `a` and `b` may
  // have different value types than and be smaller than `this` map.
  // `merge_f()` is guaranteed to be called only once per key in the
  // intersection.
  template <typename VA, typename VB, s32 size1, s32 size2>
  Map &intersect_on(optional<V> (*merge_f)(const K &key, const VA &a_value,
                                           const VB &b_value),
                    const Map<K, VA, size1> &a, const Map<K, VB, size2> &b);

  // This is like `intersect_on(merge_f, a, b, helper)`, but `a` and
  // `b` may have different value types than and be smaller than
  // `this` map.  `merge_f()` is guaranteed to be called only once per
  // key in the intersection.
  template <typename C, typename VA, typename VB, s32 size1, s32 size2>
  Map &intersect_on(optional<V> (*merge_f)(const K &key, const VA &a_value,
                                           const VB &b_value, C helper),
                    const Map<K, VA, size1> &a, const Map<K, VB, size2> &b,
                    C helper);

  // This is like `intersect_on(merge_f, other)`, but the `other` map
  // may have a different value type than and be smaller than `this`
  // map.  `merge_f()` is guaranteed to be called only once per key in
  // the intersection.
  template <typename VO, s32 other_max_size>
  Map &intersect_on(optional<V> (*merge_f)(const K &key, const V &this_value,
                                           const VO &other_value),
                    const Map<K, VO, other_max_size> &other);

  // This is like `intersect_on(merge_f, other, helper)`, but the
  // `other` map may have a different value type than and be smaller
  // than `this` map.  `merge_f()` is guaranteed to be called only
  // once per key in the intersection.
  template <typename C, typename VO, s32 other_max_size>
  Map &intersect_on(optional<V> (*merge_f)(const K &key, const V &this_value,
                                           const VO &other_value, C helper),
                    const Map<K, VO, other_max_size> &other, C helper);

  // This is like `intersect(other)`, but it returns a new `Map`
  // containing the intersection instead of modifying `this` one.
  Map intersected(const Map &other) const &;
  Map intersected(const Map &other) &&;

  // This is like `intersect_on(merge_f, other)`, but it returns a new
  // `Map` containing the intersection instead of modifying `this`
  // one.
  Map intersected_on(optional<V> (*merge_f)(const K &key, const V &a_value,
                                            const V &b_value),
                     const Map &other) const &;
  Map intersected_on(optional<V> (*merge_f)(const K &key, const V &a_value,
                                            const V &b_value),
                     const Map &other) &&;

  // This is like `intersect_on(merge_f, other, helper)`, but it
  // returns a new `Map` containing the intersection instead of
  // modifying `this` one.
  template <typename C>
  Map intersected_on(optional<V> (*merge_f)(const K &key, const V &a_value,
                                            const V &b_value, C helper),
                     const Map &other, C helper) const &;
  template <typename C>
  Map intersected_on(optional<V> (*merge_f)(const K &key, const V &a_value,
                                            const V &b_value, C helper),
                     const Map &other, C helper) &&;

  // This is like `intersected_on(merge_f, other)`, but `other` may
  // have a different value type than and be smaller than `this` map.
  template <typename VO, s32 other_max_size>
  Map intersected_on(optional<V> (*merge_f)(const K &key, const V &this_value,
                                            const VO &other_value),
                     const Map<K, VO, other_max_size> &other) const &;
  template <typename VO, s32 other_max_size>
  Map intersected_on(optional<V> (*merge_f)(const K &key, const V &this_value,
                                            const VO &other_value),
                     const Map<K, VO, other_max_size> &other) &&;

  // This is like `intersected_on(merge_f, other, helper)`, but
  // `other` may have a different value type than and be smaller than
  // `this` map.
  template <typename C, typename VO, s32 other_max_size>
  Map intersected_on(optional<V> (*merge_f)(const K &key, const V &this_value,
                                            const VO &other_value, C helper),
                     const Map<K, VO, other_max_size> &other, C helper) const &;
  template <typename C, typename VO, s32 other_max_size>
  Map intersected_on(optional<V> (*merge_f)(const K &key, const V &this_value,
                                            const VO &other_value, C helper),
                     const Map<K, VO, other_max_size> &other, C helper) &&;

  // Compute the union ("extension" since "union" is a keyword in C++)
  // of `this` map with the `other` by comparing keys.  On return,
  // `this` set will contain the resulting superset: the keys for
  // which either `this` map and the `other` map contained associated
  // values, with the values from `this` map for any overlapping keys.
  // If the union of the two maps would contain more than
  // `max_map_size` elements, then an assertion is triggered;
  // otherwise, a reference to `this` object is returned.
  Map &extend(const Map &other);

  // This is like `extend(const Map &other)`, but on return, `this`
  // map will contain the union of `a` and `b` (in that order) in
  // place of `this` and `other`.
  Map &extend(const Map &a, const Map &b);

  // Compute the union ("extension") of `this` map with the `other` by
  // comparing keys, using the given merge function `merge_f()` used
  // to resolve conflicting values.  On return, `this` set will
  // contain the resulting subset: the keys for which either `this`
  // map or the `other` map contained associated values, paired with
  // the non-empty values resulting from calling `merge_f()` on all
  // overlapping key-value pairs.  If `merge_f()` returns an empty
  // object on an input, the resulting map will contain no value
  // associated with that key.  If the resulting subset would contain
  // more than `max_map_size` elements, then an assertion is
  // triggered.
  //
  // See `intersect_on(merge_f, other)` for some example applications
  // of custom merge functions in set operations.
  //
  // Please check out FUNCTIONS_AS_ARGUMENTS in the top-level
  // documentation for this class if you are getting compiler errors
  // trying to use this function.
  Map &extend_on(V (*merge_f)(const K &key, const V &this_value,
                              const V &other_value),
                 const Map &other);

  // This is like `extend_on(merge_f, other)`, but on return, `this`
  // map will contain the union of `a` and `b` (in that order) in
  // place of `this` and `other`.
  Map &extend_on(V (*merge_f)(const K &key, const V &a_value, const V &b_value),
                 const Map &a, const Map &b);

  // This is like `extend_on(merge_f, other)`, but it allows an
  // arbitrary helper value `helper` to be passed to the merge
  // function `merge_f()` on each call.  `merge_f()` is guaranteed to
  // be called only once per overlapping key.
  template <typename C>
  Map &extend_on(V (*merge_f)(const K &key, const V &this_value,
                              const V &other_value, C helper),
                 const Map &other, C helper);

  // This is like `extend_on(merge_f, a, b)`, but it allows an
  // arbitrary helper value `helper` to be passed to the merge
  // function `merge_f()` on each call.  `merge_f()` is guaranteed to
  // be called only once per overlapping key.
  template <typename C>
  Map &extend_on(V (*merge_f)(const K &key, const V &a_value, const V &b_value,
                              C helper),
                 const Map &a, const Map &b, C helper);

  // This is like `extend(other)`, but it returns a new `Map`
  // containing the union instead of modifying `this` one.
  Map extended(const Map &other) const &;

  // This is like `extend_on(merge_f, other)`, but it returns a new
  // `Map` containing the union instead of modifying `this` one.
  Map extended_on(V (*merge_f)(const K &key, const V &this_value,
                               const V &other_value),
                  const Map &other) const &;

  // This is like `extend_on(merge_f, other, helper)`, but it returns
  // a new `Map` containing the union instead of modifying `this` one.
  template <typename C>
  Map extended_on(V (*merge_f)(const K &key, const V &this_value,
                               const V &other_value, C helper),
                  const Map &other, C helper) const &;

  // Compute the difference of `this` map with the `other` by
  // comparing keys.  On return, `this` set will contain the resulting
  // subset: the key-value pairings in `this` map for which the
  // `other` contained no pairing.
  Map &difference(const Map &other);

  // This is like `difference(other)`, but on return, `this` map will
  // contain the difference of `a` and `b` (in that order) in place of
  // `this` and `other`: the key-value pairings in `a` for which `b`
  // contained no pairing.
  Map &difference(const Map &a, const Map &b);

  // This is like `difference(other)`, but it returns a new `Map`
  // containing the union instead of modifying `this` one.
  Map differenced(const Map &other) const &;
  Map differenced(const Map &other) &&;

  // Return a new map containing the same key-value pairings as `this`
  // one but with a new maximum size specified in the template
  // argument `new_max_map_size`.  `new_max_map_size` may not be
  // smaller than `this` container's `max_map_size`.
  template <s32 new_max_map_size>
  Map<K, V, new_max_map_size> grown() const;

  // Return a new map containing the same key-value pairings as `this`
  // one but with a new maximum size specified in the template
  // argument `new_max_map_size`. `new_max_map_size` must be larger than
  // the current size (`size()`).  If it is not, we assert.
  template <s32 new_max_map_size>
  Map<K, V, new_max_map_size> resized() const;

  // Add the mapping `key` -> `value` to `this` map.  If an existing
  // value is associated with `key`, it will be replaced.  If the map
  // doesn't have room for this new element, an assertion is
  // triggered; otherwise, a reference to `this` map is returned.
  Map &insert(const K &key, const V &value);

  // Add the mapping `key` -> `value` to `this` map.  If an existing
  // value is associated with `key`, an empty object is returned.  If
  // the map doesn't have room for this new element, an assertion is
  // triggered; otherwise, a reference to `this` map is returned.
  optional<Map &> insert_missing(const K &key, const V &value);

  // Return a new map equivalent to `this` map extended with the
  // mapping `key` -> `value`.  If the map doesn't have room for this
  // new element, then a copy of `this` map is returned.
  Map inserted(const K &key, const V &value) const &;
  Map inserted(const K &key, const V &value) &&;

  // Refine `this` map to contain only the mapping associated with the
  // given `key`.  If `this` map does not contain a mapping for `key`,
  // it will be empty on return.
  Map &refine(const K &key);

  // Refine `this` map to contain only the mappings for the keys in
  // `keep_keys`.  If `this` map does not contain a mapping for any
  // elements of `keep_keys`, it will be empty on return.
  template <s32 set_size>
  Map &refine(const Set<K, set_size> &keep_keys);

  // This is like `refine(key)`, but it returns a new `Map` instead of
  // modifying `this` one.
  Map refined(const K &key) const &;
  Map refined(const K &key) &&;

  // This is like `refine(keep_keys)`, but it returns a new `Map`
  // instead of modifying `this` one.
  template <s32 set_size>
  Map refined(const Set<K, set_size> &keys) const &;
  template <s32 set_size>
  Map refined(const Set<K, set_size> &keys) &&;

  // Remove any mapping associated with the given `key` from `this`
  // map.  If `this` map contains no such mapping, it will not be
  // modified.
  Map &drop(const K &key);

  // Remove any mappings associated with the given `keys` from `this`
  // map.  If `this` map contains no such mappings, it will not be
  // modified.
  template <s32 set_size>
  Map &drop(const Set<K, set_size> &keys);

  // This is like `drop(key)`, but it returns a new `Map` instead of
  // modifying `this` one.
  Map dropped(const K &key) const &;
  Map dropped(const K &key) &&;

  // This is like `drop(keys)`, but it returns a new `Map` instead of
  // modifying `this` one.
  template <s32 set_size>
  Map dropped(const Set<K, set_size> &keys) const &;
  template <s32 set_size>
  Map dropped(const Set<K, set_size> &keys) &&;

  // Find `key` in `this` map and return the offset into member
  // storage at which `key` is stored, or an empty object if the
  // member does not exist.
  optional<s32> find(const K &key) const;

  // Look up the value associated with the given `key` in `this` map.
  // If it is found, a reference to the value is returned; otherwise,
  // an empty object is returned.
  optional<const V &> lookup(const K &key) const &;

  // Look up the value associated with the given `key` in `this` map.
  // If it is found, a reference is returned to allow the contents of
  // the map to be modified; otherwise, an empty object is returned.
  optional<V &> lookup(const K &key) &;

  // Look up the value associated with the given `key` in `this` map.
  // If it is found, a copy of the value is returned; otherwise, an
  // empty object is returned.
  //
  // This overload operates only on rvalue maps; it copies because in
  // most cases, if you have called this function, the map will
  // shortly disappear.
  optional<V> lookup(const K &key) &&;

  // This operator is a synonym for `lookup(key)`.
  optional<const V &> operator[](const K &key) const &;

  // This operator is a synonym for `lookup(key)`.
  optional<V &> operator[](const K &key) &;

  // This operator is a synonym for `lookup(key)`.
  optional<V> operator[](const K &key) &&;

  // Given a selection function `select_f()`, remove from `this` map
  // all pairings for which `select_f()` returns false (in other
  // words, keep only elements for which `select_f()` returns `true`).
  // Returns a reference to `this` map.
  Map &filter(bool (*select_f)(const Element &elem));

  // This is like `filter(select_f)`, but it allows an arbitrary
  // helper value `helper` to be passed to the selection predicate
  // `select_f()` on each call.  `select_f()` is guaranteed to be
  // called only once per element in the map.
  //
  // Please check out FUNCTIONS_AS_ARGUMENTS in the top-level
  // documentation for this class if you are getting compiler errors
  // trying to use this function.
  template <typename C>
  Map &filter(bool (*select_f)(const Element &elem,
                               const std::decay_t<C> &helper),
              C &&helper);

  // This is like `filter(select_f)`, but it returns a new, reduced
  // `Map` instead of modifying `this` one.
  Map filtered(bool (*select_f)(const Element &elem)) const &;
  Map filtered(bool (*select_f)(const Element &elem)) &&;

  // This is like `filter(select_f, helper)`, but it returns a new,
  // reduced `Map` instead of modifying `this` one.
  template <typename C>
  Map filtered(bool (*select_f)(const Element &elem,
                                const std::decay_t<C> &helper),
               C &&helper) const &;
  template <typename C>
  Map filtered(bool (*select_f)(const Element &elem,
                                const std::decay_t<C> &helper),
               C &&helper) &&;

  // Some caution must be used here - when used in a range-based for loop (the
  // usual case), this can't be used recursively. A FilteredMapView cannot be
  // used on top of another FilteredMapView and because it keeps a reference to
  // the input map as a const & (in order to avoid the CPU load of a hard copy),
  // if the input map is a temporary the behavior is undefined (see
  // http://en.cppreference.com/w/cpp/language/range-for for details).
  template <typename C>
  FilteredMapView<K, V, C, max_map_size> filtered_view(
      bool (*select_f)(const Element &elem, const C &helper),
      const C &helper) const {
    return FilteredMapView<K, V, C, max_map_size>(select_f, helper, *this);
  }

  // NOTE ABOUT `transform` AND RELATED FUNCTIONS
  //
  // The STL provides `std::transform`, but `std::transform` doesn't
  // allow for passing in helper arguments, which means you often end
  // up using it with lambdas that capture, which are in turn a bummer
  // for us because we use function pointers instead of
  // `std::function` to avoid the heap.  In addition, `std::transform`
  // is built on top of ordinary (mutable) iterators, but we only ever
  // return const iterators so that we don't have to re-sort
  // constantly to maintain our ordering invariant.  The `transform`
  // family of member functions avoids these problems.

  // Given an element-wise transformation function `transform_f`,
  // transform all the values in `this` map.  A reference to `this`
  // map is returned.
  Map &transform(V (*transform_f)(const Element &elem));

  // This is like `transform(transform_f)`, but it allows an arbitrary
  // helper value `helper` to be passed to the transformation function
  // `transform_f()` on each call.  `transform_f()` is guaranteed to
  // be called only once per element in the map.
  //
  // Please check out FUNCTIONS_AS_ARGUMENTS in the top-level
  // documentation for this class if you are getting compiler errors
  // trying to use this function.
  template <typename C>
  Map &transform(V (*transform_f)(const Element &elem,
                                  const std::decay_t<C> &helper),
                 C &&helper);

  // This is like `transform(transform_f)`, but it returns a new `Map`
  // containing the updated values instead of modifying `this` one.
  Map transformed(V (*transform_f)(const Element &elem)) const &;
  Map transformed(V (*transform_f)(const Element &elem)) &&;

  // This is like `transform(transform_f, helper)`, but it returns a
  // new `Map` containing the updated values instead of modifying
  // `this` one.
  template <typename C>
  Map transformed(V (*transform_f)(const Element &elem,
                                   const std::decay_t<C> &helper),
                  C &&helper) const &;
  template <typename C>
  Map transformed(V (*transform_f)(const Element &elem,
                                   const std::decay_t<C> &helper),
                  C &&helper) &&;

  // Given an element-wise transformation function `transform_f` which
  // returns values of type `U`, compute a new map containing the
  // transformed values from `this` one.  Each call to `transform_f`
  // computes a new value of type `U` which corresponds to the key in
  // the `MapElement` it is given as its argument; in other words,
  // this function only transforms the _values_ in the map and leaves
  // the keys unmodified.
  template <typename U>
  Map<K, U, max_map_size> transformed(
      U (*transform_f)(const Element &elem)) const;

  // This is like `transformed<U>(transform_f)`, but it allows an arbitrary
  // helper value `helper` to be passed to the transformation function
  // `transform_f()` on each call.  `transform_f()` is guaranteed to
  // be called only once per element in the map.
  template <typename U, typename C>
  Map<K, U, max_map_size> transformed(
      U (*transform_f)(const Element &elem, const std::decay_t<C> &helper),
      C &&helper) const;

  // This is like `transformed<U>(transform_f)`, but the
  // transformation function `transform_f()` may return an empty
  // object when given values for which there is no appropriate
  // transformation (i.e. if `U` -> `V` is not total).  When
  // `transform_f(MapElement(key, value))` returns an empty object,
  // the resulting map will not have any value associated with the
  // `key` it was called with.
  template <typename U>
  Map<K, U, max_map_size> transformed(
      optional<U> (*transform_f)(const Element &elem)) const;

  // This is like `transformed<U>(transform_f)`, whose transformation
  // function may return empty objects, but it allows an arbitrary
  // helper value `helper` to be passed to the transformation function
  // `transform_f()` on each call.  `transform_f()` is guaranteed to
  // be called only once per element in the map.
  template <typename U, typename C>
  Map<K, U, max_map_size> transformed(
      optional<U> (*transform_f)(const Element &elem,
                                 const std::decay_t<C> &helper),
      C &&helper) const;

  // Given an element-wise operation `traverse_f` which generates a
  // new `accumulator` value of type C when applied to each element
  // and the current `accumulator` value along with a pointer to an
  // initial accumulator value, traverse the members of the map,
  // visiting each once, and generate a final accumulated (or
  // "summary") value.  The summary value is returned.
  //
  // No guarantees are provided about the order in which the elements
  // of the set are visited.  Note that the accumulator can be passed
  // by value.
  //
  // Please check out FUNCTIONS_AS_ARGUMENTS in the top-level
  // documentation for this class if you are getting compiler errors
  // trying to use this function.
  template <typename C>
  C traverse(C (*traverse_f)(const Element &elem, const C &accumulator),
             const C &accumulator) const;

  // Given an element-wise selection function `select_f`, which
  // returns a boolean given a map element, divide `this` map into a
  // `std::tuple` of new maps.  The first (left-hand) map returned
  // contains all the elements of `this` for which `select_f` returns
  // `true`; the second (right-hand) map returned contains all the
  // remaining elements.
  std::tuple<Map, Map> partitioned(bool (*select_f)(const Element &elem)) const;

  // This is like `partitioned(select_f)`, but it allows an arbitrary
  // helper value `helper` to be passed to the selection predicate
  // `select_f()` on each call.  `select_f()` is guaranteed to be
  // called only once per element in the map.
  //
  // Please check out FUNCTIONS_AS_ARGUMENTS in the top-level
  // documentation for this class if you are getting compiler errors
  // trying to use this function.
  template <typename C>
  std::tuple<Map, Map> partitioned(
      bool (*select_f)(const Element &elem, const std::decay_t<C> &helper),
      C &&helper) const;

  // Return a const iterator pointing at the first of `this` map's
  // members.
  const Element *cbegin() const { return get_ptr(0); }

  // Return a const iterator pointing past the last of `this` map's
  // members.
  const Element *cend() const { return get_ptr(n_); }

  // Return a const iterator pointing at the first of `this` map's
  // members.
  const Element *begin() const { return cbegin(); }

  // Return an iterator pointing at the first of `this` map's members.
  ConstElement *begin() {
    // `reinterpret_cast` is used here and the warning disabled
    // because it's quite safe; the only casting going on is that
    // we're adding a `const` to the `key` field of the `MapElement`
    // so that via this iterator, the user cannot modify the key but
    // can modify the value.
    return reinterpret_cast<ConstElement *>(get_ptr(0));  // NOLINT
  }

  // Return a const iterator pointing past the last of `this` map's
  // members.
  const Element *end() const { return cend(); }

  // Return an iterator pointing past the last of `this` map's
  // members.
  ConstElement *end() {
    // See `begin()` for a note about the use of `reinterpret_cast` here.
    return reinterpret_cast<ConstElement *>(  // NOLINT
        get_ptr(n_));
  }

  // Return a const reverse_iterator pointing at the last of `this` map's
  // members.
  const std::reverse_iterator<const Element *> crbegin() const {
    return std::make_reverse_iterator(cend());
  }

  // Return a const reverse_iterator pointing past the first of `this` map's
  // members.
  const std::reverse_iterator<const Element *> crend() const {
    return std::make_reverse_iterator(cbegin());
  }

  // Return a const reverse_iterator pointing at the last of `this` map's
  // members.
  std::reverse_iterator<const Element *> rbegin() const {
    return std::make_reverse_iterator(cend());
  }

  // Return an reverse_iterator pointing at the last of `this` map's members.
  std::reverse_iterator<ConstElement *> rbegin() {
    return std::make_reverse_iterator(end());
  }

  // Return a const reverse_iterator pointing past the first of `this` map's
  // members.
  std::reverse_iterator<const Element *> rend() const {
    return std::make_reverse_iterator(cbegin());
  }

  // Return an reverse_iterator pointing past the first of `this` map's
  // members.
  std::reverse_iterator<ConstElement *> rend() {
    return std::make_reverse_iterator(begin());
  }

  // Return an Eigen array containing the values in `this` map
  // corresponding to the given `keys`.  Any keys in `keys` for which
  // no mapping is available will be ignored (but see also
  // `strict_index()`).  The values will appear in the same order as
  // their corresponding keys.
  template <s32 set_size>
  EigenArrayOfValues index(const Set<K, set_size> &keys) const {
    EigenArrayOfValues ret(max_map_size);
    s32 ret_index = 0;
    s32 elem_index = 0;
    for (auto *kptr = keys.begin(); kptr != keys.end() && elem_index < n_;) {
      const K &key = *kptr;
      const Element &elem = at(elem_index);
      if (elem.key < key) {
        elem_index++;
      } else if (key < elem.key) {
        kptr++;
      } else {
        ret[ret_index++] = elem.value;
        elem_index++;
        kptr++;
      }
    }

    ret->conservativeResize(ret_index);
    return ret;
  }

  // This is an alias for `index(keys)`.
  template <s32 set_size>
  EigenArrayOfValues operator[](const Set<K, set_size> &keys) const {
    return index(keys);
  }

  // Return an Eigen array containing the values in `this` map
  // corresponding to the given `keys`, or an empty object if given
  // any `keys` for which `this` object has no corresponding
  // value. The values will appear in the same order as their
  // corresponding keys.
  template <s32 set_size>
  optional<EigenArrayOfValues> strict_index(
      const Set<K, set_size> &keys) const {
    if (keys.differenced(keys()).size() != 0) {
      return {};
    }

    return index(keys);
  }

 private:
  template <typename R>
  static R call_reroute_(const Element &a, R (*element_f)(const Element &));

  template <typename R>
  static R call_reroute_ref_(const Element &a,
                             R (*const &element_f)(const Element &));

  template <typename R, typename VA, typename VB>
  static R merge_reroute_(const K &key, const VA &left_value,
                          const VB &right_value,
                          R (*merge_f)(const K &, const VA &, const VB &));

  // It's unclear why we need the unused markers here, but
  // `clang-tidy-3.8` seems to complain.
  static optional<V> choose_left_optional(
      const K &key SWIFT_ATTR_UNUSED, const V &left_value,
      const V &right_value SWIFT_ATTR_UNUSED);

  static V choose_left(const K &key SWIFT_ATTR_UNUSED, const V &left_value,
                       const V &right_value SWIFT_ATTR_UNUSED);

  template <typename C>
  optional<Map &> extend_on_impl(V (*merge_f)(const K &key, const V &left_value,
                                              const V &right_value, C helper),
                                 const Map &a, const Map &b, C helper);

  Map &drop_indices_(std::size_t drop_count,
                     std::array<std::size_t, max_map_size + 1> *drop_indices);

  template <typename FT>
  optional<Map &> insert_impl_(const K &key, const V &value, FT handle_present);

  // This function sorts the contents of this map.
  void sort_();

  void assert_unique() const;
  void assert_sorted() const;

  Element &at(s32 idx) & {
    return elems_[idx].value;  // NOLINT
  };

  const Element &at(s32 idx) const & { return elems_[idx].value; };  // NOLINT

  Element *get_ptr(s32 idx) & {
    assert(idx >= 0);
    idx = (idx < 0 ? 0 : idx);
    if (idx >= n_ && n_ > 0) {
      return &at(n_ - 1) + 1;
    }
    if (n_ == 0) {
      return &at(0);
    }
    return &at(idx);
  }

  const Element *get_ptr(s32 idx) const & {
    assert(idx >= 0);
    idx = (idx < 0 ? 0 : idx);
    if (idx >= n_ && n_ > 0) {
      return &at(n_ - 1) + 1;
    }
    if (n_ == 0) {
      return &at(0);
    }
    return &at(idx);
  }

  union Storage {
    Storage() : dummy(){};
    uint8_t dummy;
    Element value;
  };

  s32 n_{0};

  Storage elems_[max_map_size];
};

template <typename K, typename V, typename C, s32 max_map_size>
struct Map_iter {
 public:
  using iterator_category = std::bidirectional_iterator_tag;
  using value_type = MapElement<K, V>;
  using difference_type = std::ptrdiff_t;
  using pointer = const MapElement<K, V> *;
  using reference = const MapElement<K, V> &;

  Map_iter(s32 start_index,
           bool (*select_f)(const MapElement<K, V> &elem, const C &helper),
           const C &helper, const Map<K, V, max_map_size> &map);
  const Map_iter &operator++();
  Map_iter operator++(int);
  const Map_iter &operator--();
  Map_iter operator--(int);
  const MapElement<K, V> &operator*() const;
  const MapElement<K, V> *operator->() const;
  bool operator==(const Map_iter &rhs) const;
  bool operator!=(const Map_iter &rhs) const;

 private:
  const MapElement<K, V> *elem_;
  bool (*select_f_)(const MapElement<K, V> &elem, const C &helper);
  const Map<K, V, max_map_size> &map_;
  const C &helper_;
};

template <typename K, typename V, typename C, s32 max_map_size>
struct FilteredMapView {
 public:
  FilteredMapView(bool (*select_f)(const MapElement<K, V> &elem,
                                   const C &helper),
                  const C &helper, const Map<K, V, max_map_size> &map);
  Map_iter<K, V, C, max_map_size> cbegin() const;
  Map_iter<K, V, C, max_map_size> cend() const;
  std::reverse_iterator<Map_iter<K, V, C, max_map_size>> crbegin() const;
  std::reverse_iterator<Map_iter<K, V, C, max_map_size>> crend() const;
  Map_iter<K, V, C, max_map_size> begin() const;
  Map_iter<K, V, C, max_map_size> end() const;
  std::reverse_iterator<Map_iter<K, V, C, max_map_size>> rbegin() const;
  std::reverse_iterator<Map_iter<K, V, C, max_map_size>> rend() const;

 private:
  bool (*select_f_)(const MapElement<K, V> &elem, const C &helper);
  const Map<K, V, max_map_size> &map_;
  C helper_;
};

template <typename K, typename V, typename C, s32 max_map_size>
Map_iter<K, V, C, max_map_size>::Map_iter(
    s32 start_index,
    bool (*select_f)(const MapElement<K, V> &elem, const C &helper),
    const C &helper, const Map<K, V, max_map_size> &map)
    : elem_(map.begin() + start_index),
      select_f_(select_f),
      map_(map),
      helper_(helper) {
  assert(start_index >= 0 && start_index <= map.size());
  if (elem_ != map_.cend() && !select_f_(*elem_, helper_)) {
    operator++();
  }
}

template <typename K, typename V, typename C, s32 max_map_size>
const Map_iter<K, V, C, max_map_size>
    &Map_iter<K, V, C, max_map_size>::operator++() {
  ++elem_;
  while (elem_ != map_.cend() && !select_f_(*elem_, helper_)) {
    ++elem_;
  }
  return *this;
}

template <typename K, typename V, typename C, s32 max_map_size>
Map_iter<K, V, C, max_map_size> Map_iter<K, V, C, max_map_size>::operator++(
    int) {
  auto temp = *this;
  operator++();
  return temp;
}

template <typename K, typename V, typename C, s32 max_map_size>
const Map_iter<K, V, C, max_map_size>
    &Map_iter<K, V, C, max_map_size>::operator--() {
  --elem_;
  while (elem_ != map_.cbegin() && !select_f_(*elem_, helper_)) {
    --elem_;
  }
  return *this;
}

template <typename K, typename V, typename C, s32 max_map_size>
Map_iter<K, V, C, max_map_size> Map_iter<K, V, C, max_map_size>::operator--(
    int) {
  auto temp = *this;
  operator--();
  return temp;
}

template <typename K, typename V, typename C, s32 max_map_size>
const MapElement<K, V> &Map_iter<K, V, C, max_map_size>::operator*() const {
  return *elem_;
}

template <typename K, typename V, typename C, s32 max_map_size>
const MapElement<K, V> *Map_iter<K, V, C, max_map_size>::operator->() const {
  return elem_;
}

template <typename K, typename V, typename C, s32 max_map_size>
bool Map_iter<K, V, C, max_map_size>::operator==(
    const Map_iter<K, V, C, max_map_size> &rhs) const {
  // Do we need to consider the other members?
  return elem_ == rhs.elem_;
}

template <typename K, typename V, typename C, s32 max_map_size>
bool Map_iter<K, V, C, max_map_size>::operator!=(
    const Map_iter<K, V, C, max_map_size> &rhs) const {
  return !(*this == rhs);
}

template <typename K, typename V, typename C, s32 max_map_size>
FilteredMapView<K, V, C, max_map_size>::FilteredMapView(
    bool (*select_f)(const MapElement<K, V> &elem, const C &helper),
    const C &helper, const Map<K, V, max_map_size> &map)
    : select_f_(select_f), map_(map), helper_(helper) {}

template <typename K, typename V, typename C, s32 max_map_size>
Map_iter<K, V, C, max_map_size> FilteredMapView<K, V, C, max_map_size>::cbegin()
    const {
  return begin();
}

template <typename K, typename V, typename C, s32 max_map_size>
Map_iter<K, V, C, max_map_size> FilteredMapView<K, V, C, max_map_size>::cend()
    const {
  return end();
}

template <typename K, typename V, typename C, s32 max_map_size>
std::reverse_iterator<Map_iter<K, V, C, max_map_size>>
FilteredMapView<K, V, C, max_map_size>::crbegin() const {
  return rbegin();
}

template <typename K, typename V, typename C, s32 max_map_size>
std::reverse_iterator<Map_iter<K, V, C, max_map_size>>
FilteredMapView<K, V, C, max_map_size>::crend() const {
  return rend();
}

template <typename K, typename V, typename C, s32 max_map_size>
Map_iter<K, V, C, max_map_size> FilteredMapView<K, V, C, max_map_size>::begin()
    const {
  return Map_iter<K, V, C, max_map_size>(0, select_f_, helper_, map_);
}

template <typename K, typename V, typename C, s32 max_map_size>
Map_iter<K, V, C, max_map_size> FilteredMapView<K, V, C, max_map_size>::end()
    const {
  return Map_iter<K, V, C, max_map_size>(map_.size(), select_f_, helper_, map_);
}

template <typename K, typename V, typename C, s32 max_map_size>
std::reverse_iterator<Map_iter<K, V, C, max_map_size>>
FilteredMapView<K, V, C, max_map_size>::rbegin() const {
  return std::make_reverse_iterator(end());
}

template <typename K, typename V, typename C, s32 max_map_size>
std::reverse_iterator<Map_iter<K, V, C, max_map_size>>
FilteredMapView<K, V, C, max_map_size>::rend() const {
  return std::make_reverse_iterator(begin());
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size>::Map(s32 n, const K keys[], const V values[])
    : Map(&keys[0], &keys[0] + n, &values[0]) {}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size>::Map(const K &key, const V &value)
    : Map(1, &key, &value) {}

template <typename K, typename V, s32 max_map_size>
template <std::size_t M>
Map<K, V, max_map_size>::Map(const std::array<K, M> &keys,
                             const std::array<V, M> &values)
    : Map(static_cast<s32>(M), keys.data(), values.data()) {
  static_assert(M <= max_map_size,
                "Map<K, V, max_map_size>::Map(std::array keys, std::array "
                "values): array size is greater than maximum map size.");
}

template <typename K, typename V, s32 max_map_size>
template <typename KIter, typename VIter>
Map<K, V, max_map_size>::Map(KIter kbegin, KIter kend, VIter vbegin)
    : n_(), elems_() {
  assert(std::distance(kbegin, kend) >= 0);

  for (; kbegin != kend; ++kbegin, ++vbegin) {
    auto itr = std::find_if(cbegin(), cend(), [&kbegin](const auto &item) {
      return item.key == *kbegin;
    });
    if (itr != cend()) {
      continue;
    }

    assert(n_ < max_map_size);
    new (get_ptr(n_++)) Element(*kbegin, *vbegin);
  }

  sort_();
}

template <typename K, typename V, s32 max_map_size>
template <typename EIter>
Map<K, V, max_map_size>::Map(EIter ebegin, EIter eend) : n_(), elems_() {
  assert(std::distance(ebegin, eend) >= 0);

  for (; ebegin != eend; ++ebegin) {
    auto itr = std::find(cbegin(), cend(), *ebegin);
    if (itr != cend()) {
      continue;
    }

    assert(n_ < max_map_size);
    new (get_ptr(n_++)) Element(*ebegin);
  }

  sort_();
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size>::Map(std::initializer_list<Element> initlist)
    : Map(initlist.begin(), initlist.end()) {}

template <typename K, typename V, s32 max_map_size>
template <std::size_t M>
Map<K, V, max_map_size>::Map(s32 n, const std::array<Element, M> &elems)
    : Map(elems.cbegin(), elems.cbegin() + n) {}

template <typename K, typename V, s32 max_map_size>
void Map<K, V, max_map_size>::sort_() {
  if (std::is_sorted(get_ptr(0), get_ptr(n_))) {
    return;
  }

  std::sort(get_ptr(0), get_ptr(n_));
}

template <typename K, typename V, s32 max_map_size>
void Map<K, V, max_map_size>::assert_unique() const {
  assert(n_ >= 0 && n_ <= max_size());
  assert(n_ == static_cast<s32>(std::distance(
                   get_ptr(0), std::adjacent_find(get_ptr(0), get_ptr(n_)))));
}

template <typename K, typename V, s32 max_map_size>
void Map<K, V, max_map_size>::assert_sorted() const {
  assert(n_ >= 0 && n_ <= max_size());
  assert(std::is_sorted(get_ptr(0), get_ptr(n_)));
}

template <typename K, typename V, s32 max_map_size>
template <s32 other_max_map_size>
bool Map<K, V, max_map_size>::operator==(
    const Map<K, V, other_max_map_size> &other) const {
  if (n_ != other.n_) {
    return false;
  }

  for (s32 i = 0; i < n_; i++) {
    if (!at(i).kv_equals(other.at(i))) {
      return false;
    }
  }

  return true;
}

template <typename K, typename V, s32 max_map_size>
Set<K, max_map_size> Map<K, V, max_map_size>::keys() const {
  Set<K, max_map_size> ret;
  for (const auto &elem : *this) {
    ret.add(elem.key);
  }

  return ret;
}

template <typename K, typename V, s32 max_map_size>
typename Map<K, V, max_map_size>::EigenArrayOfValues
Map<K, V, max_map_size>::values() const {
  EigenArrayOfValues arr(size());
  s32 elem_index = 0;
  for (const auto &elem : *this) {
    arr(elem_index++) = elem.value;
  }

  return arr;
}

template <typename K, typename V, s32 max_map_size>
template <typename C, typename VA, typename VB, s32 size1, s32 size2>
Map<K, V, max_map_size> &Map<K, V, max_map_size>::intersect_on(
    optional<V> (*merge_f)(const K &key, const VA &a_value, const VB &b_value,
                           C helper),
    const Map<K, VA, size1> &a, const Map<K, VB, size2> &b, C helper) {
  static_assert(size1 <= max_map_size,
                "Input map must be no larger than output map");
  static_assert(size2 <= max_map_size,
                "Input map must be no larger than output map");
  using helper_t =
      std::tuple<optional<V> (*)(const K &, const VA &, const VB &, C), C>;
  const auto merge_elems = [](const MapElement<K, VA> &ea,
                              const MapElement<K, VB> &eb,
                              helper_t halp) -> optional<Element> {
    const auto val =
        std::get<0>(halp)(ea.key, ea.value, eb.value, std::get<1>(halp));
    if (val.has_value()) {
      return Element(ea.key, *val);
    }

    return {};
  };

  n_ = static_cast<s32>(
      internal::intersect_on_multitype<Element, MapElement<K, VA>,
                                       MapElement<K, VB>, helper_t>(
          merge_elems, get_ptr(0), static_cast<std::size_t>(max_size()),
          a.cbegin(), static_cast<std::size_t>(a.size()), b.cbegin(),
          static_cast<std::size_t>(b.size()),
          std::forward_as_tuple(merge_f, helper)));
  return *this;
}

template <typename K, typename V, s32 max_map_size>
template <typename VA, typename VB, s32 size1, s32 size2>
Map<K, V, max_map_size> &Map<K, V, max_map_size>::intersect_on(
    optional<V> (*merge_f)(const K &key, const VA &a_value, const VB &b_value),
    const Map<K, VA, size1> &a, const Map<K, VB, size2> &b) {
  return intersect_on(&Map::merge_reroute_<optional<V>, VA, VB>, a, b, merge_f);
}

template <typename K, typename V, s32 max_map_size>
template <typename C, typename VO, s32 other_max_size>
Map<K, V, max_map_size> &Map<K, V, max_map_size>::intersect_on(
    optional<V> (*merge_f)(const K &key, const V &this_value,
                           const VO &other_value, C helper),
    const Map<K, VO, other_max_size> &other, C helper) {
  return intersect_on<C, V, VO, max_map_size, other_max_size>(merge_f, *this,
                                                              other, helper);
}

template <typename K, typename V, s32 max_map_size>
template <typename VO, s32 other_max_size>
Map<K, V, max_map_size> &Map<K, V, max_map_size>::intersect_on(
    optional<V> (*merge_f)(const K &key, const V &this_value,
                           const VO &other_value),
    const Map<K, VO, other_max_size> &other) {
  return intersect_on<V, VO, max_map_size, other_max_size>(merge_f, *this,
                                                           other);
}

template <typename K, typename V, s32 max_map_size>
template <typename C>
Map<K, V, max_map_size> &Map<K, V, max_map_size>::intersect_on(
    optional<V> (*merge_f)(const K &key, const V &a_value, const V &b_value,
                           C helper),
    const Map &a, const Map &b, C helper) {
  // Call out to the type-varying function
  return intersect_on<C, V, V, max_map_size, max_map_size>(merge_f, a, b,
                                                           helper);
}

template <typename K, typename V, s32 max_map_size>
template <typename R, typename VA, typename VB>
R Map<K, V, max_map_size>::merge_reroute_(const K &key, const VA &left_value,
                                          const VB &right_value,
                                          R (*merge_f)(const K &, const VA &,
                                                       const VB &)) {
  // This is like `call_reroute_()` but for two-argument merging
  // functions with different argument types.
  return merge_f(key, left_value, right_value);
}

template <typename K, typename V, s32 max_map_size>
template <typename R>
R Map<K, V, max_map_size>::call_reroute_(const Element &a,
                                         R (*element_f)(const Element &)) {
  // The saga of call_reroute_!
  //
  // * We cannot use `std::function`; it wants RTTI and exceptions,
  //   and it uses the heap to boot.
  //
  // * Lambdas that capture cannot be passed as function pointers, but
  //   functions like `filter()` or `transform()` are massively more
  //   convenient if you can just lay down a lambda and pass it in.
  //
  // * Sometimes you want to capture in a lambda anyway.
  //
  // So instead, you can pass some additional arguments to your lambda
  // using the `helper` argument of things like `filter()` and
  // `filtered()`.  Those functions are written to plumb the helper
  // argument through.  This makes the interface annoying if you only
  // want one argument, though, so we pass the real `element_f` from
  // the single-argument function as the helper argument to
  // `call_reroute_()`, then invoke it on the element in question.
  return element_f(a);
}

/**
 * Identical to #call_reroute_, except that it takes a const reference function
 * pointer.
 */
template <typename K, typename V, s32 max_map_size>
template <typename R>
R Map<K, V, max_map_size>::call_reroute_ref_(
    const Element &a, R (*const &element_f)(const Element &)) {
  return element_f(a);
}

template <typename K, typename V, s32 max_map_size>
optional<V> Map<K, V, max_map_size>::choose_left_optional(
    const K &key SWIFT_ATTR_UNUSED, const V &left_value,
    const V &right_value SWIFT_ATTR_UNUSED) {
  return left_value;
}

template <typename K, typename V, s32 max_map_size>
V Map<K, V, max_map_size>::choose_left(const K &key SWIFT_ATTR_UNUSED,
                                       const V &left_value,
                                       const V &right_value SWIFT_ATTR_UNUSED) {
  return left_value;
}

template <typename K, typename V, s32 max_map_size>
template <typename C>
Map<K, V, max_map_size> &Map<K, V, max_map_size>::intersect_on(
    optional<V> (*merge_f)(const K &key, const V &this_value,
                           const V &other_value, C helper),
    const Map<K, V, max_map_size> &other, C helper) {
  return intersect_on(merge_f, *this, other, helper);
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size> &Map<K, V, max_map_size>::intersect_on(
    optional<V> (*merge_f)(const K &key, const V &a_value, const V &b_value),
    const Map<K, V, max_map_size> &a, const Map<K, V, max_map_size> &b) {
  return intersect_on(merge_reroute_, a, b, merge_f);
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size> &Map<K, V, max_map_size>::intersect_on(
    optional<V> (*merge_f)(const K &key, const V &this_value,
                           const V &other_value),
    const Map<K, V, max_map_size> &other) {
  return intersect_on(merge_reroute_, other, merge_f);
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size> &Map<K, V, max_map_size>::intersect(
    const Map<K, V, max_map_size> &other) {
  return intersect_on(choose_left_optional, other);
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size> &Map<K, V, max_map_size>::intersect(
    const Map<K, V, max_map_size> &a, const Map<K, V, max_map_size> &b) {
  return intersect_on(choose_left_optional, a, b);
}

template <typename K, typename V, s32 max_map_size>
template <typename C, typename VO, s32 other_max_size>
Map<K, V, max_map_size> Map<K, V, max_map_size>::intersected_on(
    optional<V> (*merge_f)(const K &key, const V &this_value,
                           const VO &other_value, C helper),
    const Map<K, VO, other_max_size> &other, C helper) const & {
  Map<K, V, max_map_size> ret;
  ret.intersect_on<C, V, VO, max_map_size, other_max_size>(merge_f, *this,
                                                           other, helper);
  return ret;
}

template <typename K, typename V, s32 max_map_size>
template <typename C, typename VO, s32 other_max_size>
Map<K, V, max_map_size> Map<K, V, max_map_size>::intersected_on(
    optional<V> (*merge_f)(const K &key, const V &this_value,
                           const VO &other_value, C helper),
    const Map<K, VO, other_max_size> &other, C helper) && {
  intersect_on<C, VO, other_max_size>(merge_f, other, helper);
  return std::move(*this);
}

template <typename K, typename V, s32 max_map_size>
template <typename VO, s32 other_max_size>
Map<K, V, max_map_size> Map<K, V, max_map_size>::intersected_on(
    optional<V> (*merge_f)(const K &key, const V &this_value,
                           const VO &other_value),
    const Map<K, VO, other_max_size> &other) const & {
  return intersected_on(&Map::merge_reroute_<optional<V>, V, VO>, *this, other,
                        merge_f);
}

template <typename K, typename V, s32 max_map_size>
template <typename VO, s32 other_max_size>
Map<K, V, max_map_size> Map<K, V, max_map_size>::intersected_on(
    optional<V> (*merge_f)(const K &key, const V &this_value,
                           const VO &other_value),
    const Map<K, VO, other_max_size> &other) && {
  intersect_on(&Map::merge_reroute_<optional<V>, V, VO>, other, merge_f);
  return std::move(*this);
}

template <typename K, typename V, s32 max_map_size>
template <typename C>
Map<K, V, max_map_size> Map<K, V, max_map_size>::intersected_on(
    optional<V> (*merge_f)(const K &key, const V &this_value,
                           const V &other_value, C helper),
    const Map<K, V, max_map_size> &other, C helper) const & {
  Map<K, V, max_map_size> ret;
  ret.intersect_on(merge_f, *this, other, helper);
  return ret;
}

// NOTE ON RVALUE REFERENCE OVERLOADS
//
// These are here for a reason -- please take a look at `set.h` at the
// comment with the same heading, "NOTE ON . . ." if you want to learn
// more.
template <typename K, typename V, s32 max_map_size>
template <typename C>
Map<K, V, max_map_size> Map<K, V, max_map_size>::intersected_on(
    optional<V> (*merge_f)(const K &key, const V &this_value,
                           const V &other_value, C helper),
    const Map<K, V, max_map_size> &other, C helper) && {
  intersect_on<C>(merge_f, other, helper);
  return std::move(*this);
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size> Map<K, V, max_map_size>::intersected_on(
    optional<V> (*merge_f)(const K &key, const V &this_value,
                           const V &other_value),
    const Map<K, V, max_map_size> &other) const & {
  return intersected_on(merge_reroute_, other, merge_f);
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size> Map<K, V, max_map_size>::intersected_on(
    optional<V> (*merge_f)(const K &key, const V &this_value,
                           const V &other_value),
    const Map<K, V, max_map_size> &other) && {
  return std::move(intersected_on(merge_reroute_, other, merge_f));
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size> Map<K, V, max_map_size>::intersected(
    const Map<K, V, max_map_size> &other) const & {
  return intersected_on(choose_left_optional, other);
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size> Map<K, V, max_map_size>::intersected(
    const Map<K, V, max_map_size> &other) && {
  return std::move(intersected_on(choose_left_optional, other));
}

template <typename K, typename V, s32 max_map_size>
template <typename C>
optional<Map<K, V, max_map_size> &> Map<K, V, max_map_size>::extend_on_impl(
    V (*merge_f)(const K &key, const V &this_value, const V &other_value,
                 C helper),
    const Map &a, const Map &b, C helper) {
  using helper_t = std::tuple<V (*)(const K &, const V &, const V &, C), C>;
  const auto merge_elements = [](const Element &ea, const Element &eb,
                                 helper_t halp) {
    return Element(ea.key, std::get<0>(halp)(ea.key, ea.value, eb.value,
                                             std::get<1>(halp)));
  };

  const auto extent = internal::extend_on<Element, helper_t>(
      merge_elements, get_ptr(0), static_cast<std::size_t>(max_size()),
      a.cbegin(), static_cast<std::size_t>(a.size()), b.cbegin(),
      static_cast<std::size_t>(b.size()), std::make_tuple(merge_f, helper));

  if (extent.has_value()) {
    n_ = static_cast<s32>(*extent);
    return *this;
  }

  return {};
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size> &Map<K, V, max_map_size>::extend_on(
    V (*merge_f)(const K &key, const V &a_value, const V &b_value),
    const Map &a, const Map &b) {
  return extend_on(merge_reroute_, a, b, merge_f);
}

template <typename K, typename V, s32 max_map_size>
template <typename C>
Map<K, V, max_map_size> &Map<K, V, max_map_size>::extend_on(
    V (*merge_f)(const K &key, const V &a_value, const V &b_value, C helper),
    const Map &a, const Map &b, C helper) {
  Map temp;
  const auto extent = temp.extend_on_impl(merge_f, a, b, helper);
  (void)extent;
  assert(extent.has_value());
  *this = temp;
  return *this;
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size> &Map<K, V, max_map_size>::extend_on(
    V (*merge_f)(const K &key, const V &this_value, const V &other_value),
    const Map &other) {
  return extend_on(merge_reroute_, *this, other, merge_f);
}

template <typename K, typename V, s32 max_map_size>
template <typename C>
Map<K, V, max_map_size> &Map<K, V, max_map_size>::extend_on(
    V (*merge_f)(const K &key, const V &this_value, const V &other_value,
                 C helper),
    const Map &other, C helper) {
  return extend_on(merge_f, *this, other, helper);
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size> &Map<K, V, max_map_size>::extend(const Map &other) {
  return extend_on(choose_left, other);
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size> &Map<K, V, max_map_size>::extend(const Map &a,
                                                         const Map &b) {
  return extend_on(choose_left, a, b);
}

template <typename K, typename V, s32 max_map_size>
template <typename C>
Map<K, V, max_map_size> Map<K, V, max_map_size>::extended_on(
    V (*merge_f)(const K &key, const V &this_value, const V &other_value,
                 C helper),
    const Map &other, C helper) const & {
  Map ret;
  const auto extent = ret.extend_on_impl(merge_f, *this, other, helper);
  (void)extent;
  assert(extent.has_value());
  return ret;
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size> Map<K, V, max_map_size>::extended_on(
    V (*merge_f)(const K &key, const V &this_value, const V &other_value),
    const Map &other) const & {
  return extended_on(merge_reroute_, other, merge_f);
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size> Map<K, V, max_map_size>::extended(
    const Map &other) const & {
  return extended_on(choose_left, other);
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size> &Map<K, V, max_map_size>::difference(const Map &a,
                                                             const Map &b) {
  n_ = static_cast<s32>(internal::difference<Element>(
      get_ptr(0), static_cast<std::size_t>(max_size()), a.cbegin(),
      static_cast<std::size_t>(a.size()), b.cbegin(),
      static_cast<std::size_t>(b.size())));
  return *this;
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size> &Map<K, V, max_map_size>::difference(const Map &other) {
  return difference(*this, other);
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size> Map<K, V, max_map_size>::differenced(
    const Map &other) const & {
  Map ret;
  ret.difference(*this, other);
  return ret;
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size> Map<K, V, max_map_size>::differenced(
    const Map &other) && {
  difference(other);
  return std::move(*this);
}

template <typename K, typename V, s32 max_map_size>
template <s32 new_max_map_size>
Map<K, V, new_max_map_size> Map<K, V, max_map_size>::grown() const {
  static_assert(new_max_map_size >= max_map_size,
                "Map<K, V, max_set_size>::grown(): new maximum size is smaller "
                "than current");
  return Map<K, V, new_max_map_size>(get_ptr(0), get_ptr(n_));
}

template <typename K, typename V, s32 max_map_size>
template <s32 new_max_map_size>
Map<K, V, new_max_map_size> Map<K, V, max_map_size>::resized() const {
  assert(new_max_map_size >= size());
  return Map<K, V, new_max_map_size>(get_ptr(0), get_ptr(n_));
}

template <typename K, typename V, s32 max_map_size>
template <typename FT>
optional<Map<K, V, max_map_size> &> Map<K, V, max_map_size>::insert_impl_(
    const K &key, const V &value, FT handle_present) {
  const auto less_than_by_key = [](const Element &elem, const K &find_k) {
    return elem.key < find_k;
  };
  Element *const spot =
      std::lower_bound(get_ptr(0), get_ptr(n_), key, less_than_by_key);
  if (spot < get_ptr(n_) && key == spot->key) {
    return handle_present(spot);
  }

  assert(size() < max_size());

  /**
   * if the location for the new Element has been determined to be at the end of
   * the element array, simply placement construct the element into the
   * uninitialized slot. otherwise if the new element is expected to sit
   * somewhere inside the element array, we need to first move the right most
   * element into the uninitialized slot before right shifting all the elements
   * right one slot to make room for the new element to be assigned to its
   * designated spot.
   */
  if (spot == get_ptr(n_) || n_ == 0) {
    new (get_ptr(n_)) Element(key, value);
  } else {
    new (get_ptr(n_)) Element(std::move(at(n_ - 1)));
    std::move_backward(spot, get_ptr(n_ - 1), get_ptr(n_));
    spot->key = key;
    spot->value = value;
  }
  n_++;

  return *this;
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size> &Map<K, V, max_map_size>::insert(const K &key,
                                                         const V &value) {
  insert_impl_(key, value,
               // If we already have this element, just overwrite the
               // value.
               [&](Element *const spot) -> optional<Map<K, V, max_map_size> &> {
                 spot->value = value;
                 return *this;
               });
  return *this;
}

template <typename K, typename V, s32 max_map_size>
optional<Map<K, V, max_map_size> &> Map<K, V, max_map_size>::insert_missing(
    const K &key, const V &value) {
  return insert_impl_(
      // If we already have this element, just return an empty object
      key, value,
      [](Element *const spot SWIFT_ATTR_UNUSED)
          -> optional<Map<K, V, max_map_size> &> { return {}; });
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size> Map<K, V, max_map_size>::inserted(
    const K &key, const V &value) const & {
  Map ret(*this);
  ret.insert(key, value);
  return ret;
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size> Map<K, V, max_map_size>::inserted(const K &key,
                                                          const V &value) && {
  insert(key, value);
  return std::move(*this);
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size> &Map<K, V, max_map_size>::refine(const K &key) {
  return refine<max_map_size>(Set<K, max_map_size>(key));
}

template <typename K, typename V, s32 max_map_size>
template <s32 set_size>
Map<K, V, max_map_size> &Map<K, V, max_map_size>::refine(
    const Set<K, set_size> &keep_keys) {
  constexpr s32 max_size = max(set_size, max_map_size);
  drop(keys().template grown<max_size>().difference(
      keep_keys.template grown<max_size>()));

  return *this;
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size> Map<K, V, max_map_size>::refined(const K &key) const & {
  Map ret(*this);
  ret.refine(key);
  return ret;
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size> Map<K, V, max_map_size>::refined(const K &key) && {
  refine(key);
  return std::move(*this);
}

template <typename K, typename V, s32 max_map_size>
template <s32 set_size>
Map<K, V, max_map_size> Map<K, V, max_map_size>::refined(
    const Set<K, set_size> &keys) const & {
  Map ret(*this);
  ret.refine<set_size>(keys);
  return ret;
}

template <typename K, typename V, s32 max_map_size>
template <s32 set_size>
Map<K, V, max_map_size> Map<K, V, max_map_size>::refined(
    const Set<K, set_size> &keys) && {
  refine<set_size>(keys);
  return std::move(*this);
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size> &Map<K, V, max_map_size>::drop(const K &key) {
  // We don't actually need 8 elements here, but if we do fewer, g++
  // dies with an error about stack-smashing protection.  Of course we
  // do want these runtime checks, but there doesn't seem to be a
  // great way to generate errors for larger arrays but not for ones
  // below the threshold.
  return drop<8>(Set<K, 8>(key));
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size> &Map<K, V, max_map_size>::drop_indices_(
    std::size_t drop_count,
    std::array<std::size_t, max_map_size + 1> *drop_indices) {
  assert(nullptr != drop_indices);
  // Insert an ending element, but don't increment `drop_count` to
  // prevent the next pass from overflowing.
  (*drop_indices)[drop_count] = static_cast<std::size_t>(n_);

  // Second pass: remove map elements from the internal array in
  // ascending order such that we only ever move an element once.
  for (std::size_t d = 0; d < drop_count; d++) {
    const std::size_t to_drop =
        (*drop_indices)[d];  // Current index to be dropped
    const std::size_t next_drop =
        (*drop_indices)[d + 1];  // Next index to be dropped
    std::move(
        // Starting with the first element after the one to be dropped
        get_ptr(0) + to_drop + 1,
        // Ending with the element preceding the next one to be dropped
        get_ptr(0) + next_drop,
        // To the range beginning `d` spots before the current element
        // to drop; this accounts for all the previously dropped
        // elements.
        get_ptr(0) + to_drop - d);
  }

  n_ -= static_cast<s32>(drop_count);

  return *this;
}

template <typename K, typename V, s32 max_map_size>
template <s32 set_size>
Map<K, V, max_map_size> &Map<K, V, max_map_size>::drop(
    const Set<K, set_size> &keys) {
  // First pass: identify indices in our internal array that need to
  // be dropped.
  std::size_t drop_count = 0;
  std::array<std::size_t, max_map_size + 1> drop_indices{};
  std::size_t elem_idx = 0;
  for (auto kptr = keys.begin();
       kptr != keys.end() && elem_idx < static_cast<std::size_t>(n_);) {
    const K &key = *kptr;
    const K &elem = at(static_cast<s32>(elem_idx)).key;
    if (key < elem) {
      // Got to advance the set pointer
      kptr++;
    } else if (elem < key) {
      // Got to advance the map index
      elem_idx++;
    } else {
      // Found a match.
      drop_indices[drop_count++] = elem_idx;
      elem_idx++;
      kptr++;
    }
  }

  // Perform an efficient multi-drop.
  return drop_indices_(drop_count, &drop_indices);
}

template <typename K, typename V, s32 max_map_size>
template <s32 set_size>
Map<K, V, max_map_size> Map<K, V, max_map_size>::dropped(
    const Set<K, set_size> &keys) const & {
  Map ret(*this);
  ret.drop(keys);
  return ret;
}

template <typename K, typename V, s32 max_map_size>
template <s32 set_size>
Map<K, V, max_map_size> Map<K, V, max_map_size>::dropped(
    const Set<K, set_size> &keys) && {
  drop(keys);
  return std::move(*this);
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size> Map<K, V, max_map_size>::dropped(const K &key) const & {
  return dropped(Set<K, max_map_size>(key));
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size> Map<K, V, max_map_size>::dropped(const K &key) && {
  return std::move(dropped(Set<K, max_map_size>(key)));
}

template <typename K, typename V, s32 max_map_size>
optional<s32> Map<K, V, max_map_size>::find(const K &key) const {
  const auto less_than_by_key = [](const MapElement<K, V> &elem,
                                   const K &find_k) {
    return elem.key < find_k;
  };
  auto iter = std::lower_bound(get_ptr(0), get_ptr(n_), key, less_than_by_key);
  if ((iter != get_ptr(n_)) && !(key < iter->key) && !(iter->key < key)) {
    return static_cast<s32>(iter - get_ptr(0));
  }

  return {};
}

template <typename K, typename V, s32 max_map_size>
optional<const V &> Map<K, V, max_map_size>::lookup(const K &key) const & {
  optional<s32> found_key = find(key);
  if (found_key) {
    return at(*found_key).value;
  }

  return {};
}

template <typename K, typename V, s32 max_map_size>
optional<V &> Map<K, V, max_map_size>::lookup(const K &key) & {
  optional<s32> found_key = find(key);
  if (found_key) {
    return at(*found_key).value;
  }

  return {};
}

template <typename K, typename V, s32 max_map_size>
optional<V> Map<K, V, max_map_size>::lookup(const K &key) && {
  optional<s32> found_key = find(key);
  if (found_key) {
    return at(*found_key).value;
  }

  return {};
}

// Synonym for `lookup()`
template <typename K, typename V, s32 max_map_size>
optional<const V &> Map<K, V, max_map_size>::operator[](const K &key) const & {
  return lookup(key);
}

// Synonym for `lookup()`
template <typename K, typename V, s32 max_map_size>
optional<V &> Map<K, V, max_map_size>::operator[](const K &key) & {
  return lookup(key);
}

// Synonym for `lookup()`
template <typename K, typename V, s32 max_map_size>
optional<V> Map<K, V, max_map_size>::operator[](const K &key) && {
  return lookup(key);
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size> &Map<K, V, max_map_size>::transform(
    V (*transform_f)(const MapElement<K, V> &elem)) {
  return transform(call_reroute_ref_, transform_f);
}

template <typename K, typename V, s32 max_map_size>
template <typename C>
Map<K, V, max_map_size> &Map<K, V, max_map_size>::transform(
    V (*transform_f)(const MapElement<K, V> &elem,
                     const std::decay_t<C> &helper),
    C &&helper) {
  for (auto eptr = get_ptr(0); eptr < get_ptr(n_); ++eptr) {
    eptr->value = transform_f(*eptr, helper);
  }
  return *this;
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size> Map<K, V, max_map_size>::transformed(
    V (*transform_f)(const MapElement<K, V> &elem)) const & {
  return transformed(call_reroute_ref_, transform_f);
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size> Map<K, V, max_map_size>::transformed(
    V (*transform_f)(const MapElement<K, V> &elem)) && {
  return std::move(transformed(call_reroute_ref_, transform_f));
}

template <typename K, typename V, s32 max_map_size>
template <typename C>
Map<K, V, max_map_size> Map<K, V, max_map_size>::transformed(
    V (*transform_f)(const MapElement<K, V> &elem,
                     const std::decay_t<C> &helper),
    C &&helper) const & {
  Map temp(*this);
  temp.transform(transform_f, std::forward<C>(helper));
  return temp;
}

template <typename K, typename V, s32 max_map_size>
template <typename C>
Map<K, V, max_map_size> Map<K, V, max_map_size>::transformed(
    V (*transform_f)(const MapElement<K, V> &elem,
                     const std::decay_t<C> &helper),
    C &&helper) && {
  transform(transform_f, std::forward<C>(helper));
  return std::move(*this);
}

// Return a new map with a new value type computed by the
// transformation function.
template <typename K, typename V, s32 max_map_size>
template <typename U>
Map<K, U, max_map_size> Map<K, V, max_map_size>::transformed(
    U (*transform_f)(const MapElement<K, V> &elem)) const {
  return transformed<U, U (*const &)(const MapElement<K, V> &)>(
      call_reroute_ref_, transform_f);
}

template <typename K, typename V, s32 max_map_size>
template <typename U, typename C>
Map<K, U, max_map_size> Map<K, V, max_map_size>::transformed(
    U (*transform_f)(const MapElement<K, V> &elem,
                     const std::decay_t<C> &helper),
    C &&helper) const {
  Map<K, U, max_map_size> ret;
  for (const auto &elem : *this) {
    ret.insert(elem.key, transform_f(elem, helper));
  }
  return ret;
}

template <typename K, typename V, s32 max_map_size>
template <typename U>
Map<K, U, max_map_size> Map<K, V, max_map_size>::transformed(
    optional<U> (*transform_f)(const MapElement<K, V> &elem)) const {
  return transformed<U>(call_reroute_ref_, transform_f);
}

template <typename K, typename V, s32 max_map_size>
template <typename U, typename C>
Map<K, U, max_map_size> Map<K, V, max_map_size>::transformed(
    optional<U> (*transform_f)(const MapElement<K, V> &elem,
                               const std::decay_t<C> &helper),
    C &&helper) const {
  Map<K, U, max_map_size> ret;
  for (const auto &elem : *this) {
    const auto got = transform_f(elem, helper);
    if (got) {
      ret.insert(elem.key, *got);
    }
  }

  return ret;
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size> &Map<K, V, max_map_size>::filter(
    bool (*select_f)(const MapElement<K, V> &elem)) {
  return filter(call_reroute_ref_, select_f);
}

template <typename K, typename V, s32 max_map_size>
template <typename C>
Map<K, V, max_map_size> &Map<K, V, max_map_size>::filter(
    bool (*select_f)(const MapElement<K, V> &elem,
                     const std::decay_t<C> &helper),
    C &&helper) {
  std::size_t drop_count = 0;
  std::array<std::size_t, max_map_size + 1> drop_indices{};
  for (std::size_t i = 0; i < static_cast<std::size_t>(n_); i++) {
    if (!select_f(at(static_cast<s32>(i)), helper)) {
      drop_indices[drop_count++] = i;
    }
  }

  return drop_indices_(drop_count, &drop_indices);
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size> Map<K, V, max_map_size>::filtered(
    bool (*select_f)(const MapElement<K, V> &elem)) const & {
  return filtered(call_reroute_ref_, select_f);
}

template <typename K, typename V, s32 max_map_size>
Map<K, V, max_map_size> Map<K, V, max_map_size>::filtered(
    bool (*select_f)(const MapElement<K, V> &elem)) && {
  return std::move(filtered(call_reroute_ref_, select_f));
}

template <typename K, typename V, s32 max_map_size>
template <typename C>
Map<K, V, max_map_size> Map<K, V, max_map_size>::filtered(
    bool (*select_f)(const MapElement<K, V> &elem,
                     const std::decay_t<C> &helper),
    C &&helper) const & {
  Map ret(*this);
  ret.filter(select_f, std::forward<C>(helper));
  return ret;
}

template <typename K, typename V, s32 max_map_size>
template <typename C>
Map<K, V, max_map_size> Map<K, V, max_map_size>::filtered(
    bool (*select_f)(const MapElement<K, V> &elem,
                     const std::decay_t<C> &helper),
    C &&helper) && {
  filter(select_f, std::forward<C>(helper));
  return std::move(*this);
}

template <typename K, typename V, s32 max_map_size>
template <typename C>
C Map<K, V, max_map_size>::traverse(
    C (*traverse_f)(const MapElement<K, V> &elem, const C &accumulator),
    const C &accumulator) const {
  C ret = accumulator;

  for (const auto &elem : *this) {
    ret = traverse_f(elem, ret);
  }

  return ret;
}

template <typename K, typename V, s32 max_map_size>
std::tuple<Map<K, V, max_map_size>, Map<K, V, max_map_size>>
Map<K, V, max_map_size>::partitioned(
    bool (*select_f)(const MapElement<K, V> &elem)) const {
  return partitioned(call_reroute_ref_, select_f);
}

template <typename K, typename V, s32 max_map_size>
template <typename C>
std::tuple<Map<K, V, max_map_size>, Map<K, V, max_map_size>>
Map<K, V, max_map_size>::partitioned(
    bool (*select_f)(const MapElement<K, V> &elem,
                     const std::decay_t<C> &helper),
    C &&helper) const {
  Map left, right;
  for (const auto &elem : *this) {
    if (select_f(elem, helper)) {
      left.insert(elem.key, elem.value);
    } else {
      right.insert(elem.key, elem.value);
    }
  }

  return std::make_tuple(left, right);
}

/// @endcond
// (CAN_DOXYGEN_CORRECTLY_PARSE_TEMPLATES_YET)

}  // namespace containers

}  // namespace pvt_common

#endif  // LIBSWIFTNAV_PVT_ENGINE_CONTAINERS_MAP_H
