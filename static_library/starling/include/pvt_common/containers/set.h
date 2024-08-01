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

#ifndef LIBSWIFTNAV_PVT_ENGINE_CONTAINERS_SET_H
#define LIBSWIFTNAV_PVT_ENGINE_CONTAINERS_SET_H

#include <pvt_common/eigen_custom.h>
#include <pvt_engine/optional.h>
#include <swiftnav/common.h>
#include "shared.h"

#include <algorithm>
#include <cassert>
#include <tuple>
#include <vector>

namespace pvt_common {

namespace containers {

template <typename T, typename C, s32 max_set_size>
struct FilteredSetView;

// This class lives on the stack and keeps an array of unique values
// of type T.  It is a container, like `std::set`, but it has a more
// mathematical interface and encourages doing things inline instead
// of using iterators and other STL features, many of which require a
// heap.
//
//                           REQUIREMENTS
//
// To put values of type T in this container, T must be equipped with
//
//     bool operator<(const T &other)
//
//   and
//
//     bool operator==(const T &other)
//
// `operator<()` should induce a total ordering on elements of T.  A
// set contains unique values, so if you have two pieces of data you
// can tell apart but on which `operator==()` returns `true`, this
// container will only hold one of them at a time.  See the
// documentation for constructors, `extend()`, `add()` etc. if you
// need to understand how the container will behave with respect to
// such data.
//
//                        NAMING CONVENTIONS
//
// Member functions whose names are participles (`intersected()`,
// `filtered()`, `grown()`) return a new set object, with the
// operation applied to the original (`this`) set.  Members whose
// names are imperative verbs (`intersect()`, `filter()`, `add()`)
// modify the object they're called on and return a reference to
// enable method chaining.
//
//                    FUNCTIONS AS ARGUMENTS
//
// To make it easier to use higher-order functions without a heap,
// many member functions have overloads that are templatized over a
// helper value of type `C` (e.g. `filter()`, `transformed()`).  When
// you use these functions, especially if you pass them a lambda
// function, you may need to explicitly specify the type C as a
// template argument to the function.  In the below example, the
// helper value `threshold` is of type `s32`, so we explicitly write
// `filter<s32>()`.
//
//     s32 cutoff_value = 22;
//
//     const auto threshold_selector =
//         [](const s32 &el, s32 threshold) {
//       return el.value > threshold;
//     };
//
//     my_set.filter<s32>(threshold_selector, cutoff_value);
//
// Note that for some functions like the type-changing
// `transformed()`, you will likely need to pass the template
// arguments for both the new member type _and_ the helper type, e.g.
//
//     Set<bar> x =
//       myset.transformed<bar, const foo &>(morph, foos);
//
// If you are passing a lambda to a member function of a set, you may
// need to explicitly specify the return type of the lambda.  This can
// happen with functions like `transformed()`, which expects a
// function returning `optional<V>`.  If your lambda always returns a
// `V` (i.e. can never return an empty object `{}`), the compiler will
// not come up with the right return type, and the errors may not be
// helpful.  For example, instead of
//
//     my_set.transformed([](const elem_t &e) { return e + 22.2; });
//
// you may need to write
//
//     my_set.transformed([](const elem_t &e) -> optional<elem_t>
//                        { return e + 22.2; });
//
// Sorry about this.
//
//                   PERFORMANCE OF CHAINED FUNCTIONS
//
// Many member functions that return a new set object,
// e.g. `intersected()` or `with()`, have multiple overloads: one
// where `*this` is a `const Set &` and one where `*this` is a `Set
// &&`.  The additional overloads are purely for performance reasons;
// they will kick in automatically where possible, and you can
// completely ignore them if you just want to use the container.  If
// you're worried about performance, the best thing you can do is
// chain several calls together in one rvalue, e.g.
//
//    const auto a = b
//        .intersected(c)
//        .filtered(is_a_foo)
//        .transformed(enswizzle);
//
// This should result in only one copy of `b`'s data being made and
// all three operations (intersect, filter, transformation) being
// performed in-place on that copy.
//
// There is some more detailed discussion for implementors in the
// comments accompanying the out-of-line implementations of these
// overloads.
template <typename T, s32 max_set_size>
class Set {
  static_assert(std::is_trivially_destructible<T>::value,
                "Set requires a trivially destructible template class for "
                "its elements.");

 public:
  // Construct an empty set.
  Set() : elems_(){};

  // Construct a set from an array `elems` of `n` elements.  Any
  // duplicate elements in `elems` will be removed; the first
  // occurrence of any elements which are equal according to
  // `operator==()` will be preserved.  `n` must be no greater than
  // the compile-time maximum set size `max_set_size` of this
  // container.
  Set(s32 n, const T elems[]);

  // Construct a set from a vector `elems` of elements.  Any duplicate
  // elements in `elems` will be removed; the first occurrence of any
  // elements which are equal according to `operator==()` will be
  // preserved.  `elems` must contain no more elements than the
  // compile-time maximum set size `max_set_size` of this container.
  //
  // This constructor is only used for testing.  You may not use STL
  // containers in the embedded code.
  explicit Set(const std::vector<T> &elems);

  // Construct a set from a single element.
  explicit Set(T elem);

  // Construct a set using an initializer list.
  Set(std::initializer_list<T> initlist);

  // Construct a set from beginning and ending key iterators and a
  // value iterator.  The resulting set will contain the number of
  // elements spanned by the iterators.
  template <typename ElemIter>
  Set(ElemIter elem_begin, ElemIter elem_end);

  // The following members are defined to avoid unnecessary copying.
  Set(const Set &other) : n_(other.n_), elems_() {
    std::copy(&other.at(0), &other.at(n_), &at(0));
  }

  Set(Set &&other) noexcept : n_(other.n_), elems_() {
    std::copy(&other.at(0), &other.at(n_), &at(0));
  }

  Set &operator=(const Set &other) {
    n_ = other.n_;
    std::copy(&other.at(0), &other.at(n_), &at(0));
    return *this;
  }

  Set &operator=(Set &&other) noexcept {
    n_ = other.n_;
    std::copy(&other.at(0), &other.at(n_), &at(0));
    return *this;
  }

  // Compare `this` set with an`other` for equality.
  template <s32 other_max_set_size>
  bool operator==(const Set<T, other_max_set_size> &other) const;

  // Compare `this` set with an`other` for inequality.
  template <s32 other_max_set_size>
  bool operator!=(const Set<T, other_max_set_size> &other) const {
    return !(*this == other);
  }

  // Test whether `this` set already contains `elem`.  Returns `true`
  // if `elem` is already a member of `this` set and `false`
  // otherwise.
  bool contains(const T &elem) const;

  // Return the number of elements stored in `this` set.
  s32 size() const { return n_; }

  // Return the maximum number of elements `this` set can contain.
  static s32 max_size() { return max_set_size; }

  // Return a new set containing the same members as `this` set, but
  // with a new maximum size (provided as a template argument).  Note
  // that this new maximum size `new_max_set_size` must be larger than
  // the current maximum size (`max_set_size`).
  template <s32 new_max_set_size>
  Set<T, new_max_set_size> grown() const;

  // Return a new set containing the same members as `this` set, but
  // with a new maximum size (provided as a template argument).  Note
  // that this new maximum size `new_max_set_size` must be larger than
  // the current size (`size()`).  If it is not, an empty object is
  // returned.
  template <s32 new_max_set_size>
  optional<Set<T, new_max_set_size>> shrunk() const;

  // Compute the intersection of `this` set with the `other`.  On
  // return, `this` set will contain the resulting subset.
  Set &intersect(const Set &other);

  // Compute the intersection of sets `a` and `b`, storing the
  // resulting subset in `this` object.
  Set &intersect(const Set &a, const Set &b);

  // Compute a new set containing the intersection of `this` set with
  // the `other`.
  Set intersected(const Set &other) const &;
  Set intersected(const Set &other) &&;

  // Compute the union of `this` set with the `other`.  If there is
  // enough room for the resulting set, then on return, `this` set
  // will contain the resulting superset elements, and a reference to
  // `this` object is returned; otherwise, an assertion is triggered.
  //
  // Any overlap is resolved by taking elements from `this` set; if
  // the equality relation defined by `operator<` gives you a way to
  // tell set members that compare equal apart later, you will find
  // that only the one originally in `this` is still present.
  Set &extend(const Set &other);

  // Compute the union of sets `a` and `b`, storing the result in
  // `this` object.  If there is enough room for the resulting set,
  // then on return, `this` set will contain the resulting superset
  // elements, and a reference to `this` object is returned;
  // otherwise, an assertion is triggered.
  //
  // Any overlap is resolved by taking elements from `this` set; if
  // the ordering defined by `operator<` gives you a way to tell set
  // members that compare equal apart later, you will find that only
  // the one originally in `this` is still present.
  Set &extend(const Set &a, const Set &b);

  // Compute a new set containing the union of `this` set with the
  // `other`.  If there is enough room for the resulting set, then on
  // return, `this` set will contain the resulting superset elements,
  // and a new object containing the union is returned; otherwise, an
  // assertion is triggered
  //
  // Any overlap is resolved by taking elements from `this` set; if
  // `compare_f` indicates that an element in `this` set is equal to
  // one in the `other`, but you have a way to tell them apart later,
  // you will find that only the one originally in `this` is present.
  Set extended(const Set &other) const &;

  // Compute the difference between `this` set and the `other`.  On
  // return, `this` object will contain only elements that were
  // members of `this` object but not the `other`.  A reference to
  // `this` object is returned.
  Set &difference(const Set &other);

  // Compute the difference between sets `a` and `b`, storing the
  // result in `this` object.  On return, `this` object will contain
  // only elements that are members of `a` but not `b`.  A reference
  // to `this` object is returned.
  Set &difference(const Set &a, const Set &b);

  // Compute a new set containing the difference between `this` set
  // and the `other`.  The new set will contain only elements that
  // were members of `this` object but not the `other`.
  Set differenced(const Set &other) const &;
  Set differenced(const Set &other) &&;

  // Add a new member `elem` to `this` set.  If there is no room, an
  // assertion is triggered; otherwise, `this` set now contains
  // `elem`, and a reference to `this` set is returned.
  Set &add(const T &elem);

  // Add a new member `elem` to `this` set.  If there is no room, an
  // assertion is triggered.  If `this` set already contains `elem`,
  // an empty object is returned.  Otherwise, `this` set now contains
  // `elem`, and a reference to `this` set is returned.
  optional<Set &> add_missing(const T &elem);

  // Remove `elem` from `this` set if it's a member.  A reference to
  // `this` set is returned, at which point `this` set no longer
  // contains `elem`.
  Set &drop(const T &elem);

  // Remove `elem` from `this` set.  If `this` set does not contain
  // `elem`, an empty object is returned (this is the only difference
  // between `Set::drop_present()` and `Set::drop()`;
  // otherwise, a reference to `this` set is returned, and `this` set
  // no longer contains `elem`.
  optional<Set &> drop_present(const T &elem);

  // Return a new Set equivalent to the union of the contents of
  // `this` set and the set containing only `elem`.  If `this` set
  // already contains `elem` or there is no room to add more elements,
  // then the returned set will be a copy of `this` set.
  Set with(const T &elem) const &;
  Set with(const T &elem) &&;

  // Return a new Set that contains everything in `this` set
  // except `elem`.  If `elem` is not in `this` set to begin with,
  // then the returned set will be a copy of `this` set.
  Set without(const T &elem) const &;
  Set without(const T &elem) &&;

  // Given a selection function `select_f`, remove from `this` set all
  // elements for which `select_f` returns `false` (in other words,
  // keep only elements for which `select_f` returns `true`).  Returns
  // a reference to `this` object.
  Set &filter(bool (*select_f)(const T &elem));

  // Given a selection function `select_f`, remove from `this` set all
  // elements for which `select_f` returns `false` (in other words,
  // keep only elements for which `select_f` returns `true`).  Returns
  // the new number of elements in the set after all removals have
  // taken place.  Every time `select_f` is called, the given argument
  // `helper` will be passed to it.  Returns a reference to `this`
  // object.
  //
  // Please check out FUNCTIONS_AS_ARGUMENTS in the top-level
  // documentation for this class if you are getting compiler errors
  // trying to use this function.
  template <typename C>
  Set &filter(bool (*select_f)(const T &elem, C helper), C helper);

  // Return a new Set containing all elements of `this` set for
  // which the selection function `select_f` returns `true`.
  Set filtered(bool (*select_f)(const T &elem)) const &;
  Set filtered(bool (*select_f)(const T &elem)) &&;

  // Return a new Set containing all elements of `this` set for
  // which the selection function `select_f` returns `true`.  Every
  // time `select_f` is called, the given argument `helper` will be
  // passed to it.
  //
  // Please check out FUNCTIONS_AS_ARGUMENTS in the top-level
  // documentation for this class if you are getting compiler errors
  // trying to use this function.
  template <typename C>
  Set filtered(bool (*select_f)(const T &elem, C helper), C helper) const &;
  template <typename C>
  Set filtered(bool (*select_f)(const T &elem, C helper), C helper) &&;

  // Some caution must be used here - when used in a range-based for loop (the
  // usual case), this can't be used recursively. A FilteredMapView cannot be
  // used on top of another FilteredMapView and because it keeps a reference to
  // the input map as a const & (in order to avoid the CPU load of a hard copy),
  // if the input map is a temporary the behavior is undefined (see
  // http://en.cppreference.com/w/cpp/language/range-for for details).
  template <typename C>
  FilteredSetView<T, C, max_set_size> filtered_view(
      bool (*select_f)(const T &elem, const C &helper), const C &helper) const {
    return FilteredSetView<T, C, max_set_size>(select_f, helper, *this);
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
  // transform all elements in `this` set.  (This function is like
  // `std::transform` from the STL and "map" in functional programming
  // languages.)  A reference to `this` set is returned.
  Set &transform(T (*transform_f)(const T &elem));

  // Given an element-wise transformation function `transform_f`,
  // transform all elements in `this` set.  (This function is like
  // `std::transform` from the STL and "map" in functional programming
  // languages; Haskellers will recognize it as more like "traverse".)
  // Every time `transform_f` is called, the given argument `helper`
  // will be passed to it.  A reference to `this` set is returned.
  //
  // Please check out FUNCTIONS_AS_ARGUMENTS in the top-level
  // documentation for this class if you are getting compiler errors
  // trying to use this function.
  template <typename C>
  Set &transform(T (*transform_f)(const T &elem, C helper), C helper);

  // Given an element-wise transformation function `transform_f`,
  // return a new set containing the transformed elements of `this`
  // set.  (This function is like `std::transform` from the STL and
  // "map" in functional programming languages.)
  Set transformed(T (*transform_f)(const T &elem)) const &;
  Set transformed(T (*transform_f)(const T &elem)) &&;

  // Given an element-wise transformation function `transform_f`,
  // transform all elements in `this` set.  (This function is like
  // `std::transform` from the STL and "map" in functional programming
  // languages; Haskellers will recognize it as more like "traverse".)
  // Every time `transform_f` is called, the given argument `helper`
  // will be passed to it.
  //
  // Please check out FUNCTIONS_AS_ARGUMENTS in the top-level
  // documentation for this class if you are getting compiler errors
  // trying to use this function.
  template <typename C>
  Set transformed(T (*transform_f)(const T &elem, C helper), C helper) const &;
  template <typename C>
  Set transformed(T (*transform_f)(const T &elem, C helper), C helper) &&;

  // Given an element-wise transformation function `transform_f` which
  // converts each element of type `T` in `this` set to a new element
  // of a different type `S`, transform all elements in `this` set and
  // return the new set.  (This function is like `std::transform` from
  // the STL and "map" in functional programming languages.)
  //
  // Please check out FUNCTIONS_AS_ARGUMENTS in the top-level
  // documentation for this class if you are getting compiler errors
  // trying to use this function.
  template <typename S>
  Set<S, max_set_size> transformed(S (*transform_f)(const T &elem)) const;

  // Given an element-wise transformation function `transform_f` which
  // converts each element of type `T` in `this` set to a new element
  // of a different type `S`, transform all elements in `this` set and
  // return the new set.  (This function is like `std::transform` from
  // the STL and "map" in functional programming languages; Haskellers
  // will recognize it as more like "traverse".)  Every time
  // `transform_f` is called, the given argument `helper` will be
  // passed to it.
  //
  // Please check out FUNCTIONS_AS_ARGUMENTS in the top-level
  // documentation for this class if you are getting compiler errors
  // trying to use this function.
  template <typename S, typename C>
  Set<S, max_set_size> transformed(S (*transform_f)(const T &elem, C helper),
                                   C helper) const;

  // Given an element-wise transformation function `transform_f` which
  // converts each element of type `T` in `this` set to a new element
  // of a different type `S` or returns an empty object if such a
  // conversion is not possible, transform all elements in `this` set
  // and return the new set.  (This function is like `std::transform`
  // from the STL and "map" in functional programming languages.)
  //
  // Please check out FUNCTIONS_AS_ARGUMENTS in the top-level
  // documentation for this class if you are getting compiler errors
  // trying to use this function.
  template <typename S>
  Set<S, max_set_size> transformed(
      optional<S> (*transform_f)(const T &elem)) const;

  // Given an element-wise transformation function `transform_f` which
  // converts each element of type `T` in `this` set to a new element
  // of a different type `S` or returns an empty object if such a
  // conversion is not possible, transform all elements in `this` set
  // and return the new set.  (This function is like `std::transform`
  // from the STL and "map" in functional programming languages.)
  // Every time `transform_f` is called, the given argument `helper`
  // will be passed to it.
  //
  // Please check out FUNCTIONS_AS_ARGUMENTS in the top-level
  // documentation for this class if you are getting compiler errors
  // trying to use this function.
  template <typename S, typename C>
  Set<S, max_set_size> transformed(optional<S> (*transform_f)(const T &elem,
                                                              C helper),
                                   C helper) const;

  // Given an element-wise operation `traverse_f` which generates a
  // new `accumulator` value of type C when applied to each element
  // and the current `accumulator` value along with a pointer to an
  // initial accumulator value, traverse the members of the set,
  // visiting each once, and generate a final accumulated (or
  // "summary") value.  The summary value is returned.
  //
  // No guarantees are provided about the order in which the elements
  // of the set are visited.
  //
  // In a previous version of this code, there was a set datatype for
  // organizing signal IDs which was equipped with a member function
  // `unique_sats_size()`.  Here is an example of how you might
  // calculate the number of sats in a set of signals using
  // `traverse()`.  Note that the accumulator can be passed by value.
  //
  //     const auto accumulate_sats = [](const gnss_signal_t &sid,
  //                                     Set<s16> sats) {
  //       sats.add(sid.sat);
  //       return sats;
  //     };
  //
  //     const s32 num_unique_sats =
  //         my_signals.traverse<Set<s16>>(accumulate_sats,
  //                                       Set<s16>()).size();
  template <typename C>
  C traverse(C (*traverse_f)(const T &elem, C accumulator),
             C accumulator) const;

  // Given an element-wise selection function `select_f`, which
  // returns a boolean given a set element, divide `this` set into a
  // `std::tuple` of new sets.  The first (left-hand) set returned
  // contains all the elements of `this` for which `select_f` returns
  // `true`; the second (right-hand) set returned contains all the
  // remaining elements.
  std::tuple<Set, Set> partitioned(bool (*select_f)(const T &elem)) const;

  // Given an element-wise selection function `select_f`, which
  // returns a boolean given a set element and a helper argument of
  // type `C`, divide `this` set into a `std::tuple` of new sets.  The
  // first (left-hand) set returned contains all the elements of
  // `this` for which `select_f` returns `true`; the second
  // (right-hand) set returned contains all the remaining elements.
  // Every time `select_f` is called, the given argument `helper` will
  // be passed to it.
  //
  // Please check out FUNCTIONS_AS_ARGUMENTS in the top-level
  // documentation for this class if you are getting compiler errors
  // trying to use this function.
  template <typename C>
  std::tuple<Set, Set> partitioned(bool (*select_f)(const T &elem, C helper),
                                   C helper) const;

  // Return a const iterator pointing at the first of `this` set's
  // members.
  const T *cbegin() const { return &at(0); }

  // Return a const iterator pointing past the last of `this` set's
  // members.
  const T *cend() const { return &at(n_); }

  // It's not safe to provide non-const iterators for this class (we'd
  // need to re-sort constantly to be safe), but range-based for
  // syntax uses `begin()` and `end()` even if the loop variable is
  // const due to the way C++ deals with overloading.  So we return
  // const pointers anyway and let the user deal with the error if the
  // const is left off the loop variable.  Sorry, but that's the way
  // it is.
  //
  // Return a const iterator pointing at the first of `this` set's
  // members.
  const T *begin() const { return cbegin(); }
  // Return a const iterator pointing past the last of `this` set's
  // members.
  const T *end() const { return cend(); }

  using EigenArrayOfSet =
      Eigen::Array<T, Eigen::Dynamic, 1, 0, max_set_size, 1>;

  // Return an Eigen `Array` (NB: not a `Matrix` containing the
  // members of the set.  This is used solely because it knows how
  // many elements it contains and therefore makes for a clean
  // interface when returned by value.).
  EigenArrayOfSet get_contents() const {
    return Eigen::Map<const EigenArrayOfSet>(&at(0), n_);
  }

  // Find `elem` in `this` set and return the offset into member
  // storage at which `elem` is stored, or an empty object if the
  // member does not exist.  This can only be used with `lookup()`.
  optional<s32> find(const T &elem) const;

  // Directly get the member of `this` set that is stored at offset
  // `index` in the internal array, or an empty object if the index is
  // invalid.
  optional<T> lookup(s32 index) const;

 private:
  optional<Set &> extend_impl(const Set &a, const Set &b);

  // Given an element-wise transformation function `transform_f`,
  // transform all elements in `this` set.  (This function is like
  // `std::transform` from the STL and "map" in functional programming
  // languages; Haskellers will recognize it as more like "traverse".)
  // The caller is responsible for ensuring that `transform_f` does
  // not change the ordering of the elements of the set under the
  // original total ordering with which the set was constructed.
  //
  // This function has "unsafe" in the name for a reason.  If you
  // aren't clear what is going on, use `transform` instead.  The
  // point of this function is to avoid an extra sort.
  void unsafe_transform(T (*transform_f)(const T &elem));

  // Given an element-wise transformation function `transform_f`,
  // transform all elements in `this` set.  (This function is like
  // `std::transform` from the STL and "map" in functional programming
  // languages; Haskellers will recognize it as more like "traverse".)
  // The caller is responsible for ensuring that `transform_f` does
  // not change the ordering of the elements of the set under the
  // original total ordering with which the set was constructed.
  // Every time `transform_f` is called, the given argument `helper`
  // will be passed to it.
  //
  // Note that if `transform_f` is a lambda, you will likely need to
  // pass the template argument for the helper type, e.g.
  //
  //     myset.unsafe_transform<const foo &>(morph, extras);
  //
  // This function has "unsafe" in the name for a reason.  If you
  // aren't clear what is going on, use `transform` instead.  The
  // point of this function is to avoid an extra sort.
  template <typename C>
  void unsafe_transform(T (*transform_f)(const T &elem, C helper), C helper);

  // This function sorts the contents of this set.
  void sort_();

  // Helper function for `drop()`, `without()` and `filter()`.
  void drop_index_(T *iter);

  // Helper function for `filter()` and `filtered()`: this redirects
  // the one-argument version through the two-argument version in a
  // way that allows us to implement the body only once.
  template <typename R>
  static R call_reroute_(const T &elem, R (*select_f)(const T &elem));

  T &at(s32 index) & {
    return elems_[index].value;  // NOLINT
  };
  const T &at(s32 index) const & { return elems_[index].value; };  // NOLINT

  union Storage {
    Storage() : dummy(){};
    uint8_t dummy;
    T value;
  };

  // The number of elements currently stored.
  s32 n_{0};

  // INVARIANT: These are always stored in ascending order.  This
  // allows for fast set operations and makes it easier to traverse
  // them.
  //
  // They are also supposed to be unique.  Right now this is just
  // checked with some `assert()` statements in the constructor.
  //
  // A `std::array<T, max_set_size>` will default-initialize all of
  // its arguments on construction.  We don't want to pay that
  // performance cost; any time we copy or initialize one of these, we
  // only want to pay for the storage that we're using.
  Storage elems_[max_set_size];
};

template <typename T, typename C, s32 max_set_size>
struct Set_iter {
 public:
  Set_iter(s32 start_index, bool (*select_f)(const T &elem, const C &helper),
           const C &helper, const Set<T, max_set_size> &set);
  const Set_iter &operator++();
  const T &operator*() const;
  bool operator!=(const Set_iter<T, C, max_set_size> &rhs) const;

 private:
  const T *elem_;
  bool (*select_f_)(const T &elem, const C &helper);
  const Set<T, max_set_size> &set_;
  const C &helper_;
};

template <typename T, typename C, s32 max_set_size>
struct FilteredSetView {
 public:
  FilteredSetView(bool (*select_f)(const T &elem, const C &helper),
                  const C &helper, const Set<T, max_set_size> &set);
  const Set_iter<T, C, max_set_size> begin() const;
  const Set_iter<T, C, max_set_size> end() const;

 private:
  bool (*select_f_)(const T &elem, const C &helper);
  const Set<T, max_set_size> &set_;
  C helper_;
};

template <typename T, typename C, s32 max_set_size>
Set_iter<T, C, max_set_size>::Set_iter(
    s32 start_index, bool (*select_f)(const T &elem, const C &helper),
    const C &helper, const Set<T, max_set_size> &set)
    : elem_(set.begin() + start_index),
      select_f_(select_f),
      set_(set),
      helper_(helper) {
  assert(start_index >= 0 && start_index <= set.size());
  if (elem_ != set_.cend() && !select_f_(*elem_, helper_)) {
    operator++();
  }
}

template <typename T, typename C, s32 max_set_size>
const Set_iter<T, C, max_set_size> &Set_iter<T, C, max_set_size>::operator++() {
  ++elem_;
  while (elem_ != set_.cend() && !select_f_(*elem_, helper_)) {
    ++elem_;
  }
  return *this;
}

template <typename T, typename C, s32 max_set_size>
const T &Set_iter<T, C, max_set_size>::operator*() const {
  return *elem_;
}

template <typename T, typename C, s32 max_set_size>
bool Set_iter<T, C, max_set_size>::operator!=(
    const Set_iter<T, C, max_set_size> &rhs) const {
  // Do we need to consider the other members?
  return elem_ != rhs.elem_;
};

template <typename T, typename C, s32 max_set_size>
FilteredSetView<T, C, max_set_size>::FilteredSetView(
    bool (*select_f)(const T &elem, const C &helper), const C &helper,
    const Set<T, max_set_size> &set)
    : select_f_(select_f), set_(set), helper_(helper) {}

template <typename T, typename C, s32 max_set_size>
const Set_iter<T, C, max_set_size> FilteredSetView<T, C, max_set_size>::begin()
    const {
  return Set_iter<T, C, max_set_size>(0, select_f_, helper_, set_);
}

template <typename T, typename C, s32 max_set_size>
const Set_iter<T, C, max_set_size> FilteredSetView<T, C, max_set_size>::end()
    const {
  return Set_iter<T, C, max_set_size>(set_.size(), select_f_, helper_, set_);
}

template <typename T, s32 max_set_size>
Set<T, max_set_size>::Set(s32 n, const T elems[]) : n_(n), elems_() {
  assert(n_ >= 0 && n_ <= max_size());
  std::copy(elems, elems + n_, &at(0));
  sort_();
}

template <typename T, s32 max_set_size>
Set<T, max_set_size>::Set(T elem) : elems_() {
  add(elem);
}

template <typename T, s32 max_set_size>
Set<T, max_set_size>::Set(std::initializer_list<T> initlist)
    : n_(static_cast<s32>(initlist.size())), elems_() {
  assert(n_ >= 0 && n_ <= max_size());
  std::copy(initlist.begin(), initlist.end(), &at(0));
  sort_();
}

template <typename T, s32 max_set_size>
template <typename ElemIter>
Set<T, max_set_size>::Set(ElemIter elem_begin, ElemIter elem_end)
    : n_(), elems_() {
  assert(elem_end - elem_begin <= max_set_size);
  n_ = static_cast<s32>(elem_end - elem_begin);
  std::copy(elem_begin, elem_end, &at(0));
  sort_();
}

template <typename T, s32 max_set_size>
void Set<T, max_set_size>::sort_() {
  std::stable_sort(&at(0), &at(n_));
  n_ = static_cast<s32>(std::distance(&at(0), std::unique(&at(0), &at(n_))));
  assert(n_ >= 0 && n_ <= max_size());
}

template <typename T, s32 max_set_size>
template <s32 other_max_set_size>
bool Set<T, max_set_size>::operator==(
    const Set<T, other_max_set_size> &other) const {
  if (n_ != other.n_) {
    return false;
  }
  for (s32 i = 0; i < n_; i++) {
    if (at(i) < other.at(i) || other.at(i) < at(i)) {
      return false;
    }
  }

  return true;
};

template <typename T, s32 max_set_size>
Set<T, max_set_size> &Set<T, max_set_size>::intersect(const Set &a,
                                                      const Set &b) {
  const auto choose_left =
      [](const T &ea, const T &eb __attribute__((unused)),
         void *helper __attribute__((unused))) -> optional<T> { return ea; };

  n_ = static_cast<s32>(internal::intersect_on<T, void *>(
      choose_left, &at(0), static_cast<std::size_t>(max_size()), a.cbegin(),
      static_cast<std::size_t>(a.size()), b.cbegin(),
      static_cast<std::size_t>(b.size()), nullptr));
  return *this;
}

template <typename T, s32 max_set_size>
Set<T, max_set_size> &Set<T, max_set_size>::intersect(const Set &other) {
  // See `intersect(a, b)` for a discussion of why aliasing is OK
  // here.
  return intersect(*this, other);
}

// TODO(MP): In C++, the overloading never stops.  If `other` is an
// rvalue, could we steal it?
template <typename T, s32 max_set_size>
Set<T, max_set_size> Set<T, max_set_size>::intersected(
    const Set &other) const & {
  Set ret;
  ret.intersect(*this, other);
  return ret;
}

// NOTE ON RVALUE REFERENCE OVERLOADS
//
// Normally the "participle functions" simply return a new object on
// the stack.  This is somewhat inefficient, because it typically
// involves copying an entire internal array, but it makes for
// high-level code that is easy to get right.  At the time of writing,
// it's definitely better to trade a bit of CPU time for ease of
// understanding.
//
// There are few nice things about C++, and most of them appeared in
// C++11.  Rvalue references, which enable true move semantics, are
// one of these; in this case, they turn out to be the key ingredient
// in the concurrent possession and consumption of cake.  For these
// participle functions, if we know `this` points to an rvalue, then
// there is no point copying and creating a new object, because nobody
// can ever use `this` object again anyway.  In such situations, we
// can instead modify `this` object in place and return another
// rvalue-reference.  Combined with appropriate use of `std::move()`,
// this should avoid making more copies than necessary.  A chain of
// function calls like
//
//    out_set = foo.filtered(my_filter)
//                 .transformed<Bar>(enswizzle, some_random_bar)
//                 .intersected(butts);
//
// should perform only one copy of the data in `foo`, because
// `filtered()` gets called on foo, whose `*this` is a `const Set &`.
// The set `filtered()` returns is an rvalue, and from there on out,
// everything should stay on the `&&` path all the way through the
// move-assignment operator of `out_set`, which should accept an
// rvalue ref and be able to mutate things in place.  In other words,
// in the generated assembly, the data should get copied directly from
// `foo`'s internal array to `out_set`'s internal array, and then the
// filter, transform and intersection routines run in place and in
// sequence.
//
// What can't we pull off with this?
//
//    * Our internal union algorithm can't union in-place, so there's
//      no point doing this for `extended()`.
//
//    * The types are ambiguous if we try to define
//      e.g. `intersected(Set &&other) const &;`, so we don't cover
//      that case even though it could reduce copying in some
//      situations.
template <typename T, s32 max_set_size>
Set<T, max_set_size> Set<T, max_set_size>::intersected(const Set &other) && {
  intersect(*this, other);
  return std::move(*this);
}

template <typename T, s32 max_set_size>
Set<T, max_set_size> &Set<T, max_set_size>::difference(const Set &a,
                                                       const Set &b) {
  n_ = static_cast<s32>(
      internal::difference(&at(0), static_cast<std::size_t>(max_size()),
                           a.cbegin(), static_cast<std::size_t>(a.size()),
                           b.cbegin(), static_cast<std::size_t>(b.size())));

  return *this;
}

template <typename T, s32 max_set_size>
Set<T, max_set_size> &Set<T, max_set_size>::difference(const Set &other) {
  // See `difference(a, b)` for a discussion of why aliasing is OK
  // here.
  return difference(*this, other);
}

template <typename T, s32 max_set_size>
Set<T, max_set_size> Set<T, max_set_size>::differenced(
    const Set &other) const & {
  Set ret;
  ret.difference(*this, other);
  return ret;
}

template <typename T, s32 max_set_size>
Set<T, max_set_size> Set<T, max_set_size>::differenced(const Set &other) && {
  difference(*this, other);
  return std::move(*this);
}

template <typename T, s32 max_set_size>
optional<Set<T, max_set_size> &> Set<T, max_set_size>::extend_impl(
    const Set &a, const Set &b) {
  const auto choose_left = [](const T &ea, const T &eb __attribute__((unused)),
                              void *helper
                              __attribute__((unused))) { return ea; };
  const auto extent = internal::extend_on<T, void *>(
      choose_left, &at(0), static_cast<std::size_t>(max_size()), a.cbegin(),
      static_cast<std::size_t>(a.size()), b.cbegin(),
      static_cast<std::size_t>(b.size()), nullptr);
  if (!extent.has_value()) {
    return {};
  }

  n_ = static_cast<s32>(*extent);
  return *this;
}

template <typename T, s32 max_set_size>
Set<T, max_set_size> &Set<T, max_set_size>::extend(const Set &a, const Set &b) {
  assert(extend_impl(a, b).has_value());
  return *this;
}

template <typename T, s32 max_set_size>
Set<T, max_set_size> &Set<T, max_set_size>::extend(const Set &other) {
  // Unlike the other set operation member functions, aliasing `a`
  // with `*this` won't work here.  Just allocate a temporary and
  // reuse `union(a, b)` safely.
  Set temp;
  *this = temp.extend(*this, other);
  return *this;
}

// There is no point in doing an rvalue-reference overload for
// `extended()`, because the implementation of the set-union algorithm
// cannot currently perform the union in-place and needs to make an
// extra copy anyway.  Sorry.
template <typename T, s32 max_set_size>
Set<T, max_set_size> Set<T, max_set_size>::extended(const Set &other) const & {
  Set ret;
  const auto extent = ret.extend_impl(*this, other);
  assert(extent.has_value());
  return ret;
}

template <typename T, s32 max_set_size>
bool Set<T, max_set_size>::contains(const T &elem) const {
  return std::binary_search(&at(0), &at(n_), elem);
}

template <typename T, s32 max_set_size>
optional<s32> Set<T, max_set_size>::find(const T &elem) const {
  auto iter = std::lower_bound(&at(0), &at(n_), elem);
  if ((iter != &at(n_)) && !(elem < *iter) && !(*iter < elem)) {
    return static_cast<s32>(iter - &at(0));
  }

  return {};
}

template <typename T, s32 max_set_size>
optional<T> Set<T, max_set_size>::lookup(s32 index) const {
  if (index < 0 || index >= size()) {
    return {};
  }

  return at(index);
}

template <typename T, s32 max_set_size>
optional<Set<T, max_set_size> &> Set<T, max_set_size>::add_missing(
    const T &elem) {
  // Calculate the appropriate spot.
  const auto iter = std::lower_bound(&at(0), &at(n_), elem);

  if (iter < &at(n_) && !(*iter < elem || elem < *iter)) {
    // This element is already present.
    return {};
  }

  assert(size() < max_size());

  // Shift everything greater than this new element up in the array to
  // make room.
  std::copy_backward(iter, &at(n_), &at(n_ + 1));

  // Fill in the new element and increment the size counter.
  *iter = elem;
  n_++;

  return *this;
}

template <typename T, s32 max_set_size>
Set<T, max_set_size> &Set<T, max_set_size>::add(const T &elem) {
  add_missing(elem);
  return *this;
}

template <typename T, s32 max_set_size>
void Set<T, max_set_size>::drop_index_(T *iter) {
  assert(iter < &at(n_));

  if (iter < &at(n_ - 1)) {
    // We're not at the last element in the array, so we have to do
    // some work.  Shift everything greater than this new element down
    // in the array,
    // overwriting it.
    std::copy(iter + 1, &at(n_), iter);
  }

  // Decrease size counter.  If `iter == elems_ + n_ - 1`, the element
  // to be dropped is the last one in the array, so we don't need to
  // copy anything.
  n_--;
}

template <typename T, s32 max_set_size>
Set<T, max_set_size> &Set<T, max_set_size>::drop(const T &elem) {
  if (contains(elem)) {
    // Calculate the appropriate spot.  We already know this element is
    // present in the array thanks to the check above, so the lower
    // bound we find should be the exact offset from which to delete.
    const auto iter = std::lower_bound(&at(0), &at(n_), elem);
    drop_index_(iter);
  }

  return *this;
}

template <typename T, s32 max_set_size>
optional<Set<T, max_set_size> &> Set<T, max_set_size>::drop_present(
    const T &elem) {
  if (contains(elem)) {
    return drop(elem);
  }

  return {};
}

template <typename T, s32 max_set_size>
Set<T, max_set_size> Set<T, max_set_size>::with(const T &elem) const & {
  Set<T, max_set_size> temp(*this);
  temp.add(elem);
  return temp;
}

template <typename T, s32 max_set_size>
Set<T, max_set_size> Set<T, max_set_size>::with(const T &elem) && {
  add(elem);
  return std::move(*this);
}

template <typename T, s32 max_set_size>
Set<T, max_set_size> Set<T, max_set_size>::without(const T &elem) const & {
  Set<T, max_set_size> temp(*this);
  temp.drop(elem);
  return temp;
}

template <typename T, s32 max_set_size>
Set<T, max_set_size> Set<T, max_set_size>::without(const T &elem) && {
  drop(elem);
  return std::move(*this);
}

template <typename T, s32 max_set_size>
template <typename R>
R Set<T, max_set_size>::call_reroute_(const T &elem,
                                      R (*select_f)(const T &elem)) {
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
  // want one argument, though, so we pass the real `select_f` from
  // the single-argument function as the helper argument to
  // `call_reroute_()`, then invoke it on the element in question.
  return select_f(elem);
}

template <typename T, s32 max_set_size>
Set<T, max_set_size> &Set<T, max_set_size>::filter(
    bool (*select_f)(const T &elem)) {
  return filter<bool (*)(const T &)>(call_reroute_, select_f);
}

// NOTE: This could be implemented with a void pointer to prevent
// the template stuff generating new versions, as in C, but the user
// would have to cast, and types would not be checked.
template <typename T, s32 max_set_size>
template <typename C>
Set<T, max_set_size> &Set<T, max_set_size>::filter(
    bool (*select_f)(const T &elem, C helper), C helper) {
  // Identify the indices to drop
  s32 idxs[max_set_size];
  s32 idx_count = 0;
  for (s32 i = 0; i < n_; i++) {
    // If the selection function returns false, away with it!
    if (!select_f(at(i), helper)) {
      idxs[idx_count++] = i;
    }
  }

  // Drop all the marked indices.  We have to drop them from the top
  // down, or else our indices will be invalidated.
  for (s32 i = idx_count - 1; i >= 0; i--) {
    drop_index_(&at(idxs[i]));
  }

  return *this;
}

template <typename T, s32 max_set_size>
Set<T, max_set_size> Set<T, max_set_size>::filtered(
    bool (*select_f)(const T &elem)) const & {
  return filtered<bool (*)(const T &)>(call_reroute_, select_f);
}

template <typename T, s32 max_set_size>
Set<T, max_set_size> Set<T, max_set_size>::filtered(
    bool (*select_f)(const T &elem)) && {
  return std::move(filtered<bool (*)(const T &)>(call_reroute_, select_f));
}

// NOTE: This could be implemented with a void pointer to prevent the
// template stuff generating new versions, but the user would have to
// cast, and types would not be checked.
template <typename T, s32 max_set_size>
template <typename C>
Set<T, max_set_size> Set<T, max_set_size>::filtered(
    bool (*select_f)(const T &elem, C helper), C helper) const & {
  Set<T, max_set_size> ret;
  for (const auto &elem : *this) {
    if (select_f(elem, helper)) {
      // This is implemented by successively adding rather than just
      // wrapping `filter()` because it avoids copying as much.
      ret.add(elem);
    }
  }

  return ret;
}

template <typename T, s32 max_set_size>
template <typename C>
Set<T, max_set_size> Set<T, max_set_size>::filtered(
    bool (*select_f)(const T &elem, C helper), C helper) && {
  filter<C>(select_f, helper);
  return std::move(*this);
}

template <typename T, s32 max_set_size>
template <typename C>
Set<T, max_set_size> &Set<T, max_set_size>::transform(
    T (*transform_f)(const T &elem, C helper), C helper) {
  unsafe_transform(transform_f, helper);

  sort_();
  return *this;
}

template <typename T, s32 max_set_size>
Set<T, max_set_size> &Set<T, max_set_size>::transform(
    T (*transform_f)(const T &elem)) {
  return transform(call_reroute_, transform_f);
}

template <typename T, s32 max_set_size>
template <typename C>
Set<T, max_set_size> Set<T, max_set_size>::transformed(
    T (*transform_f)(const T &elem, C helper), C helper) const & {
  Set ret(*this);
  ret.transform(transform_f, helper);
  return ret;
}

template <typename T, s32 max_set_size>
template <typename C>
Set<T, max_set_size> Set<T, max_set_size>::transformed(
    T (*transform_f)(const T &elem, C helper), C helper) && {
  transform(transform_f, helper);
  return std::move(*this);
}

template <typename T, s32 max_set_size>
Set<T, max_set_size> Set<T, max_set_size>::transformed(
    T (*transform_f)(const T &elem)) const & {
  return transformed(call_reroute_, transform_f);
}

template <typename T, s32 max_set_size>
Set<T, max_set_size> Set<T, max_set_size>::transformed(
    T (*transform_f)(const T &elem)) && {
  return std::move(transformed(call_reroute_, transform_f));
}

template <typename T, s32 max_set_size>
template <typename C>
void Set<T, max_set_size>::unsafe_transform(T (*transform_f)(const T &elem,
                                                             C helper),
                                            C helper) {
  for (s32 i = 0; i < n_; i++) {
    const T temp = transform_f(at(i), helper);
    at(i) = temp;
  }
}

template <typename T, s32 max_set_size>
void Set<T, max_set_size>::unsafe_transform(T (*transform_f)(const T &elem)) {
  unsafe_transform(call_reroute_, transform_f);
}

template <typename T, s32 max_set_size>
template <typename S, typename C>
Set<S, max_set_size> Set<T, max_set_size>::transformed(
    optional<S> (*transform_f)(const T &elem, C helper), C helper) const {
  Set<S, max_set_size> ret;
  for (const auto &elem : *this) {
    const optional<S> selem = transform_f(elem, helper);
    if (selem) {
      ret.add(selem);
    }
  }

  return ret;
}

template <typename T, s32 max_set_size>
template <typename S>
Set<S, max_set_size> Set<T, max_set_size>::transformed(
    optional<S> (*transform_f)(const T &elem)) const {
  return transformed<S>(call_reroute_, transform_f);
}

template <typename T, s32 max_set_size>
template <typename S, typename C>
Set<S, max_set_size> Set<T, max_set_size>::transformed(
    S (*transform_f)(const T &elem, C helper), C helper) const {
  Set<S, max_set_size> ret;
  for (const auto &elem : *this) {
    ret.add(transform_f(elem, helper));
  }

  return ret;
}

template <typename T, s32 max_set_size>
template <typename S>
Set<S, max_set_size> Set<T, max_set_size>::transformed(
    S (*transform_f)(const T &elem)) const {
  return transformed<S>(call_reroute_, transform_f);
}

template <typename T, s32 max_set_size>
template <s32 new_max_set_size>
Set<T, new_max_set_size> Set<T, max_set_size>::grown() const {
  static_assert(new_max_set_size >= max_set_size,
                "Set<T, max_set_size>::grown(): new maximum size is smaller "
                "than current");
  return Set<T, new_max_set_size>(n_, &at(0));
}

template <typename T, s32 max_set_size>
template <s32 new_max_set_size>
optional<Set<T, new_max_set_size>> Set<T, max_set_size>::shrunk() const {
  assert(new_max_set_size >= size());
  return Set<T, new_max_set_size>(n_, &at(0));
}

template <typename T, s32 max_set_size>
template <typename C>
C Set<T, max_set_size>::traverse(C (*traverse_f)(const T &elem, C accumulator),
                                 C accumulator) const {
  for (const auto &elem : *this) {
    accumulator = traverse_f(elem, accumulator);
  }

  return accumulator;
}

template <typename T, s32 max_set_size>
template <typename C>
std::tuple<Set<T, max_set_size>, Set<T, max_set_size>>
Set<T, max_set_size>::partitioned(bool (*select_f)(const T &elem, C helper),
                                  C helper) const {
  Set pass, fail;
  for (const auto &elem : *this) {
    if (select_f(elem, helper)) {
      pass.add(elem);
    } else {
      fail.add(elem);
    }
  }
  return std::make_tuple(pass, fail);
}

template <typename T, s32 max_set_size>
std::tuple<Set<T, max_set_size>, Set<T, max_set_size>>
Set<T, max_set_size>::partitioned(bool (*select_f)(const T &elem)) const {
  return partition(call_reroute_, select_f);
}

}  // namespace containers

}  // namespace pvt_common

#endif  // LIBSWIFTNAV_PVT_ENGINE_CONTAINERS_SET_H
