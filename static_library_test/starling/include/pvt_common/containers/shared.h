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

#ifndef LIBSWIFTNAV_PVT_ENGINE_CONTAINERS_SHARED_H
#define LIBSWIFTNAV_PVT_ENGINE_CONTAINERS_SHARED_H

#include <pvt_engine/optional.h>
#include <algorithm>
#include <cassert>
#include <tuple>

namespace pvt_common {

namespace containers {

namespace internal {

template <typename T>
void assert_valid(std::size_t n, std::size_t max_size_out, T elems_out[]) {
  // We've come up with a valid number of output elements
  assert(n >= 0);
  assert(n <= max_size_out);

  // The resulting sequence is sorted
  assert(std::is_sorted(elems_out, elems_out + n));

  // The resulting sequence doesn't contain any duplicate elements.
  assert(n == static_cast<std::size_t>(std::distance(
                  elems_out, std::unique(elems_out, elems_out + n))));
}

// Compute the set-intersection of two sorted arrays containing
// elements of type `T`, using the function `merge_f` to generate the
// appropriate output for any overlapping elements.  `merge_f`
// receives the helper value `helper` on each call.  Aliasing is
// permitted: `elems_out` may be the same as `a` or `b` (or both).
// See the internal comments for discussion of aliasing.
template <typename T, typename C>
std::size_t intersect_on(optional<T> (*merge_f)(const T &a, const T &b,
                                                C helper),
                         T elems_out[], std::size_t max_size_out, const T a[],
                         std::size_t a_size, const T b[], std::size_t b_size,
                         C helper) {
  // Reset the state.
  std::size_t n = 0;

  // For this to be safe to call as the implementation of
  // `intersect_on` using `elems_out` as `a` (i.e. `a` will be aliased
  // with `elems_out`), we need to prove that we will never overwrite
  // an existing map element before we read it.  `n` is the index to
  // which we write, and `i` is the index from which we read, so this
  // means we have to prove that `n` is never greater than `i`
  // (i.e. we will never overwrite an element we haven't already
  // examined and dealt with).  I think it's OK, because the only time
  // we increment `n`, we also increment `i`, and they both start at
  // 0.
  for (std::size_t i = 0, j = 0; i < a_size && j < b_size;) {
    const auto &ai = a[i];
    const auto &bj = b[j];
    if (bj < ai) {
      // skip the key from `b`, since it precedes the one in `a`.
      j++;
    } else if (ai < bj) {
      // skip the key from `a`, since it precedes the one in `b`.
      i++;
    } else {
      // We increment `i` and `j` here, because whether or not our
      // merge function returns a new element, we have consumed the
      // input elements and must move on.
      const auto merged = merge_f(a[i++], b[j++], helper);
      if (merged) {
        // Elems match and the merge function returned a result;
        // keep it.
        elems_out[n++] = *merged;
      }
    }
  }

  assert_valid(n, max_size_out, elems_out);

  return n;
}

// Compute the set-intersection of two sorted arrays containing
// elements of type `TA` and `TB` respectively, using the function
// `merge_f` to generate the appropriate output elements of type `T`
// for any overlapping elements.  `merge_f` receives the helper value
// `helper` on each call.  Aliasing is permitted: `elems_out` may be
// the same as `a` or `b` (or both).  See the internal comments for
// discussion of aliasing.
template <typename T, typename TA, typename TB, typename C>
std::size_t intersect_on_multitype(
    optional<T> (*merge_f)(const TA &a, const TB &b, C helper), T elems_out[],
    std::size_t max_size_out, const TA a[], std::size_t a_size, const TB b[],
    std::size_t b_size, C helper) {
  // Reset the state.
  std::size_t n = 0;

  // For this to be safe to call as the implementation of
  // `intersect_on` using `elems_out` as `a` (i.e. `a` will be aliased
  // with `elems_out`), we need to prove that we will never overwrite
  // an existing map element before we read it.  `n` is the index to
  // which we write, and `i` is the index from which we read, so this
  // means we have to prove that `n` is never greater than `i`
  // (i.e. we will never overwrite an element we haven't already
  // examined and dealt with).  I think it's OK, because the only time
  // we increment `n`, we also increment `i`, and they both start at
  // 0.
  for (std::size_t i = 0, j = 0; i < a_size && j < b_size;) {
    const auto &ai = a[i].key;
    const auto &bj = b[j].key;
    if (bj < ai) {
      // skip the key from `b`, since it precedes the one in `a`.
      j++;
    } else if (ai < bj) {
      // skip the key from `a`, since it precedes the one in `b`.
      i++;
    } else {
      // We increment `i` and `j` here, because whether or not our
      // merge function returns a new element, we have consumed the
      // input elements and must move on.
      const auto merged = merge_f(a[i++], b[j++], helper);
      if (merged) {
        // Elems match and the merge function returned a result;
        // keep it.
        elems_out[n++] = *merged;
      }
    }
  }

  assert_valid(n, max_size_out, elems_out);

  return n;
}

// Compute the set-union of two sorted arrays containing elements of
// type `T`, using the function `merge_f` to generate the appropriate
// output for any overlapping elements.  `merge_f` receives the helper
// value `helper` on each call.  None of the arrays may overlap
// (i.e. aliasing is not permitted).
template <typename T, typename C>
optional<std::size_t> extend_on(T (*merge_f)(const T &a, const T &b, C helper),
                                T elems_out[], std::size_t max_size_out,
                                const T a[], std::size_t a_size, const T b[],
                                std::size_t b_size, C helper) {
  std::size_t n = 0;
  std::size_t i = 0, j = 0;

  // Every iteration of this loop produces one output element, unless
  // the output array is full, in which case we immediately return the
  // empty object from the function.
  while (i < a_size || j < b_size) {
    if (n == max_size_out) {
      return {};
    }

    if (i == a_size) {
      // Nothing left in `a`, just finish out `b`.
      elems_out[n++] = b[j++];
      continue;
    }
    if (j == b_size) {
      // Nothing left in `b`, just finish out `a`.
      elems_out[n++] = a[i++];
      continue;
    }

    // We have two elements to compare.
    const auto &ai = a[i];
    const auto &bj = b[j];
    if (bj < ai) {
      // `a` is greater, so `b` has the next key-value pair we need.
      elems_out[n++] = b[j++];
    } else if (ai < bj) {
      // `b` is greater, so take the key-value pair from `a`.
      elems_out[n++] = a[i++];
    } else {
      // Elems are the same, so invoke the merge function.
      elems_out[n++] = merge_f(a[i++], b[j++], helper);
    }
  }

  assert_valid(n, max_size_out, elems_out);
  return n;
}

// `std::set_difference()` does not allow `elems_out` to alias `a` or
// `b`.  Therefore, we define this function which does allow it.
//
// Compute the set-difference of two sorted arrays containing elements
// of type `T`.  On return, `elems_out` contains the elements that
// were in `a` but not in `b`.  `elems_out` may alias the input array
// `a` but not `b`.
template <typename T>
std::size_t difference(T elems_out[], std::size_t max_size_out, const T a[],
                       std::size_t a_size, const T b[], std::size_t b_size) {
  std::size_t n = 0;

  // See `intersect_on(a, b)` for a discussion of aliasing `a` with
  // `*this`; the same reasoning applies to this function.
  std::size_t i = 0;
  for (std::size_t j = 0; i < a_size && j < b_size;) {
    const auto &ai = a[i];
    const auto &bj = b[j];
    if (bj < ai) {
      // The key in `b` is not in `a`, because `a` has advanced ahead.
      // Ignore and move to the next one in `b`.
      j++;
    } else if (ai < bj) {
      // The key in `a` is not in `b`, because `b` has advanced ahead.
      // This means we keep the key and advance to the next one in
      // `a`.
      elems_out[n++] = a[i++];
    } else {
      // This key exists in both maps, so we should not take it.
      i++;
      j++;
    }
  }

  // If we exited the loop before we finished `a`, then the rest of
  // `a` belongs in `elems_`.
  std::copy(a + i, a + a_size, elems_out + n);
  n += a_size - i;

  assert_valid(n, max_size_out, elems_out);
  return n;
}

}  // namespace internal

}  // namespace containers

}  // namespace pvt_common

#endif  // LIBSWIFTNAV_PVT_ENGINE_CONTAINERS_SHARED_H
