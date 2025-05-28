/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_PVT_ENGINE_RANGES_ZIP_H
#define LIBSWIFTNAV_PVT_ENGINE_RANGES_ZIP_H

#include <algorithm>
#include <array>
#include <cassert>
#include <numeric>
#include <tuple>

#include <pvt_common/optional.h>
#include <swiftnav/macros.h>

namespace pvt_engine {

namespace ranges {

namespace internal {

template <class F, size_t... Is>
constexpr auto index_apply_impl(F f, std::index_sequence<Is...>) {
  return f(std::integral_constant<size_t, Is>{}...);
}

template <class... Tuple, class F>
constexpr auto index_apply(F f) {
  return index_apply_impl(f, std::index_sequence_for<Tuple...>{});
}

template <class Iterator>
using distance_type_of_iter =
    decltype(std::distance(std::declval<Iterator>(), std::declval<Iterator>()));

template <class... IteratorTypes>
using common_distance_type_iter =
    typename std::common_type<distance_type_of_iter<IteratorTypes>...>::type;

template <class Iterator, class Tag>
struct iterator_is {
  static constexpr bool value = std::is_base_of<
      Tag, typename std::iterator_traits<Iterator>::iterator_category>::value;
};

template <bool... B>
struct conjunction {};

template <bool Head, bool... Tail>
struct conjunction<Head, Tail...>
    : std::integral_constant<bool, Head && conjunction<Tail...>::value> {};

template <bool B>
struct conjunction<B> : std::integral_constant<bool, B> {};

template <bool... B>
struct disjunction {};

template <bool Head, bool... Tail>
struct disjunction<Head, Tail...>
    : std::integral_constant<bool, Head || disjunction<Tail...>::value> {};

template <bool B>
struct disjunction<B> : std::integral_constant<bool, B> {};

template <class Tag, class... IteratorTypes>
struct all_iterators_are {
  static constexpr bool value =
      conjunction<iterator_is<IteratorTypes, Tag>::value...>::value;
};

template <class... IteratorTypes>
struct all_random_access {
  static constexpr bool value =
      all_iterators_are<std::random_access_iterator_tag,
                        IteratorTypes...>::value;
};

template <class... IteratorTypes>
struct all_bidirectional {
  static constexpr bool value =
      all_iterators_are<std::bidirectional_iterator_tag,
                        IteratorTypes...>::value;
};

template <class... IteratorTypes>
struct all_at_least_bidirectional {
  static constexpr bool value =
      disjunction<all_random_access<IteratorTypes...>::value,
                  all_bidirectional<IteratorTypes...>::value>::value;
};

template <class... IteratorTypes>
struct computed_iterator_tag {
  using type = typename std::conditional<
      all_random_access<IteratorTypes...>::value,
      std::random_access_iterator_tag,
      typename std::conditional<all_bidirectional<IteratorTypes...>::value,
                                std::bidirectional_iterator_tag,
                                std::forward_iterator_tag>::type>::type;
};

}  // namespace internal

// This object is the iterator type for `ZipSequence` which follows
// below.  Dereferencing yields a tuple of references to the elements
// from each sequence at the current position.
//
// This iterator supports only the behavior of its least efficient
// underlying sequence iterator type.  In other words, if one of the
// underlying iterators is a `std::forward_list`, this iterator will,
// like a `std::forward_list`, be a `ForwardIterator` with no support
// for reversing or random access, even if all the other underlying
// sequences support it.  This is to prevent you accidentally
// introducing linear- or quadratic-time behavior when using the
// iterators.
template <class... IteratorTypes>
class ZipIterator
    : public std::iterator<
          typename internal::computed_iterator_tag<IteratorTypes...>::type,
          std::tuple<decltype(*std::declval<IteratorTypes>())...>,
          typename internal::common_distance_type_iter<IteratorTypes...>> {
 public:
  using difference_type =
      typename internal::common_distance_type_iter<IteratorTypes...>;

  ZipIterator(IteratorTypes... args) : iterators_(args...) {}

  ZipIterator &operator++() {
    internal::index_apply<IteratorTypes...>([&](const auto... Is) {
      return std::make_tuple(++std::get<Is>(iterators_)...);
    });
    return *this;
  }

  ZipIterator operator++(int SWIFT_ATTR_UNUSED) {
    ZipIterator old(*this);
    ++(*this);
    return old;
  }

  // This hilarious mess is to disable functions that don't exist when
  // this class is not a `BidirectionalIterator`.  `std::enable_if`
  // doesn't work on its own as this wouldn't be a templated function,
  // but we can make it templated and default everything to the
  // appropriate types to be able to use SFINAE as we hoped.
  //
  // This pattern is repeated all over for members whose existence
  // depends on the underlying iterator categories.
  template <
      typename U = ZipIterator &,
      class = std::enable_if_t<
          internal::all_at_least_bidirectional<IteratorTypes...>::value, U>>
  ZipIterator &operator--() {
    internal::index_apply<IteratorTypes...>([&](const auto... Is) {
      return std::make_tuple(--std::get<Is>(iterators_)...);
    });
    return *this;
  }

  template <
      typename U = ZipIterator,
      class = std::enable_if_t<
          internal::all_at_least_bidirectional<IteratorTypes...>::value, U>>
  ZipIterator operator--(int SWIFT_ATTR_UNUSED) {
    ZipIterator old(*this);
    --(*this);
    return old;
  }

  bool operator==(const ZipIterator &other) const {
    return iterators_ == other.iterators_;
  }

  bool operator!=(const ZipIterator &other) const { return !(*this == other); }

  std::tuple<decltype(*std::declval<IteratorTypes>())...> operator*() {
    return internal::index_apply<IteratorTypes...>([&](const auto... Is) {
      return std::tie(*std::get<Is>(iterators_)...);
    });
  }

  template <
      typename U = std::tuple<decltype(*std::declval<IteratorTypes>())...>,
      class = std::enable_if_t<
          internal::all_random_access<IteratorTypes...>::value, U>>
  U operator[](difference_type n) {
    return *(*this + n);
  }

  template <typename U = ZipIterator,
            class = std::enable_if_t<
                internal::all_random_access<IteratorTypes...>::value, U>>
  U operator+(difference_type n) const {
    const auto advance_copy = [&](const auto &iter) {
      return iter + static_cast<decltype(std::distance(iter, iter))>(n);
    };
    return internal::index_apply<IteratorTypes...>([&](const auto... Is) {
      return ZipIterator(advance_copy(std::get<Is>(iterators_))...);
    });
  }

  template <typename U = ZipIterator &,
            class = std::enable_if_t<
                internal::all_random_access<IteratorTypes...>::value, U>>
  U operator+=(difference_type n) {
    const auto advance = [&](auto iter) {
      return iter += static_cast<decltype(std::distance(iter, iter))>(n);
    };

    internal::index_apply<IteratorTypes...>([&](const auto... Is) {
      return std::make_tuple(advance(std::get<Is>(iterators_))...);
    });
    return *this;
  }

  template <typename U = ZipIterator,
            class = std::enable_if_t<
                internal::all_random_access<IteratorTypes...>::value, U>>
  U operator-(difference_type n) const {
    return *this + (-n);
  }

  template <typename U = ZipIterator &,
            class = std::enable_if_t<
                internal::all_random_access<IteratorTypes...>::value, U>>
  U operator-=(difference_type n) {
    return *this += (-n);
  }

  template <typename U = difference_type,
            class = std::enable_if_t<
                internal::all_random_access<IteratorTypes...>::value, U>>
  U operator-(const ZipIterator &other) const {
    const auto diff_iters = [&](const auto Idx) {
      const auto diff = static_cast<difference_type>(
          std::get<Idx>(iterators_) - std::get<Idx>(other.iterators_));
      return diff;
    };
    const auto diffs =
        internal::index_apply<IteratorTypes...>([&](const auto... Is) {
          return std::array<difference_type, sizeof...(IteratorTypes)>{
              {diff_iters(Is)...}};
        });
    // Ensure that all iterator distances are equal (otherwise there
    // is no correct distance we can return).
    const bool all_equal =
        std::accumulate(std::next(diffs.begin()), diffs.end(), true,
                        [&](const bool acc, const difference_type diff) {
                          return acc && (diff == *diffs.begin());
                        });
    (void)all_equal;
    assert(all_equal);
    return *diffs.begin();
  }

  template <typename U = bool,
            class = std::enable_if_t<
                internal::all_random_access<IteratorTypes...>::value, U>>
  U operator<(const ZipIterator &other) const {
    return other - *this > 0;
  }

  template <typename U = bool,
            class = std::enable_if_t<
                internal::all_random_access<IteratorTypes...>::value, U>>
  U operator<=(const ZipIterator &other) const {
    return other - *this >= 0;
  }

  template <typename U = bool,
            class = std::enable_if_t<
                internal::all_random_access<IteratorTypes...>::value, U>>
  U operator>(const ZipIterator &other) const {
    return !(*this <= other);
  }

  template <typename U = bool,
            class = std::enable_if_t<
                internal::all_random_access<IteratorTypes...>::value, U>>
  U operator>=(const ZipIterator &other) const {
    return !(*this < other);
  }

 private:
  std::tuple<IteratorTypes...> iterators_;
};

template <class... IteratorTypes>
ZipIterator<IteratorTypes...> operator+(
    typename ZipIterator<IteratorTypes...>::difference_type n,
    ZipIterator<IteratorTypes...> iter) {
  iter += n;
  return iter;
}

template <class... Sequences>
using zip_of = ZipIterator<decltype(std::begin(std::declval<Sequences>()))...>;

template <class... Sequences>
auto begin_zip(Sequences &&... seqs) {
  return zip_of<Sequences...>(std::begin(seqs)...);
}

template <class... Sequences>
auto end_zip(Sequences &&... seqs) {
  return zip_of<Sequences...>(std::end(seqs)...);
}

template <class... Sequences>
auto advanced_zip(std::size_t n, Sequences &&... seqs) {
  const auto advance_beginning = [&](auto &&s) -> decltype(std::begin(s)) {
    auto it = std::begin(s);
    using distance_type = decltype(std::distance(std::begin(s), std::end(s)));
    std::advance(it, static_cast<distance_type>(n));
    return it;
  };
  return zip_of<Sequences...>(advance_beginning(seqs)...);
}

template <class Sequence>
using distance_type_of = decltype(std::distance(
    std::begin(std::declval<Sequence>()), std::end(std::declval<Sequence>())));

template <class... Sequences>
using common_distance_type =
    typename std::common_type<distance_type_of<Sequences>...>::type;

template <class... Sequences>
bool all_lengths_equal(Sequences &&... seqs) {
  using distance_type =
      typename std::common_type<distance_type_of<Sequences>...>::type;
  const auto get_distance = [](auto &&s) {
    return static_cast<distance_type>(
        std::distance(std::begin(s), std::end(s)));
  };
  // This function checks whether all sequences are the same size.
  const auto distances =
      std::array<distance_type, sizeof...(Sequences)>{{get_distance(seqs)...}};

  const auto fold_distance_equality = [&](bool acc, distance_type dist) {
    return acc && (dist == *std::begin(distances));
  };
  return std::accumulate(std::next(std::begin(distances)), std::end(distances),
                         true, fold_distance_equality);
}

template <class... Sequences>
bool all_lengths_at_least(std::size_t n, Sequences &&... seqs) {
  const auto at_least_n = [&](auto &&s) {
    const auto dist(std::distance(std::begin(s), std::end(s)));
    return dist >= static_cast<decltype(dist)>(n);
  };
  // This function checks whether all sequences are at least `n`
  // elements long.
  const auto distances =
      std::array<bool, sizeof...(Sequences)>{{at_least_n(seqs)...}};

  return std::accumulate(std::begin(distances), std::end(distances), true,
                         [](bool acc, bool dist) { return acc && dist; });
}

// This object represents a combination of several sequences.  The
// only thing you are permitted do with it is get iterators from it.
//
// One situation in which you may deal with this object explicitly is
// if you want to implement something like Haskell's `zipWith`, which
// merges sequences using a user-provided function.  This would look
// something like:
//
//   // let's say v, w, x are std::vector<double>
//   const auto sequences = ranges::zip(v, w, x);
//   std::vector<double> output{};
//   std::transform(sequences.begin(), sequences.end(),
//                  std::back_inserter(output),
//                  [](const std::tuple<double, double, double> &vwx_elements) {
//                    // do something with these elements
//                    return result
//                  });
template <class... Sequences>
class ZipSequence {
 public:
  ZipSequence(Sequences &... seqs) : sequences(seqs...), n() {}
  ZipSequence(std::size_t nn, Sequences &... seqs)
      : sequences(seqs...), n(nn) {}
  auto begin() {
    return internal::index_apply<Sequences...>([&](const auto... Is) {
      return begin_zip(std::get<Is>(sequences)...);
    });
  }
  auto end() {
    if (n.has_value()) {
      return internal::index_apply<Sequences...>([&](const auto... Is) {
        return advanced_zip(*n, std::get<Is>(sequences)...);
      });
    }
    return internal::index_apply<Sequences...>(
        [&](const auto... Is) { return end_zip(std::get<Is>(sequences)...); });
  }

 private:
  std::tuple<Sequences &...> sequences;
  optional<std::size_t> n;
};

// Create a combined sequence object from the provided sequences,
// which must be equipped with their own iterators.  The sequences
// must be of equal length; if they aren't, an assertion will be
// triggered.
//
// The main way of interacting with the resulting object is via its
// iterators, which, when dereferenced, yield tuples of the elements
// that occur at matching positions in the input sequences.  Mutating
// elements of these tuples will mutate the original input sequences.
//
// One example use is in a range-based for-loop.  Instead of writing
//
//   for (std::size_t i = 0; i < v.size(); ++i) {
//     v[i] = g(v[i], w[i]);
//   }
//
// you can use a zip:
//
//   for (auto pair : ranges::zip(v, w)) {
//     std::get<0>(pair) = g(std::get<0>(pair), std::get<1>(pair));
//   }
//
// See the documentation for `ZipSequence` for some more usage
// information.
//
// The length-checking assertion can take O(n) time if the underlying
// sequences are not random-access iterators (see the documentation
// for `zip_min()` for more information on this penalty).
template <class... Sequences>
auto zip(Sequences &... seqs) {
  // For now we assert that all the sequences are equal.  Dealing with
  // non-matching sequences gets super complicated due to the way C++
  // iterators work.
  assert(all_lengths_equal(seqs...));
  return ZipSequence<Sequences...>(seqs...);
}

// Create a combined sequence object from the provided sequences,
// which must be equipped with their own iterators.  The sequences
// must all be at least of length `n`; if they aren't, an assertion
// will be triggered.
//
// The main way of interacting with the resulting object is via its
// iterators, which, when dereferenced, yield tuples of the elements
// that occur at matching positions in the input sequences.  Mutating
// elements of these tuples will mutate the original input sequences.
//
// The length-checking assertion can take O(n) time if the underlying
// sequences are not random-access iterators (see the documentation
// for `zip_min()` for more information on this penalty).
template <class... Sequences>
auto zip_n(std::size_t n, Sequences &... seqs) {
  assert(all_lengths_at_least(n, seqs...));
  return ZipSequence<Sequences...>(n, seqs...);
}

// Create a combined sequence object from the provided sequences,
// which must be equipped with their own iterators.  The resulting
// sequence will only be as long as the shortest input sequence; all
// elements of other sequences after this point are ignored and not
// accessible through the result of this function.
//
// The main way of interacting with the resulting object is via its
// iterators, which, when dereferenced, yield tuples of the elements
// that occur at matching positions in the input sequences.  Mutating
// elements of these tuples will mutate the original input sequences.
//
// Calculating the lengths of the input sequences is done by
// iterators.  If the input sequences are random-access iterators,
// like vectors, arrays or the `pvt_common/containers` structures,
// this is very efficient.  This function will incur an extra cost of
// O(n) time for each sequence that does not support random access by
// iterators in order to determine the length of the smallest input
// sequence.  Sorry.
template <class... Sequences>
auto zip_min(Sequences &... seqs) {
  using distance_t =
      typename std::common_type<distance_type_of<Sequences>...>::type;
  const auto get_distance = [](auto &&s) {
    return std::distance(std::begin(s), std::end(s));
  };

  const auto distances =
      std::array<distance_t, sizeof...(Sequences)>{{get_distance(seqs)...}};

  const distance_t n = *std::min_element(distances.begin(), distances.end());
  return zip_n(static_cast<std::size_t>(n), seqs...);
}

}  // namespace ranges

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_RANGES_ZIP_H
