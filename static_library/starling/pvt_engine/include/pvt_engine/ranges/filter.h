/**
 * Copyright (C) 2020 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_PVT_ENGINE_RANGES_FILTER_H
#define LIBSWIFTNAV_PVT_ENGINE_RANGES_FILTER_H

#include <iterator>
#include <type_traits>

namespace pvt_engine {
namespace ranges {

/**
 * Provides a way to filter out iterators containing unwanted values.
 *
 * @note This function is rarely constructed directly, instead the `filter`
 * function is provided which does the needed type deduction.
 *
 * This class implements the `filter` function from languages like Haskell,
 * OCaml, etc. but in a C++-y way using iterators. This class takes a begin and
 * end iterator and a predicate and provides a new iterator type which wraps the
 * provided iterator. The new iterator will call the `Predicate` function object
 * with each wrapped iterator, and will skip over the ones that `Predicate`
 * returns `false`.
 *
 * @code{.cc}
 * std::vector<int> numbers{1,2,3,4,5,6,7,8};
 * auto is_even = [](const int &i) { return (i % 2) == 0; };
 * for (const auto &even_values : ranges::filter(numbers, is_even)) {
 *   // Do something with even values
 * }
 * @endcode
 *
 * @tparam Iter The underlying iterator type to filter
 * @tparam Predicate The function object used to determine which `Iter` objects
 * to filter out. Should take in a single value of type `Iter::reference` and
 * returns a `bool`. `true` is the value should be made available, `false` if it
 * should be filtered out.
 */
template <typename Iter, typename Predicate>
class FilterView {
 public:
  class Iterator {
   public:
    using difference_type = typename Iter::difference_type;
    using value_type = typename Iter::value_type;
    using pointer = typename Iter::pointer;
    using reference = typename Iter::reference;
    using iterator_category = std::conditional_t<
        std::is_base_of<std::forward_iterator_tag,
                        typename Iter::iterator_category>::value,
        std::forward_iterator_tag, typename Iter::iterator_category>;

    Iterator(Iter current, Iter end, Predicate &pred)
        : current_(current), end_(end), pred_(pred) {
      while ((current_ != end_) && !pred_(*current_)) {
        ++current_;
      }
    }

    decltype(auto) operator*() { return *current_; }

    decltype(auto) operator*() const { return *current_; }

    Iterator &operator++() {
      increment();
      return *this;
    }

    Iterator operator++(int) {
      Iterator tmp = *this;
      increment();
      return tmp;
    }

    bool operator==(const Iterator &rhs) const {
      return current_ == rhs.current_;
    }

    bool operator!=(const Iterator &rhs) const { return !operator==(rhs); }

   private:
    void increment() {
      if (current_ != end_) {
        do {
          ++current_;
        } while (!pred_(*current_) && (current_ != end_));
      }
    };

    Iter current_;
    Iter end_;
    Predicate &pred_;
  };

  FilterView(const Iter &begin, const Iter &end, const Predicate &pred)
      : begin_(begin), end_(end), pred_(pred) {}

  Iterator begin() { return Iterator(begin_, end_, pred_); }

  Iterator end() { return Iterator(end_, end_, pred_); }

 private:
  Iter begin_;
  Iter end_;
  Predicate pred_;
};

/**
 * Filters the iterators of a range, only allowing the values that the predicate
 * returns `true` for.
 * @tparam Range The type of the range to filter. Should have `begin()` and
 * `end()` functions that provide iterators
 * @tparam Predicate The type of the predicate. Should take in a value of the
 * `Range` and return a `bool`. `true` if the value should be available, `false`
 * if it should be filtered  out.
 * @param r The range object to filter
 * @param p The predicate object to use
 * @return A `FilterView` object  which wraps the iterators from `r` and filters
 * them using `p`
 */
template <typename Range, typename Predicate>
auto filter(Range &r, const Predicate &p) {
  using Iterator = decltype(std::declval<Range>().begin());
  return FilterView<Iterator, Predicate>(r.begin(), r.end(), p);
}

}  // namespace ranges
}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_RANGES_FILTER_H
