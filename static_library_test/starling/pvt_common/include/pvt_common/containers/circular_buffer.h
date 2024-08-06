/*
 * Copyright (C) 2017 Swift Navigation Inc.
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

#ifndef LIBSWIFTNAV_PVT_ENGINE_CIRCULAR_BUFFER_H
#define LIBSWIFTNAV_PVT_ENGINE_CIRCULAR_BUFFER_H

#include <pvt_common/optional.h>
#include <swiftnav/common.h>

#include <algorithm>
#include <array>
#include <iterator>
#include <type_traits>
#include <vector>

#include "shared.h"

namespace pvt_common {

namespace containers {

// This class implements a simple circular buffer which lives on the stack
// and has a fixed, maximum size. Once the maximum size is reached, the
// oldest elements are overwritten so the data structure contains the last
// inserted N items (where N = max_size).
//
// The most common use case will be `insert()`ing elements then iterating
// over them from oldest to newest using the defined `begin()` and `end()`
// functions which return iterators, or from newest to oldest using the
// `rbegin()` and `rend()` functions.
//
// Using a range based for loop makes it easy to iterate automatically from
// the oldest to newest elements without worrying about indexing.
template <typename T, u16 max_size>
class CircularBuffer {
  // Need to forward declare these private nested classes since they're used
  // by the CircularBuffer.
  template <typename U>
  class IteratorImpl {
    using BufferType = std::conditional_t<std::is_const<U>::value,
                                          const CircularBuffer<T, max_size>,
                                          CircularBuffer<T, max_size>>;

   public:
    using value_type = U;
    using pointer = U *;
    using reference = U &;
    using difference_type = s32;
    using iterator_category = std::random_access_iterator_tag;

    IteratorImpl(BufferType *cb, s32 index, s32 num_elements, bool is_end)
        : buf_(cb), index_(index), count_(is_end ? num_elements : 0){};

    template <typename B>
    bool operator==(const IteratorImpl<B> &that) const {
      return index_ == that.index_ && count_ == that.count_;
    }

    template <typename B>
    bool operator!=(const IteratorImpl<B> &that) const {
      return not operator==(that);
    }

    U &operator*() const { return buf_->get(index_); }

    U *operator->() const { return &this->operator*(); }

    IteratorImpl<U> &operator++() {
      index_ = increment(index_);
      count_++;
      return *this;
    }

    IteratorImpl<U> operator++(int) {
      auto temp = *this;
      this->operator++();
      return temp;
    }

    IteratorImpl<U> &operator--() {
      index_ = decrement(index_);
      count_--;
      return *this;
    }

    IteratorImpl<U> operator--(int) {
      auto temp = *this;
      this->operator--();
      return temp;
    }

    IteratorImpl<U> &operator+=(s32 offset) {
      index_ = advance(index_, offset);
      count_ += offset;
      return *this;
    }

    IteratorImpl<U> &operator-=(s32 offset) {
      *this += -offset;
      return *this;
    }

    IteratorImpl<U> operator+(s32 offset) const {
      auto result = *this;
      return (result += offset);
    }

    IteratorImpl<U> operator-(s32 offset) const {
      auto result = *this;
      return (result -= offset);
    }

    template <typename B>
    difference_type operator-(const IteratorImpl<B> &that) {
      return count_ - that.count_;
    }

    template <typename B>
    bool operator<(const IteratorImpl<B> &that) const {
      return count_ < that.count_;
    }

    template <typename B>
    bool operator>(const IteratorImpl<B> &that) const {
      return (that < *this);
    }

    template <typename B>
    bool operator<=(const IteratorImpl<B> &that) const {
      return !(*this > that);
    }

    template <typename B>
    bool operator>=(const IteratorImpl<B> &that) const {
      return !(*this < that);
    }

    U &operator[](s32 offset) const {
      return buf_->get(advance(index_, offset));
    }

   private:
    BufferType *buf_;
    s32 index_;
    s32 count_;
  };

  typedef IteratorImpl<T> Iterator;
  typedef IteratorImpl<const T> ConstIterator;
  typedef std::reverse_iterator<Iterator> ReverseIterator;
  typedef std::reverse_iterator<ConstIterator> ConstReverseIterator;

 public:
  CircularBuffer() : data_(), tail_(0), num_elements_(0){};

  // This constructor should only be called from the test suite.
  explicit CircularBuffer(const std::vector<T> &elems);

  // Insert a new element into the buffer. If the buffer has reached its
  // maximum size, the oldest element will be overwritten.
  void insert(const T &element) {
    data_[static_cast<std::size_t>(get_write_index())] = element;
    advance_write_ptr();
  }

  // Insert a new uninitialized element into the buffer. This element can be
  // accessed with rbegin() and modified in place.
  void extend() { advance_write_ptr(); }

  // This erases the buffer in constant time without having to traverse
  // each element.
  void clear() {
    tail_ = 0;
    num_elements_ = 0;
  }

  // Drop the oldest N elements in the buffer. If there are fewer than
  // N elements currently in the buffer, all elements will be
  // removed.
  void pop_earliest(int n) {
    if (n >= size()) {
      clear();
    } else {
      num_elements_ -= n;
      tail_ = advance(tail_, n);
    }
  }

  // Drop the oldest element in the buffer.
  void pop_earliest() { pop_earliest(1); }

  // Returns a copy of the element which was inserted most recently.
  optional<T> get_latest() const {
    if (num_elements_ == 0) {
      return {};
    }
    return data_[static_cast<std::size_t>(get_latest_index())];
  }

  // Returns a copy of the element which was inserted least recently.
  optional<T> get_earliest() const {
    if (num_elements_ == 0) {
      return {};
    }
    return data_[static_cast<std::size_t>(tail_)];
  }

  // Returns a reference to the element which was inserted least recently.
  optional<const T &> get_earliest_ref() const {
    if (num_elements_ == 0) {
      return {};
    }
    return data_[static_cast<std::size_t>(tail_)];
  }

  s32 size() const { return num_elements_; }

  static s32 get_max_size() { return max_size; }

  Iterator begin() { return Iterator(this, tail_, num_elements_, false); }

  Iterator end() {
    return Iterator(this, get_write_index(), num_elements_, true);
  }

  ConstIterator begin() const { return cbegin(); }

  ConstIterator end() const { return cend(); }

  ConstIterator cbegin() const {
    return ConstIterator(this, tail_, num_elements_, false);
  }

  ConstIterator cend() const {
    return ConstIterator(this, get_write_index(), num_elements_, true);
  }

  ReverseIterator rbegin() { return std::make_reverse_iterator(end()); }

  ReverseIterator rend() { return std::make_reverse_iterator(begin()); }

  ConstReverseIterator crbegin() const {
    return std::make_reverse_iterator(cend());
  }

  ConstReverseIterator crend() const {
    return std::make_reverse_iterator(cbegin());
  }

 private:
  void advance_write_ptr() {
    // Depending if the buffer is full or not, we might need to move the
    // `head_` pointer.
    if (num_elements_ < max_size) {
      num_elements_++;
    } else if (num_elements_ == max_size) {
      tail_ = increment(tail_);
    }
  }

  // Get the index where to next insert an element.
  s32 get_write_index() const { return (tail_ + num_elements_) % max_size; }

  // Get the index where an element was mostly recently inserted. Need to
  // handle wrapping around 0.
  s32 get_latest_index() const { return decrement(get_write_index()); }

  static s32 increment(s32 i) { return (i + 1) % max_size; }
  static s32 decrement(s32 i) { return (i > 0) ? (i - 1) : (max_size - 1); }
  static s32 advance(s32 i, s32 n) {
    s32 j = (i + n) % max_size;
    if (j < 0) {
      // restore remainder into non-negative range
      j += max_size;
    }
    return j;
  }

  const T &get(s32 index) const {
    return data_[static_cast<std::size_t>(index)];
  }

  T &get(s32 index) { return data_[static_cast<std::size_t>(index)]; }

  std::array<T, max_size> data_;
  // We store the `index_` of the element which was inserted least recently
  // as well as the total number of elements in the buffer.
  s32 tail_, num_elements_;
};

template <typename T, u16 N>
constexpr auto operator+(s32 lhs,
                         const typename CircularBuffer<T, N>::Iterator &rhs) {
  return rhs + lhs;
}

template <typename T, u16 N>
constexpr auto operator+(
    s32 lhs, const typename CircularBuffer<T, N>::ConstIterator &rhs) {
  return rhs + lhs;
}

template <typename T, u16 N>
constexpr auto operator+(
    s32 lhs, const typename CircularBuffer<T, N>::ReverseIterator &rhs) {
  return rhs + lhs;
}

template <typename T, u16 N>
constexpr auto operator+(
    s32 lhs, const typename CircularBuffer<T, N>::ConstReverseIterator &rhs) {
  return rhs + lhs;
}

}  // namespace containers

}  // namespace pvt_common

#endif  // LIBSWIFTNAV_PVT_ENGINE_CIRCULAR_BUFFER_H
