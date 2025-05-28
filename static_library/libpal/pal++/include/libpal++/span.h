#ifndef LIBPAL_CPP_SPAN_H
#define LIBPAL_CPP_SPAN_H

#include <cassert>
#include <cstddef>
#include <iterator>
#include <limits>
#include <type_traits>

#include <libpal++/size.h>  // pal::size can be imported via header file

namespace pal {

/**
 * This is a C++14 version of the std::span class.
 *
 * This class represent a contiguous region of memory through a class like
 * interface. The benefit that this class provides over direct pointer
 * arithmetic is that by default all public interfaces perform assert checks to
 * make sure that all operations are valid and no buffer overflow takes place.
 *
 * @note Unfortunately this version of the std::span alternative won't be able
 * to achieve what the real class does as the language requires Class Template
 * Argument Deduction (CTAD) for a number of constructors to work. As such we've
 * removed the concept of "Extent" from the class as it would make the class a
 * bit harder to use with little benefit
 *
 * @tparam T span data type
 */
template <typename T>
class Span final {
 public:
  using element_type = T;
  using value_type = std::remove_const_t<T>;
  using size_type = size_t;
  using difference_type = ptrdiff_t;
  using pointer = T *;
  using const_pointer = const T *;
  using reference = T &;
  using const_reference = const T &;
  using iterator = pointer;
  using reverse_iterator = std::reverse_iterator<iterator>;

 public:
  /**
   * Default Constructor
   *
   * Empty span
   */
  constexpr Span() noexcept = default;

  /**
   * Construct span from primitive array
   *
   * @tparam N size of array
   * @param array reference to array
   */
  template <size_t N>
  constexpr Span(T (&array)[N]) noexcept : data_(array), size_(N) {}

  /**
   * Construct span from contiguous memory region
   *
   * @note constructor will assert if the the data pointer is nullptr and the
   * size > 0.
   *
   * @param data pointer to the start of memory region
   * @param size elements within memory region
   */
  constexpr Span(T *data, size_t size) : data_(data), size_(size) {
    assert(!(data_ == nullptr && size_ > 0));
  }

  /**
   * Default Copy Constructor
   *
   * @param other other span
   */
  constexpr Span(const Span &other) noexcept = default;

  /**
   * Default Assignment Operator
   *
   * @param other other span
   * @return this span
   */
  constexpr Span &operator=(const Span &other) noexcept = default;

  /**
   * @note this class asserts in developer builds if the index requested is past
   * the span size.
   *
   * @param index index of the element to return
   * @return reference to the requested element
   */
  constexpr reference operator[](size_type index) const {
    assert(index < size_);
    return data_[index];
  }

  /**
   * @note this class asserts in developer builds if the span is empty.
   *
   * @return reference to the first element
   */
  constexpr reference front() const {
    assert(!empty());
    return data_[0];
  }

  /**
   * @note this class asserts in developer builds if the span is empty.
   *
   * @return reference to the last element
   */
  constexpr reference back() const {
    assert(!empty());
    return data_[size_ - 1];
  }

  /**
   * @return pointer to the underlying span, nullptr if the span is empty
   */
  constexpr pointer data() const noexcept { return data_; }

  /**
   * @return an iterator to the first element
   */
  constexpr iterator begin() const noexcept { return iterator(data_); }

  /**
   * @return an iterator to the element following the last element
   */
  constexpr iterator end() const noexcept { return iterator(&data_[size_]); }

  /**
   * @return reverse iterator to the first element
   */
  constexpr reverse_iterator rbegin() const noexcept {
    return reverse_iterator(end());
  }

  /**
   * @return reverse iterator to the element following the last element
   */
  constexpr reverse_iterator rend() const noexcept {
    return reverse_iterator(begin());
  }

  /**
   * @return true if the span is empty, false otherwise
   */
  constexpr bool empty() const noexcept { return size_ == 0; }

  /**
   * @return number of elements in the span
   */
  constexpr size_type size() const noexcept { return size_; }

  /**
   * @return size of the sequence in bytes
   */
  constexpr size_type size_bytes() const noexcept { return size_ * sizeof(T); }

  /**
   * @note this class asserts in developer builds if the count requested is
   * larger than the span.
   *
   * @return span that is a view over the first count elements of *this
   */
  constexpr Span first(size_type count) const {
    assert(count <= size_);
    return Span(data_, count);
  }

  /**
   * @note this class asserts in developer builds if the count requested is
   * larger than the span.
   *
   * @return span that is a view over the last count elements of *this
   */
  constexpr Span last(size_type count) const {
    assert(count <= size_);
    return Span(data_ + size_ - count, count);
  }

  /**
   * @note this class asserts in developer builds if the requests subspan is
   * illogical.
   *
   * @param offset start index of the substring
   * @param count number of characters in the substring
   * @return sub span region
   */
  constexpr Span subspan(
      size_type offset,
      size_type count = std::numeric_limits<size_type>::max()) const {
    assert(offset < size_);

    if (count == std::numeric_limits<size_type>::max()) {
      count = size_ - offset;
    }

    assert(count + offset <= size_);
    return Span(data_ + offset, count);
  }

 private:
  T *data_ = nullptr;
  size_t size_ = 0;
};

/**
 * This is a C++14 version of the std::as_bytes for the Span class.
 *
 * @tparam T span data type
 * @param span which is expected to be reinterpreted
 * @return reinterpreted span
 */
template <typename T>
Span<const uint8_t> as_bytes(Span<T> span) noexcept {
  return Span<const uint8_t>(reinterpret_cast<const uint8_t *>(span.data()),
                             span.size_bytes());
}

/**
 * This is a C++14 version of the std::as_writable_bytes for the Span class.
 *
 * @tparam T span data type
 * @param span which is expected to be reinterpreted
 * @return reinterpreted span
 */
template <typename T>
Span<uint8_t> as_writable_bytes(Span<T> span) noexcept {
  return Span<uint8_t>(reinterpret_cast<uint8_t *>(span.data()),
                       span.size_bytes());
}

}  // namespace pal

#endif  // LIBPAL_CPP_SPAN_H
