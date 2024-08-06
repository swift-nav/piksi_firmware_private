#ifndef LIBPAL_CPP_STRING_VIEW_H
#define LIBPAL_CPP_STRING_VIEW_H

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <iterator>
#include <string>

namespace pal {

/**
 * This is a C++14 version of the std::basic_string_view class.
 *
 * @note this class is not fully implemented as per the standard library, should
 * methods be missing, they will need to be added on a per need basis.
 */
template <typename CharT, typename Traits = std::char_traits<CharT>>
class BasicStringView final {
 public:
  using traits_type = Traits;
  using value_type = CharT;
  using pointer = CharT *;
  using const_pointer = const CharT *;
  using reference = CharT &;
  using const_reference = const CharT &;
  using const_iterator = const_pointer;
  using iterator = const_iterator;
  using const_reverse_iterator = std::reverse_iterator<const_iterator>;
  using reverse_iterator = const_reverse_iterator;
  using size_type = size_t;
  using difference_type = ptrdiff_t;

 public:
  /**
   * Default Constructor
   *
   * Empty string view
   */
  constexpr BasicStringView() noexcept = default;

  /**
   * Construct string view from a C string
   *
   * @param string pointer to the start of the string
   */
  constexpr BasicStringView(const CharT *string) noexcept
      : string_(string),
        size_(string != nullptr ? Traits::length(string) : 0) {}

  /**
   * Construct string view from a sequence of characters
   *
   * @note constructor will assert if the the string pointer is nullptr and the
   * size > 0.
   *
   * @param string pointer to the start of the string
   * @param size number of characters in the string
   */
  constexpr BasicStringView(const CharT *string, size_type size)
      : string_(string), size_(size) {
    assert(!(string_ == nullptr && size_ > 0));
  }

  /**
   * Default Copy Constructor
   *
   * @param other other string view
   */
  constexpr BasicStringView(const BasicStringView &other) noexcept = default;

  /**
   * Default Assignment Operator
   *
   * @param other other string view
   * @return this string view
   */
  constexpr BasicStringView &operator=(const BasicStringView &other) noexcept =
      default;

  /**
   * @return an iterator to the first element
   */
  constexpr const_iterator begin() noexcept { return iterator(string_); }

  /**
   * @return an iterator to the first element
   */
  constexpr const_iterator begin() const noexcept { return cbegin(); }

  /**
   * @return an iterator to the first element
   */
  constexpr const_iterator cbegin() const noexcept {
    return const_iterator(string_);
  }

  /**
   * @return an iterator to the element following the last element
   */
  constexpr const_iterator end() noexcept { return iterator(&string_[size_]); }

  /**
   * @return an iterator to the element following the last element
   */
  constexpr const_iterator end() const noexcept { return cend(); }

  /**
   * @return an iterator to the element following the last element
   */
  constexpr const_iterator cend() const noexcept {
    return const_iterator(&string_[size_]);
  }

  /**
   * @return reverse iterator to the first element
   */
  constexpr const_reverse_iterator rbegin() noexcept {
    return reverse_iterator(end());
  }

  /**
   * @return reverse iterator to the first element
   */
  constexpr const_reverse_iterator rbegin() const noexcept {
    return const_reverse_iterator(crbegin());
  }

  /**
   * @return reverse iterator to the first element
   */
  constexpr const_reverse_iterator crbegin() const noexcept {
    return const_reverse_iterator(cend());
  }

  /**
   * @return reverse iterator to the element following the last element
   */
  constexpr const_reverse_iterator rend() noexcept {
    return reverse_iterator(begin());
  }

  /**
   * @return reverse iterator to the element following the last element
   */
  constexpr const_reverse_iterator rend() const noexcept {
    return const_reverse_iterator(crend());
  }

  /**
   * @return reverse iterator to the element following the last element
   */
  constexpr const_reverse_iterator crend() const noexcept {
    return const_reverse_iterator(cbegin());
  }

  /**
   * @note this class asserts in developer builds if the index requested is past
   * the string view size.
   *
   * @param index index of the element to return
   * @return reference to the requested element
   */
  constexpr const_reference operator[](size_type index) const {
    assert(index < size_);
    return string_[index];
  }

  /**
   * @note this class asserts in developer builds if the string view is empty.
   *
   * @return reference to the first element
   */
  constexpr const_reference front() const {
    assert(!empty());
    return string_[0];
  }

  /**
   * @note this class asserts in developer builds if the string view is empty.
   *
   * @return reference to the last element
   */
  constexpr const_reference back() const {
    assert(!empty());
    return string_[size_ - 1];
  }

  /**
   * @return pointer to the underlying string view, nullptr if the string view
   * is empty
   */
  constexpr const_pointer data() const noexcept { return string_; }

  /**
   * @return number of characters in the string view
   */
  constexpr size_type size() const noexcept { return size_; }

  /**
   * @return number of characters in the string view
   */
  constexpr size_type length() const noexcept { return size_; }

  /**
   * @return true if the string view is empty, false otherwise
   */
  constexpr bool empty() const noexcept { return size_ == 0; }

  /**
   * Moves the start of the view forward by n characters
   *
   * @param n number of characters
   */
  constexpr void remove_prefix(size_type n) {
    assert(n <= size_);
    string_ += n;
    size_ -= n;
  }

  /**
   * Moves the end of the view back by n characters
   *
   * @param n number of characters
   */
  constexpr void remove_suffix(size_type n) {
    assert(n <= size_);
    size_ -= n;
  }

  /**
   * Exchanges the view with that of other
   *
   * @param other basic string view
   */
  constexpr void swap(BasicStringView &other) noexcept {
    std::swap(string_, other.string_);
    std::swap(size_, other.size_);
  }

  /**
   * Copies the string view onto the buffer
   *
   * @note method will assert if @p index > size().
   *
   * @param buffer pointer to the destination buffer
   * @param max_count maximum number of characters to copy across to buffer
   * @param index index position of the first character from the basic string
   * view
   *
   * @return number of characters copied
   */
  constexpr size_type copy(CharT *buffer, size_type max_count,
                           size_type index = 0) const {
    assert(index <= size());

    size_type count = std::min(max_count, size_ - index);
    Traits::copy(buffer, string_ + index, count);
    return count;
  }

  /**
   * Return a substring view of the view
   *
   * @note method will assert if @p index > size().
   *
   * @param index position of the first character
   * @param count number of characters within substring
   * @return substring view
   */
  constexpr BasicStringView substr(
      size_type index = 0,
      size_type count = std::numeric_limits<size_type>::max()) const {
    assert(index <= size());
    return BasicStringView(string_ + index, std::min(count, size_ - index));
  }

  /**
   * Performs comparison operation on other
   *
   * @param other basic string view to compare against
   *
   * @return negative value if this view is less than the other character
   * sequence, zero if the both character sequences are equal, positive value if
   * this view is greater than the other character sequence.
   */
  constexpr int compare(BasicStringView other) const noexcept {
    const int result =
        Traits::compare(string_, other.string_, std::min(size_, other.size_));

    if (result != 0) {
      return result;
    }

    return size_ == other.size_ ? 0 : (size_ < other.size_ ? -1 : 1);
  }

 private:
  const CharT *string_ = nullptr;
  size_type size_ = 0;
};

template <class CharT, class Traits>
constexpr bool operator==(BasicStringView<CharT, Traits> lhs,
                          BasicStringView<CharT, Traits> rhs) noexcept {
  return lhs.size() != rhs.size() ? false : lhs.compare(rhs) == 0;
}

template <class CharT, class Traits>
constexpr bool operator!=(BasicStringView<CharT, Traits> lhs,
                          BasicStringView<CharT, Traits> rhs) noexcept {
  return lhs.size() != rhs.size() ? true : lhs.compare(rhs) != 0;
}

template <class CharT, class Traits>
constexpr bool operator<(BasicStringView<CharT, Traits> lhs,
                         BasicStringView<CharT, Traits> rhs) noexcept {
  return lhs.compare(rhs) < 0;
}

template <class CharT, class Traits>
constexpr bool operator>(BasicStringView<CharT, Traits> lhs,
                         BasicStringView<CharT, Traits> rhs) noexcept {
  return lhs.compare(rhs) > 0;
}

template <class CharT, class Traits>
constexpr bool operator<=(BasicStringView<CharT, Traits> lhs,
                          BasicStringView<CharT, Traits> rhs) noexcept {
  return lhs.compare(rhs) <= 0;
}

template <class CharT, class Traits>
constexpr bool operator>=(BasicStringView<CharT, Traits> lhs,
                          BasicStringView<CharT, Traits> rhs) noexcept {
  return lhs.compare(rhs) >= 0;
}

using StringView = BasicStringView<char>;

}  // namespace pal

#endif  // LIBPAL_CPP_STRING_VIEW_H
