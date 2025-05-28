#ifndef LIBPAL_CPP_VECTOR_H
#define LIBPAL_CPP_VECTOR_H

#include <cassert>
#include <iterator>
#include <limits>
#include <type_traits>
#include <utility>

#include <libpal++/size.h>  // pal::size can be imported via header file
#include <libpal/mem/mem.h>

namespace pal {
/**
 * Class offers the ability for users to dynamically create an array much in the
 * same way that std::vector does. There are a number of key differences between
 * the two classes, this class:
 *
 *   - uses the platform abstraction layer for memory allocation/deallocation
 *   - offers an API that avoids throwing any exception related to platform
 *     errors
 *   - does not automatically resize the array when attempting to push onto a
 *     full array, in fact it prevents the resizing of the array entirely once
 *     it is set.
 *
 * The philosophy behind the last bullet point was to impede developers from
 * wildly coding in such a way that doesn't account of the embedded nature of
 * the platform, and potentially developing less than optimal software.
 *
 * For those that are familiar with the std::vector, the above bullet points are
 * achieved by:
 *
 *   - removing all the various std::vector parameterized constructor and moving
 *     them into an appropriate init() function
 *   - removing the following methods that automatically resize the underlying
 *     array and or throw an exception
 *
 * @code
 * void resize (size_type n);
 * void resize (size_type n, const value_type& val);
 * void shrink_to_fit();
 * reference at (size_type n);
 * const_reference at (size_type n) const;
 * template <class InputIterator>
 * void assign (InputIterator first, InputIterator last);
 * void assign (size_type n, const value_type& val);
 * void assign (initializer_list<value_type> il);
 * @endcode
 *
 * There are still a number of methods that still need to be completed which
 * haven't been due to time constraints in developing this class, below we
 * outline them:
 *
 * @code
 * iterator insert (const_iterator position, const value_type& val);
 * iterator insert (const_iterator position, size_type n, const_reference val);
 * template <class InputItr>
 * iterator insert (const_iterator position, InputItr first, InputItr last);
 * iterator insert (const_iterator position, value_type&& val);
 * iterator insert (const_iterator position, initializer_list<value_type> il);
 * iterator erase (const_iterator position);
 * iterator erase (const_iterator first, const_iterator last);
 * template <class... Args>
 * iterator emplace (const_iterator position, Args&&... args);
 * @endcode
 *
 * For all std::vector methods that have the potential to updated the vector
 * size, this class has changed its API such that it returns a pal_error
 * notifying that it was unable to do the operation without resizing the array.
 *
 * @note this class should work with the standard C++ algorithm library.
 * @note the above member function lists haven't been fully investigated, so
 * there is the possibility of changing it.
 *
 * @tparam T data type for which the container will manage
 */
template <typename T>
class Vector final {
 public:
  using value_type = T;
  using size_type = size_t;
  using difference_type = ptrdiff_t;
  using reference = T &;
  using const_reference = const T &;
  using pointer = T *;
  using const_pointer = const T *;
  using iterator = pointer;
  using const_iterator = const_pointer;
  using reverse_iterator = std::reverse_iterator<iterator>;
  using const_reverse_iterator = std::reverse_iterator<const_iterator>;

 public:
  Vector() noexcept : size_(0), capacity_(0), data_(nullptr) {}

  Vector(const Vector &other) noexcept = delete;
  Vector(Vector &&other) noexcept : Vector() { swap(other); }

  Vector &operator=(const Vector &other) noexcept = delete;
  Vector &operator=(Vector &&other) noexcept {
    swap(other);
    return *this;
  }

  ~Vector() { deinit(); }

  /**
   * Create an array of the specified capacity. There will be no entries inside
   * the array (ie: empty() == true), to fill the array with content please use
   * init(size_t, const_reference).
   *
   * @param capacity number of elements for which the array will be capable to
   * hold
   * @return platform error code. PAL_INVALID if the class has already been
   * initialized of if the capacity is set to zero.
   */
  pal_error init(size_type capacity) noexcept {
    if (data_ != nullptr) {
      return PAL_INVALID;
    }

    if (capacity == 0) {
      return PAL_INVALID;
    }

    void *data;
    pal_error error;

    error = pal_mem_alloc(&data, sizeof(value_type) * capacity);

    if (error != PAL_SUCCESS) {
      return error;
    }

    size_ = 0;
    capacity_ = capacity;
    data_ = static_cast<pointer>(data);

    return PAL_SUCCESS;
  }

  /**
   * Create an array of @p size filled with copies of @p value.
   *
   * @param size number of elements held by array
   * @param value value to copy onto every array slot
   * @return platform error code. PAL_INVALID if the class has already been
   * initialized of if the capacity is set to zero.
   */
  pal_error init(size_type size, const_reference value) noexcept(
      std::is_nothrow_copy_constructible<value_type>().value) {
    pal_error error = init(size);

    if (error != PAL_SUCCESS) {
      return error;
    }

    while (size_ < size) {
      new (&data_[size_]) value_type(value);
      ++size_;
    }

    return PAL_SUCCESS;
  }

  /**
   * Creates an array that holds a copy of the content presented in the range of
   * [ @p first, @p last ).
   *
   * @tparam InputIterator iterator type
   * @param first iterator to the start of the source container
   * @param last iterator to the end fo the source container
   * @return platform error code. PAL_INVALID if the class has already been
   * initialized of if the capacity is set to zero.
   */
  template <typename InputIterator>
  pal_error init(InputIterator first,
                 InputIterator last) noexcept(noexcept(value_type(*first))) {
    auto distance = std::distance(first, last);
    if (distance <= 0) {
      return PAL_INVALID;
    }

    pal_error error = init(static_cast<size_type>(distance));

    if (error != PAL_SUCCESS) {
      return error;
    }

    auto itr = first;

    while (itr != last) {
      new (&data_[size_]) value_type(*itr);
      ++size_;
      ++itr;
    }

    return PAL_SUCCESS;
  }

  /**
   * Performed the inverse operation of whatever the init() functions have
   * performed.
   *
   * @return platform error code. PAL_INVALID if the class hasn't been
   * initialized.
   */
  pal_error deinit() noexcept(
      std::is_nothrow_destructible<value_type>().value) {
    if (data_ == nullptr) {
      return PAL_SUCCESS;
    }

    clear();

    void *data = data_;
    pal_error error = pal_mem_free(&data);

    size_ = 0;
    capacity_ = 0;
    data_ = nullptr;

    return error;
  }

  /**
   * @return an iterator to the first element
   */
  iterator begin() noexcept { return iterator(data_); }

  /**
   * @return an iterator to the first element
   */
  const_iterator begin() const noexcept { return cbegin(); }

  /**
   * @return an iterator to the first element
   */
  const_iterator cbegin() const noexcept { return const_iterator(data_); }

  /**
   * @return an iterator to the element following the last element
   */
  iterator end() noexcept { return iterator(&data_[size_]); }

  /**
   * @return an iterator to the element following the last element
   */
  const_iterator end() const noexcept { return cend(); }

  /**
   * @return an iterator to the element following the last element
   */
  const_iterator cend() const noexcept { return const_iterator(&data_[size_]); }

  /**
   * @return reverse iterator to the first element
   */
  reverse_iterator rbegin() noexcept { return reverse_iterator(end()); }

  /**
   * @return reverse iterator to the first element
   */
  const_reverse_iterator rbegin() const noexcept {
    return const_reverse_iterator(crbegin());
  }

  /**
   * @return reverse iterator to the first element
   */
  const_reverse_iterator crbegin() const noexcept {
    return const_reverse_iterator(cend());
  }

  /**
   * @return reverse iterator to the element following the last element
   */
  reverse_iterator rend() noexcept { return reverse_iterator(begin()); }

  /**
   * @return reverse iterator to the element following the last element
   */
  const_reverse_iterator rend() const noexcept {
    return const_reverse_iterator(crend());
  }

  /**
   * @return reverse iterator to the element following the last element
   */
  const_reverse_iterator crend() const noexcept {
    return const_reverse_iterator(cbegin());
  }

  /**
   * @return number of elements in the array
   */
  size_type size() const noexcept { return size_; }

  /**
   * @return maximum possible number of elements in an array
   */
  size_type max_size() const noexcept {
    return std::numeric_limits<size_type>::max();
  }

  /**
   * @return capacity of the array
   */
  size_type capacity() const noexcept { return capacity_; }

  /**
   * @return true if the container is empty, false otherwise
   */
  bool empty() const noexcept { return size_ == 0; }

  /**
   * @note this class asserts in developer builds if the index requested is past
   * the array size.
   *
   * @param index index of the element to return
   * @return reference to the requested element
   */
  reference operator[](size_type index) {
    assert(index < size_);
    return data_[index];
  }

  /**
   * @note this class asserts in developer builds if the index requested is past
   * the array size.
   *
   * @param index index of the element to return
   * @return reference to the requested element
   */
  const_reference operator[](size_type index) const {
    assert(index < size_);
    return data_[index];
  }

  /**
   * @note this class asserts in developer builds if the array is empty.
   *
   * @return reference to the first element
   */
  reference front() {
    assert(!empty());
    return data_[0];
  }

  /**
   * @note this class asserts in developer builds if the array is empty.
   *
   * @return reference to the first element
   */
  const_reference front() const {
    assert(!empty());
    return data_[0];
  }

  /**
   * @note this class asserts in developer builds if the array is empty.
   *
   * @return reference to the last element
   */
  reference back() {
    assert(!empty());
    return data_[size_ - 1];
  }

  /**
   * @note this class asserts in developer builds if the array is empty.
   *
   * @return reference to the last element
   */
  const_reference back() const {
    assert(!empty());
    return data_[size_ - 1];
  }

  /**
   * @return pointer to the underlying array, nullptr if the array is empty
   */
  pointer data() noexcept { return data_; }

  /**
   * @return pointer to the underlying array, nullptr if the array is empty
   */
  const_pointer data() const noexcept { return data_; }

  /**
   * Appends the given element value to the end of the container.
   *
   * @param value value of the element to append
   * @return PAL_INVALID if the array is already full, otherwise PAL_SUCCESS.
   */
  pal_error push_back(const value_type &value) noexcept(
      std::is_nothrow_copy_constructible<value_type>().value) {
    if (size_ == capacity_) {
      return PAL_INVALID;
    }

    new (&data_[size_]) value_type(value);
    ++size_;

    return PAL_SUCCESS;
  }

  /**
   * Appends the given element value to the end of the container.
   *
   * @param value value of the element to append
   * @return PAL_INVALID if the array is already full, otherwise PAL_SUCCESS
   */
  pal_error push_back(value_type &&value) noexcept(
      std::is_nothrow_move_constructible<value_type>().value) {
    if (size_ == capacity_) {
      return PAL_INVALID;
    }

    new (&data_[size_]) value_type(std::move(value));
    ++size_;

    return PAL_SUCCESS;
  }

  /**
   * Removes the last element of the container.
   *
   * @return PAL_INVALID if the array is empty, otherwise PAL_SUCCESS
   */
  pal_error pop_back() noexcept(
      std::is_nothrow_destructible<value_type>().value) {
    if (empty()) {
      return PAL_INVALID;
    }

    data_[size_ - 1].~value_type();
    --size_;

    return PAL_SUCCESS;
  }

  /**
   * Exchanges the contents of the container with those of other
   *
   * @param other vector to exchange the contents with
   */
  void swap(Vector &other) noexcept {
    std::swap(size_, other.size_);
    std::swap(capacity_, other.capacity_);
    std::swap(data_, other.data_);
  }

  /**
   * Erases all elements from the array.
   */
  void clear() noexcept(std::is_nothrow_destructible<value_type>().value) {
    while (size_ > 0) {
      data_[size_ - 1].~value_type();
      --size_;
    }
  }

  /**
   * Appends a new element to the end of the container.
   *
   * @tparam Args argument types to forward to the constructor of the element
   * @param args arguments to forward to the constructor of the element
   * @return PAL_INVALID if the array is already full, otherwise PAL_SUCCESS
   */
  template <typename... Args>
  pal_error emplace_back(Args &&... args) noexcept(
      std::is_nothrow_constructible<value_type, Args &&...>().value) {
    if (size_ == capacity_) {
      return PAL_INVALID;
    }

    new (&data_[size_]) value_type(std::forward<Args>(args)...);
    ++size_;

    return PAL_SUCCESS;
  }

 private:
  size_type size_;
  size_type capacity_;
  pointer data_;
};
}  // namespace pal

#endif  // LIBPAL_CPP_VECTOR_H
