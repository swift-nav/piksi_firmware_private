#ifndef PVT_COMMON_CONTAINERS_INDEXED_ENUM_LOOKUP_TABLE_H
#define PVT_COMMON_CONTAINERS_INDEXED_ENUM_LOOKUP_TABLE_H

#include <bitset>
#include <cstddef>
#include <initializer_list>
#include <limits>
#include <type_traits>

#include <pvt_common/optional.h>

namespace pvt_common {
namespace containers {

/**
 * Lookup table that is optimized for types whose key values are index values.
 * What that means is that if you have an enumerator type as such:
 *
 * \code
 *  enum Color {
 *    INVALID = -1,
 *    RED,
 *    GREEN,
 *    BLUE,
 *    COUNT
 *  };
 * \endcode
 *
 * This class will operate in much the same way that pvt_common::containers::Map
 * would, but in a much more efficient manner. One can populate the the table
 * using either:
 *
 * \code
 *  LookupTable<Color, const char*, Color::COUNT> lut;
 *  lut.insert(RED, "Red");
 *  lut.insert(GREEN, "Green");
 *  lut.insert(BLUE, "Blue");
 * \endcode
 *
 * or
 *
 * \code
 *  LookupTable<Color, const char*, Color::COUNT> lut = {
 *   {RED, "Red"},
 *   {GREEN, "Green"},
 *   {BLUE, "Blue"},
 *  };
 * \endcode
 *
 * When trying to iterate through the table, the iterator will point to the
 * underlying key which have values:
 *
 * \code
 *  for (auto key : lut) {
 *    std::cout << "Key: " << static_cast<int>(key) << std::endl;
 *  }
 * \endcode
 *
 * As a summary the performance for various operations are listed below:
 *
 *  * insert: O(1)
 *  * remove: O(1)
 *  * iterate: O(N) (for both best and worse case scenario)
 *
 * \tparam K key type (must be convertable to ptrdiff_t value and vice versa)
 * \tparam V value type (must be trivialially deconstructable, this requirements
 * was set to mirror that of pvt_common::containers::Map)
 * \tparam Size max number of enumerators keys that will be fit in the map
 */
template <typename K, typename V, size_t Size>
class LookupTable {
  static_assert(std::is_convertible<K, ptrdiff_t>::value,
                "Key type must be convertable to ptrdiff_t value");
  static_assert(std::is_trivially_destructible<V>::value,
                "Value type must be trivialially deconstructable");
  static_assert(Size > 0, "Size must be greater than zero");
  static_assert(Size <= std::numeric_limits<ptrdiff_t>::max(),
                "Maximum size is too large for the container to handle");

 public:
  class Iterator {
   public:
    using iterator_category = std::bidirectional_iterator_tag;
    using value_type = K;
    using difference_type = ptrdiff_t;
    using pointer = value_type;
    using reference = value_type;

   public:
    Iterator() : Iterator(nullptr, 0) {}
    Iterator(const LookupTable *base, ptrdiff_t offset)
        : base_(base), offset_(offset) {}

    Iterator(const Iterator &other) = default;
    Iterator &operator=(const Iterator &other) = default;

    bool operator==(const Iterator &other) const {
      return base_ == other.base_ && offset_ == other.offset_;
    }

    bool operator!=(const Iterator &other) const {
      return !this->operator==(other);
    }

    // please note that the `reference` and `pointer` type are not actually
    // reference or pointers, they are mapped to `value_type`. this is done to
    // conform to the `std::iterator_traits` concept.
    reference operator*() const { return static_cast<K>(offset_); }
    pointer operator->() const { return static_cast<K>(offset_); }

    Iterator &operator++() {
      if (base_ != nullptr && offset_ < static_cast<ptrdiff_t>(Size)) {
        do {
          ++offset_;
        } while (offset_ < static_cast<ptrdiff_t>(Size) &&
                 !base_->contains(static_cast<K>(offset_)));
      }
      return *this;
    }

    Iterator operator++(int) {
      auto temp = *this;
      this->operator++();
      return temp;
    }

    Iterator &operator--() {
      if (base_ != nullptr && offset_ >= 0) {
        do {
          --offset_;
        } while (offset_ >= 0 && !base_->contains(static_cast<K>(offset_)));
      }
      return *this;
    }

    Iterator operator--(int) {
      auto temp = *this;
      this->operator--();
      return temp;
    }

   private:
    const LookupTable *base_;
    ptrdiff_t offset_;
  };

 public:
  using iterator = Iterator;
  using const_iterator = Iterator;
  using reverse_iterator = std::reverse_iterator<iterator>;
  using const_reverse_iterator = std::reverse_iterator<const_iterator>;

 public:
  /**
   * Default Constructor
   */
  LookupTable() : values_(), has_value_(), count_(0) {}

  /**
   * List Constructor
   *
   * @param entries list of key -> value entries to add to the table
   */
  LookupTable(std::initializer_list<std::pair<K, V>> entries) : LookupTable() {
    for (const auto &entry : entries) {
      assert_key_validate(entry.first);
      new (&lookup(entry.first)) V(entry.second);
      has_value_[static_cast<size_t>(entry.first)] = true;
      ++count_;
    }
  }

  /**
   * Copy Constructor
   *
   * @param other object to copy from
   */
  LookupTable(const LookupTable &other) : LookupTable() {
    for (size_t i = 0; i < Size; ++i) {
      new (&lookup(static_cast<K>(i))) V(other.lookup(static_cast<K>(i)));
    }
    has_value_ = other.has_value_;
    count_ = other.count_;
  }

  /**
   * Copy Assignment
   *
   * @param other object to copy from
   * @return self
   */
  LookupTable &operator=(const LookupTable &other) {
    for (size_t i = 0; i < Size; ++i) {
      new (&lookup(static_cast<K>(i))) V(other.lookup(static_cast<K>(i)));
    }
    has_value_ = other.has_value_;
    count_ = other.count_;
    return *this;
  }

  /**
   * Value getter.
   *
   * @note function asserts if no value is associated with key.
   *
   * @param key key value to lookup
   * @return value for that key
   */
  V &at(K key) & {
    assert_key_validate(key);
    assert(contains(key));
    return lookup(key);
  }

  /**
   * Const value getter.
   *
   * @note function asserts if no value is associated with key.
   *
   * @param key key value to lookup
   * @return value for that key
   */
  const V &at(K key) const & {
    assert_key_validate(key);
    assert(contains(key));
    return lookup(key);
  }

  /**
   * Value getter.
   *
   * @note function asserts if no value is associated with key.
   *
   * @param key key value to lookup
   * @return value for that key
   */
  V at(K key) && {
    assert_key_validate(key);
    assert(contains(key));
    return lookup(key);
  }

  /**
   * Value getter.
   *
   * @param key key value to lookup
   * @return value for that key
   */
  optional<V &> operator[](K key) & {
    if (!is_valid_key(key)) {
      return {};
    }
    if (contains(key)) {
      return lookup(key);
    }
    return {};
  }

  /**
   * Const value getter.
   *
   * @param key key value to lookup
   * @return value for that key
   */
  optional<const V &> operator[](K key) const & {
    if (!is_valid_key(key)) {
      return {};
    }
    if (contains(key)) {
      return lookup(key);
    }
    return {};
  }

  /**
   * R instance value getter.
   *
   * @param key key value to lookup
   * @return value for that key
   */
  optional<V> operator[](K key) && {
    if (!is_valid_key(key)) {
      return {};
    }
    if (contains(key)) {
      return lookup(key);
    }
    return {};
  }

  /**
   * Number of entries in table..
   *
   * @return size of table
   */
  size_t size() const { return count_; }

  /**
   * Maximum number of entries in table.
   *
   * @return size of table
   */
  size_t max_size() const { return Size; }

  /**
   * Tests if the key has an associated value with it.
   *
   * @param key key to test
   * @return true if an associated value exists for it
   */
  bool contains(K key) const {
    assert_key_validate(key);
    return has_value_[static_cast<size_t>(key)];
  }

  /**
   * Empties out all key/value pairs
   */
  void clear() {
    has_value_.reset();
    count_ = 0;
  }

  /**
   * Adds key value pair to the table
   *
   * @param key key entry
   * @param value value entry
   * @return self
   */
  LookupTable &insert(K key, const V &value) {
    assert_key_validate(key);

    new (&lookup(key)) V(value);
    size_t index = static_cast<size_t>(key);
    if (!has_value_[index]) {
      has_value_[index] = true;
      ++count_;
    }

    return *this;
  }

  /**
   * Adds key value pair to the table
   *
   * @param key key entry
   * @param value value entry
   * @return self
   */
  LookupTable &insert(K key, V &&value) {
    assert_key_validate(key);

    new (&lookup(key)) V(std::move(value));
    size_t index = static_cast<size_t>(key);
    if (!has_value_[index]) {
      has_value_[index] = true;
      ++count_;
    }

    return *this;
  }

  /**
   * Remove key value pair from the table
   *
   * @param key key entry
   */
  LookupTable &remove(K key) {
    assert_key_validate(key);
    size_t index = static_cast<size_t>(key);
    if (has_value_[index]) {
      has_value_[index] = false;
      --count_;
    }
    return *this;
  }

  /**
   * Checks if lookup table is equal to another one. Unlike the
   * operator==(const LookupTable &other) which uses the global equality
   * operator on the values, this function allows you to specify a custom
   * function.
   *
   * @param other other lookup table to compare against
   * @param is_equal callback function to check if values are identical. the
   * first parameter addresses the key in question, where as the left and right
   * correspond the value associate with this lookup table and the other
   * respectively
   * @return true if both tables have identical keys and their respective values
   * are equal based on the callback function
   */
  bool is_equal(const LookupTable &other,
                bool (*is_value_equal)(K key, const V &left,
                                       const V &right)) const {
    assert(is_value_equal);

    if (count_ != other.count_) {
      return false;
    }

    if (has_value_ != other.has_value_) {
      return false;
    }

    for (size_t i = 0; i < Size; ++i) {
      K key = static_cast<K>(i);
      if (has_value_[i] &&
          !is_value_equal(key, lookup(key), other.lookup(key))) {
        return false;
      }
    }

    return true;
  }

  /**
   * @return an iterator to the first element
   */
  iterator begin() noexcept { return ++iterator(this, -1); }

  /**
   * @return an iterator to the first element
   */
  const_iterator begin() const noexcept { return cbegin(); }

  /**
   * @return an iterator to the first element
   */
  const_iterator cbegin() const noexcept { return ++const_iterator(this, -1); }

  /**
   * @return an iterator to the element following the last element
   */
  iterator end() noexcept { return iterator(this, Size); }

  /**
   * @return an iterator to the element following the last element
   */
  const_iterator end() const noexcept { return cend(); }

  /**
   * @return an iterator to the element following the last element
   */
  const_iterator cend() const noexcept { return const_iterator(this, Size); }

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
   * Equality comparison operator.
   *
   * @param other other lookup table to compare against
   * @return true if both are identical
   */
  bool operator==(const LookupTable &other) const {
    if (count_ != other.count_) {
      return false;
    }

    if (has_value_ != other.has_value_) {
      return false;
    }

    for (size_t i = 0; i < Size; ++i) {
      K key = static_cast<K>(i);
      if (has_value_[i] && lookup(key) != other.lookup(key)) {
        return false;
      }
    }

    return true;
  }

  /**
   * Inequality comparison operator.
   *
   * @param other other lookup table to compare against
   * @return true if both are different
   */
  bool operator!=(const LookupTable &other) const {
    return !this->operator==(other);
  }

 private:
  /**
   * Function performs assertion validation on the user input key value.
   */
  void assert_key_validate(K key) const {
    (void)key;
    assert(static_cast<ptrdiff_t>(key) >= 0 && "Key index value is below zero");
    assert(static_cast<ptrdiff_t>(key) < static_cast<ptrdiff_t>(Size) &&
           "Key index value is higher than capacity");
  }

  bool is_valid_key(K key) const {
    const bool is_above_zero = static_cast<ptrdiff_t>(key) >= 0;
    const bool is_within_capacity =
        static_cast<ptrdiff_t>(key) < static_cast<ptrdiff_t>(Size);
    return (is_above_zero && is_within_capacity);
  }

  /**
   * Function is equivalent to at(K key), however it bypasses the assertion.
   *
   * @param key key value to lookup
   * @return value for that key
   */
  V &lookup(K key) {
    return static_cast<V *>(static_cast<void *>(values_))[key];
  }

  /**
   * Function is equivalent to const at(K key), however it bypasses the
   * assertion.
   *
   * @param key key value to lookup
   * @return value for that key
   */
  const V &lookup(K key) const {
    return static_cast<const V *>(static_cast<const void *>(values_))[key];
  }

 private:
  typename std::aligned_storage<sizeof(V), alignof(V)>::type values_[Size];
  std::bitset<Size> has_value_;
  size_t count_;
};

}  // namespace containers
}  // namespace pvt_common
#endif  // PVT_COMMON_CONTAINERS_INDEXED_ENUM_LOOKUP_TABLE_H
