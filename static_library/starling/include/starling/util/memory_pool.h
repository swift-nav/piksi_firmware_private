/**
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_UTIL_MEMORYPOOL_H
#define STARLING_UTIL_MEMORYPOOL_H

#include <swiftnav/logging.h>

#include <memory>
#include <new>
#include <type_traits>

namespace starling {
namespace util {

/**
 * A class that manages a chunk of memory as a pool for one specific type.
 * Allocating is constant time, but freeing is linear (worst case, could make
 * constant at the risk of UB).
 *
 * You must first initialize the pool with a chunk of memory for it to use as
 * it's pool.
 *
 * `Alloc()` constructs the object before returning a pointer to it
 *
 * `Free()` double checks that the given pointer has been allocated from this
 * pool before calling the object's destructor and returning the memory to the
 * pool.
 *
 * Note: `T` must be default constructable currently, for simplicity.
 *
 * The MemoryPool object treats each space in the pool as either an object of
 * type T, or as an element of a singly linked list. The linked list consists of
 * all spaces in the pool that have not been allocated yet. When a new object is
 * allocated from the pool the first item in the linked list is removed from the
 * list and a new object of type T is constructed at that segment of memory.
 * When an object is freed back into the pool the object's destructor is called,
 * and then that segment of memory is turned into an element of the linked list
 * and placed at the front of the list.
 */
template <typename T>
class MemoryPool {
  static_assert(std::is_default_constructible<T>::value,
                "T must be default constructible");

 public:
  using pointer = T *;
  using value_type = T;

 protected:
  union Node {
    Node *next;
    value_type value;
  };

  std::size_t size_;
  Node *pool_;
  Node *unallocated_list_head_;
  const char *name_;

 public:
  /**
   * Gets the size of the nodes managed by MemoryPool.
   * This is useful for allocating buffers for MemoryPool
   * since the size of the nodes aren't guaranteed to be
   * equal to `sizeof(T)`.
   */
  static constexpr std::size_t node_size = sizeof(Node);

  /**
   * Gets the alignment of the nodes managed by MemoryPool.
   * This is useful for allocating buffers for MemoryPool
   * since the alignment of the nodes aren't guaranteed to be
   * equal to `alignof(T)`.
   */
  static constexpr std::size_t node_alignment = alignof(Node);

  /**
   * Constructs an instance of `MemoryPool<T>` that doesn't manage any memory
   * yet.
   */
  MemoryPool() noexcept
      : size_(0),
        pool_(nullptr),
        unallocated_list_head_(nullptr),
        name_(nullptr) {}

  /**
   * Destroys an instance of `MemoryPool<T>`
   *
   * Note: The destructor doesn't automatically free an allocated elements,
   * you must call `Free()` on all allocated elements before destroying the
   * MemoryPool.
   */
  ~MemoryPool() {
    // TODO(jangelo): Do any of our targets need us to free the memory we hold?
  }

  MemoryPool(const MemoryPool &) = delete;
  MemoryPool(MemoryPool &&) = delete;
  MemoryPool &operator=(const MemoryPool &) = delete;
  MemoryPool &operator=(MemoryPool &&) = delete;

  /**
   * Initialize the memory pool with a buffer to manage.
   *
   * This should only be called successfully once, after a successful
   * initialization all other calls will fail.
   *
   * buffer_size is the size of the buffer in bytes
   *
   * The memory doesn't need to be aligned properly, but if it isn't some of the
   * space won't be usable.
   *
   * Returns true upon successful initialization, false if an error as occured
   */
  bool Init(void *buffer, const std::size_t buffer_size,
            const char *name) noexcept {
    if (0 != size_ || nullptr != pool_) {
      log_error(
          "MemoryPool: Already been initialized, failing the second attempt");
      return false;
    }

    name_ = name;

    size_t aligned_size = buffer_size;
    void *aligned_buffer =
        std::align(alignof(Node), sizeof(Node), buffer, aligned_size);

    if (nullptr == aligned_buffer) {
      log_error(
          "MemoryPool %s: Provided memory pool is to small to properly align!",
          name_);
      return false;
    }
    if (buffer_size != aligned_size) {
      size_t difference = buffer_size - aligned_size;
      log_error("MemoryPool %s: Lost %zu bytes in a misaligned buffer.", name_,
                difference);
    }

    size_ = aligned_size / sizeof(Node);
    pool_ = static_cast<Node *>(aligned_buffer);
    unallocated_list_head_ = nullptr;

    for (std::size_t i = 0; i < size_; ++i) {
      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-union-access)
      pool_[i].next = unallocated_list_head_;
      unallocated_list_head_ = &pool_[i];
    }

    return true;
  }

  /**
   * Allocates an instance of `T` from the pool, if there is any space available
   *
   * Returns a pointer to the constructed object, or `nullptr` if no space is
   * available
   */
  pointer Alloc() noexcept(noexcept(value_type())) {
    Node *current = unallocated_list_head_;
    pointer object = nullptr;

    if (nullptr != current) {
      unallocated_list_head_ = current->next;
      object = new (&current->value) value_type;
    }

    return object;
  }

  /**
   * Attempts to return memory to the pool, and destroys the object in that
   * memory
   *
   * If `element` doesn't point to an allocated element in the pool this
   * function immediately returns with out doing anything.
   */
  void Free(void *element) noexcept(
      noexcept(std::declval<value_type>().~value_type())) {
    if (nullptr == element) {
      return;
    }
    if ((element < &pool_[0]) || (element >= &pool_[size_])) {
      log_error("MemoryPool %s: Trying to free memory outside the pool (%p)",
                name_, element);
      return;
    }

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    Node *node = reinterpret_cast<Node *>(element);

    if (!IsAllocated(node)) {
      log_error("MemoryPool %s: Trying to free unallocated memory.", name_);
      return;
    }

    node->value.~T();

    node->next = unallocated_list_head_;
    unallocated_list_head_ = node;
  }

 protected:
  /**
   * Checks to see if `element` points to a node in the unallocated linked list
   */
  bool IsAllocated(const Node *const element) const noexcept {
    for (const Node *current = unallocated_list_head_; nullptr != current;
         current = current->next) {
      if (current == element) {
        return false;
      }
    }
    return true;
  }
};

}  // namespace util
}  // namespace starling

#endif /* STARLING_UTIL_MEMORYPOOL_H */
