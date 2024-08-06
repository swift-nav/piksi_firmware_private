#ifndef LIBPAL_CPP_MEMORY_POOL_H
#define LIBPAL_CPP_MEMORY_POOL_H

#include <libpal++/condition_variable.h>
#include <libpal++/mutex.h>
#include <libpal++/scoped_lock.h>
#include <libpal/error.h>
#include <libpal/mem/mem.h>

#include <atomic>
#include <cassert>
#include <new>
#include <utility>

namespace pal {
namespace detail {
template <typename T>
bool is_valid_memory_block_location(const T *pool, const T *memory_block) {
  return (reinterpret_cast<uintptr_t>(memory_block) -  // NOLINT
          reinterpret_cast<uintptr_t>(pool)) %         // NOLINT
             sizeof(T) ==
         0;
}
}  // namespace detail
/**
 * Offers users the ability to create a memory pool.
 *
 * Unlike SharedMemoryPool, those that call MemoryPool::alloc<T**, Args...>(Args
 * &&... args) to request a memory block will be the sole owner of the allocated
 * data type.
 *
 * Performance wise, this class is O(1) for both allocation and deallocation.
 *
 * @note This class IS NOT thread safe.
 * @note If a memory pool is deconstructed while there are still some allocated
 * spaces in the memory pool, than the memory pool will be deemed corrupted
 * and as such will crash in a developer build.
 *
 * @tparam T Data type stored in the memory pool
 */
template <typename T>
class MemoryPool final {
 public:
  /**
   * This class acts much the same way that std::unique_ptr<T> does. The
   * difference is that this class explicitly references a memory block within
   * a MemoryPool.
   *
   * Class attempts to use much the same API that std::unique_ptr uses for
   * easy adoption by users.
   */
  class UniquePointer final {
    friend class MemoryPool;

   public:
    /**
     * Constructor to a empty unique pointer.
     */
    UniquePointer() noexcept : UniquePointer(nullptr, nullptr) {}

    UniquePointer(const UniquePointer &other) = delete;
    UniquePointer &operator=(const UniquePointer &other) noexcept = delete;

    UniquePointer(UniquePointer &&other) noexcept
        : pool_(other.pool_), memory_block_(other.memory_block_) {
      other.pool_ = nullptr;
      other.memory_block_ = nullptr;
    }

    UniquePointer &operator=(UniquePointer &&other) noexcept {
      if (&other != this) {
        reset();

        pool_ = other.pool_;
        memory_block_ = other.memory_block_;

        other.pool_ = nullptr;
        other.memory_block_ = nullptr;
      }

      return *this;
    }

    ~UniquePointer() { reset(); }

    /**
     * Relinquish control of the managed pointer
     *
     * @return the managed pointer
     */
    T *release() noexcept {
      T *memory_block = memory_block_;

      pool_ = nullptr;
      memory_block_ = nullptr;

      return memory_block;
    }

    /**
     * Destroys the managed pointer
     */
    void reset() noexcept {
      if (pool_ == nullptr) {
        return;
      }

      pool_->free(memory_block_);

      pool_ = nullptr;
      memory_block_ = nullptr;
    }

    /**
     * @return the managed pointer
     */
    T *get() const noexcept { return memory_block_; }

    /**
     * @return reference to the managed pointer
     */
    T &operator*() const noexcept { return *get(); }

    /**
     * @return the managed pointer
     */
    T *operator->() const noexcept { return get(); }

    /**
     * @return true if the class currently manages a memory block from the
     * memory pool, otherwise false
     */
    explicit operator bool() const noexcept { return get() != nullptr; }

   private:
    /**
     * Constructs a unique pointer that manages the specified memory block from
     * a memory pool.
     *
     * @param pool memory pool which holds the memory block
     * @param memory_block memory block which will be managed
     */
    UniquePointer(MemoryPool<T> *pool, T *memory_block) noexcept
        : pool_(pool), memory_block_(memory_block) {}

   private:
    MemoryPool<T> *pool_;
    T *memory_block_;
  };

 public:
  MemoryPool() noexcept
      : size_(0),
        available_(0),
        free_(nullptr),
        pool_(nullptr),
        tracker_(nullptr) {}

  MemoryPool(const MemoryPool<T> &) noexcept = delete;
  MemoryPool(MemoryPool<T> &&) noexcept = delete;

  MemoryPool<T> &operator=(const MemoryPool<T> &) noexcept = delete;
  MemoryPool<T> &operator=(MemoryPool<T> &&) noexcept = delete;

  /**
   * @see MemoryPool
   */
  ~MemoryPool() {
    assert(deinit() != PAL_ERROR &&
           "Memory pools is being deconstructed while there are still memory "
           "blocks which have not been explicitly deallocated");
  }

  /**
   * @return number of memory blocks that have been allocated for the memory
   * pool
   */
  size_t size() const noexcept { return size_; }

  /**
   * @return number of memory blocks still available from the memory pool
   */
  size_t available() const noexcept { return available_; }

  /**
   * Creates the memory pool, as in it requests resources from the platform
   * for which it will then manage. This method should be called before any
   * request to allocate/deallocate is made to the class.
   *
   * If the memory pool has already been created, PAL_INVALID is returned.
   * Likewise PAL_INVALID is returned if the pool size is set to ZERO.
   *
   * @param size number of memory blocks to allocated within the memory pool
   * @return platform error code
   */
  pal_error init(std::size_t size) noexcept {
    pal_error error = pal_require(free_ == nullptr && pool_ == nullptr &&
                                  tracker_ == nullptr);

    if (error != PAL_SUCCESS) {
      return PAL_INVALID;
    }

    if (size == 0) {
      return PAL_INVALID;
    }

    void *free_ptr = nullptr;
    void *pool_ptr = nullptr;
    void *tracker_ptr = nullptr;

    pal_error free_error = pal_mem_alloc(
        &free_ptr, sizeof(std::remove_pointer_t<decltype(free_)>) * size);
    pal_error pool_error = pal_mem_alloc(
        &pool_ptr, sizeof(std::remove_pointer_t<decltype(pool_)>) * size);
    pal_error tracker_error = pal_mem_alloc(
        &tracker_ptr, sizeof(std::remove_pointer_t<decltype(tracker_)>) * size);

    for (pal_error entry : {free_error, pool_error, tracker_error}) {
      if (entry != PAL_SUCCESS) {
        error = entry;
        break;
      }
    }

    if (error != PAL_SUCCESS) {
      pal_mem_free(&free_ptr);
      pal_mem_free(&pool_ptr);
      pal_mem_free(&tracker_ptr);

      return error;
    }

    size_ = size;
    available_ = size;

    free_ = static_cast<decltype(free_)>(free_ptr);
    pool_ = static_cast<decltype(pool_)>(pool_ptr);
    tracker_ = static_cast<decltype(tracker_)>(tracker_ptr);

    for (size_t i = 0; i < size_; ++i) {
      free_[i] = true;
      tracker_[i] = &pool_[i];
    }

    return PAL_SUCCESS;
  }

  /**
   * Destroys the memory pool, as in it releases the allocated resource back to
   * the platform.
   *
   * If the memory pool has not been allocated and the destroy method is called
   * an error code of PAL_SUCCESS is returned. If any memory block is
   * still being used, because it hasn't been deallocated, than the method will
   * return PAL_ERROR.
   *
   * @return platform error code
   */
  pal_error deinit() noexcept {
    if (free_ == nullptr || pool_ == nullptr || tracker_ == nullptr) {
      return PAL_SUCCESS;
    }

    if (available_ < size_) {
      return PAL_ERROR;
    }

    void *free_ptr = free_;
    void *pool_ptr = pool_;
    void *tracker_ptr = tracker_;

    pal_error free_error = pal_mem_free(&free_ptr);
    pal_error pool_error = pal_mem_free(&pool_ptr);
    pal_error tracker_error = pal_mem_free(&tracker_ptr);

    size_ = 0;
    available_ = 0;

    free_ = nullptr;
    pool_ = nullptr;
    tracker_ = nullptr;

    pal_error error = PAL_SUCCESS;

    for (pal_error entry : {free_error, pool_error, tracker_error}) {
      if (entry != PAL_SUCCESS) {
        error = entry;
        break;
      }
    }

    return error;
  }

  /**
   * Checks to make sure the class has been successfully initialized and capable
   * of responding to a request to allocate memory.
   *
   * @return true if the class is valid, otherwise false.
   */
  bool is_valid() const noexcept { return size_ > 0; }

  /**
   * Essentially it performs the same as alloc<Args...>(T **, Args &&... args),
   * except that the location where the memory block is saved into is a
   * unique_ptr.
   *
   * @tparam Args list of arguments types
   * @param unique_pointer pointer to the location to save the allocated data,
   * only in the case where PAL_SUCCESS is returned will the parameter be
   * updated.
   * @param args list of arguments
   *
   * @return platform error code, PAL_INVALID if the \p unique_pointer is
   * nullptr, PAL_OOM if there aren't any available memory locations.
   */
  template <typename... Args>
  pal_error alloc(UniquePointer *unique_pointer,
                  Args &&... args) noexcept(noexcept(T(args...))) {
    if (unique_pointer == nullptr) {
      return PAL_INVALID;
    }

    if (!is_valid()) {
      return PAL_INVALID;
    }

    T *memory_block;
    pal_error error = alloc(&memory_block, std::forward<Args>(args)...);

    if (error == PAL_SUCCESS) {
      *unique_pointer = UniquePointer(this, memory_block);
    }

    return error;
  }

  /**
   * Requests a memory block region. The arguments passed through here are
   * forwarded to the constructor of the class's data type.
   *
   * @tparam Args list of arguments types
   * @param memory_block pointer to the location to save the allocated data,
   * only in the case where PAL_SUCCESS is returned will the parameter be
   * updated.
   * @param args list of arguments
   *
   * @return platform error code, PAL_INVALID if the \p memory_block is nullptr,
   * PAL_OOM if there aren't any available memory locations.
   */
  template <typename... Args>
  pal_error alloc(T **memory_block,
                  Args &&... args) noexcept(noexcept(T(args...))) {
    if (memory_block == nullptr) {
      return PAL_INVALID;
    }

    if (!is_valid()) {
      return PAL_INVALID;
    }

    if (available_ == 0) {
      return PAL_OOM;
    }

    *memory_block = *tracker_;
    new (*memory_block) T(std::forward<Args>(args)...);

    ptrdiff_t memory_block_index = *memory_block - pool_;

    --available_;
    free_[memory_block_index] = false;
    ++tracker_;

    return PAL_SUCCESS;
  }

  /**
   * Releases the memory block region. Essentially this is the inverse of what
   * MemoryPool::alloc<T**, Args...>(Args &&... args) performs.
   *
   * Function will return PAL_INVALID should the memory block pointer be
   * incorrect in any way. Likewise if the memory block pointer has already been
   * previously deallocated.
   *
   * @param memory_block pointer to the memory block
   * @return platform error code
   */
  pal_error free(T *memory_block) noexcept(noexcept(std::declval<T>().~T())) {
    if (memory_block < pool_ || memory_block >= pool_ + size_ ||
        !detail::is_valid_memory_block_location(pool_, memory_block)) {
      return PAL_INVALID;
    }

    ptrdiff_t memory_block_index = memory_block - pool_;

    if (free_[memory_block_index]) {
      return PAL_INVALID;
    }

    memory_block->~T();

    ++available_;
    free_[memory_block_index] = true;
    --tracker_;

    *tracker_ = memory_block;

    return PAL_SUCCESS;
  }

 private:
  size_t size_;
  size_t available_;

  bool *free_;
  T *pool_;
  T **tracker_;
};

/**
 * Offers users the ability to create a memory pool.
 *
 * Unlike MemoryPool, this class is MOSTLY thread safe and provides the ability
 * for various objects to reference a common memory block within the memory
 * pool. The term MOSTLY was used as the only two methods that aren't thread
 * safe are the create and destroy methods within the class, all others will be.
 *
 * Performance wise, this class is O(1) for both allocation and deallocation.
 *
 * @note This memory pool's lifetime MUST always exceed those of its shared
 * memory pointer. If a memory pool is deconstructed while a reference pointer
 * exists for its memory pool, than the memory pool will be corrupted and as
 * such will crash in a developer build.
 *
 * @tparam T Data type stored in the memory pool
 */
template <typename T>
class SharedMemoryPool final {
 private:
  /**
   * Class represents the memory block within the memory pool. It will contain
   * the contents of the data type as well as some additional information, most
   * notably the data's reference pointer, which we call "use count".
   */
  class MemoryBlock final {
    friend class SharedMemoryPool;

   public:
    MemoryBlock(const MemoryBlock &) noexcept = delete;
    MemoryBlock(MemoryBlock &&) noexcept = delete;

    MemoryBlock &operator=(const MemoryBlock &) noexcept = delete;
    MemoryBlock &operator=(MemoryBlock &&) noexcept = delete;

    ~MemoryBlock() = default;

   private:
    /**
     * Constructor
     *
     * @tparam Args list of arguments types passed to the data type's
     * constructor
     * @param memory_pool pointer to the memory pool that manages the memory
     * block
     * @param args list of arguments pass to the data type's constructor
     */
    template <typename... Args>
    explicit MemoryBlock(
        SharedMemoryPool<T> *memory_pool,
        Args &&... args) noexcept(noexcept(T(std::forward<Args>(args)...)))
        : memory_pool_(memory_pool),
          use_count_(0),
          data_(std::forward<Args>(args)...) {}

    /**
     * @return pointer to the memory pool managing the memory block
     */
    SharedMemoryPool<T> *memory_pool() const noexcept { return memory_pool_; }

    /**
     * @return reference count for the memory block
     */
    size_t use_count() const noexcept { return use_count_; }

    /**
     * Increments the reference count by 1
     *
     * @return new reference count after the operation
     */
    size_t increment_use_count() noexcept { return ++use_count_; }

    /**
     * Decrement the reference count by 1
     *
     * @return new reference count after the operation
     */
    size_t decrement_use_count() noexcept { return --use_count_; }

    /**
     * @return reference to the underlying data held by the memory block
     */
    T &data() noexcept { return data_; }

   private:
    SharedMemoryPool<T> *memory_pool_;
    std::atomic<size_t> use_count_;
    T data_;
  };

 public:
  /**
   * This class acts much the same way that std::shared_ptr<T> does, in the
   * sense that it manages a resource that is accessible and referenced from
   * multiple places. The difference is that this class explicitly references
   * a memory block within a SharedMemoryPool.
   *
   * Class attempts to use much the same API that std::shared_ptr uses for
   * easy adoption by users.
   */
  class SharedPointer final {
    friend class SharedMemoryPool;

   public:
    /**
     * Constructor to a empty shared pointer.
     */
    SharedPointer() noexcept : SharedPointer(nullptr) {}

    SharedPointer(const SharedPointer &other) noexcept
        : memory_block_(other.memory_block_) {
      increment_use_count();
    }

    SharedPointer(SharedPointer &&other) noexcept
        : memory_block_(other.memory_block_) {
      other.memory_block_ = nullptr;
    }

    SharedPointer &operator=(const SharedPointer &other) noexcept {
      if (&other != this) {
        reset();
        memory_block_ = other.memory_block_;
        increment_use_count();
      }

      return *this;
    }

    SharedPointer &operator=(SharedPointer &&other) noexcept {
      if (&other != this) {
        reset();
        memory_block_ = other.memory_block_;
        other.memory_block_ = nullptr;
      }

      return *this;
    }

    ~SharedPointer() { reset(); }

    /**
     * Will release its hold on the underlying memory block. In doing so it will
     * decrement the reference counter.
     */
    void reset() noexcept {
      decrement_use_count();
      memory_block_ = nullptr;
    }

    /**
     * @return pointer to the underlying memory block's data
     */
    T *get() const noexcept {
      if (memory_block_ == nullptr) {
        return nullptr;
      }

      return &(memory_block_->data());
    }

    /**
     * @return reference to the underlying memory block's data
     */
    T &operator*() const noexcept { return *get(); }

    /**
     * @return pointer to the underlying memory block's data
     */
    T *operator->() const noexcept { return get(); }

    /**
     * @return the reference counter for the managed memory block
     */
    size_t use_count() const noexcept {
      if (memory_block_ == nullptr) {
        return 0;
      }

      return memory_block_->use_count();
    }

    /**
     * @return true if the class currently manages a memory block, otherwise
     * false
     */
    explicit operator bool() const noexcept { return get() != nullptr; }

   private:
    /**
     * Constructs a shared pointer that manages the specified memory block.
     *
     * @param memory_block memory block which will be managed
     */
    explicit SharedPointer(
        SharedMemoryPool<T>::MemoryBlock *memory_block) noexcept
        : memory_block_(memory_block) {
      increment_use_count();
    }

    /**
     * Increments the reference counter of the underlying memory block.
     */
    void increment_use_count() noexcept {
      if (memory_block_ == nullptr) {
        return;
      }

      memory_block_->increment_use_count();
    }

    /**
     * Decrements the reference counter of the underlying memory block.
     */
    void decrement_use_count() noexcept {
      if (memory_block_ == nullptr) {
        return;
      }

      if (memory_block_->decrement_use_count() == 0) {
        memory_block_->memory_pool()->free(memory_block_);
      }
    }

   private:
    SharedMemoryPool<T>::MemoryBlock *memory_block_;
  };

 public:
  SharedMemoryPool() noexcept
      : size_(0), available_(0), pool_(nullptr), tracker_(nullptr) {}

  SharedMemoryPool(const SharedMemoryPool<T> &) noexcept = delete;
  SharedMemoryPool(SharedMemoryPool<T> &&) noexcept = delete;

  SharedMemoryPool<T> &operator=(const SharedMemoryPool<T> &) noexcept = delete;
  SharedMemoryPool<T> &operator=(SharedMemoryPool<T> &&) noexcept = delete;

  /**
   * @see SharedMemoryPool
   */
  ~SharedMemoryPool() {
    assert(deinit() != PAL_ERROR &&
           "Shared memory pools is being destroyed while there are still "
           "pointers out there referencing this pools memory blocks");
  }

  /**
   * @return number of memory blocks allocated to the memory pool
   */
  size_t size() const noexcept {
    if (!is_valid()) {
      return 0;
    }
    return size_;
  }

  /**
   * @return number of memory blocks available in the memory pool
   */
  size_t available() const noexcept {
    if (!is_valid()) {
      return 0;
    }
    return available_;
  }

  /**
   * Creates the memory pool, as in it requests a memory block from the platform
   * for which it managed. This method should be called before any memory
   * blocks are allocated.
   *
   * If the memory pool has already been created, PAL_INVALID is returned.
   * Likewise PAL_INVALID is returned if the pool size is set to ZERO.
   *
   * @note This method IS NOT thread safe.
   *
   * @param size number of memory blocks to allocated within the memory pool
   * @return platform error code
   */
  pal_error init(std::size_t size) noexcept {
    pal_error error = pal_require(pool_ == nullptr && tracker_ == nullptr);

    if (error != PAL_SUCCESS) {
      return PAL_INVALID;
    }

    if (size == 0) {
      return PAL_INVALID;
    }

    void *pool_ptr = nullptr;
    void *tracker_ptr = nullptr;

    pal_error mutex_error = mutex_.init();
    pal_error condition_variable_error = condition_variable_.init();

    pal_error pool_error = pal_mem_alloc(
        &pool_ptr, sizeof(std::remove_pointer_t<decltype(pool_)>) * size);
    pal_error tracker_error = pal_mem_alloc(
        &tracker_ptr, sizeof(std::remove_pointer_t<decltype(tracker_)>) * size);

    for (pal_error entry :
         {mutex_error, condition_variable_error, pool_error, tracker_error}) {
      if (entry != PAL_SUCCESS) {
        error = entry;
        break;
      }
    }

    if (error != PAL_SUCCESS) {
      mutex_.deinit();
      condition_variable_.deinit();

      pal_mem_free(&pool_ptr);
      pal_mem_free(&tracker_ptr);

      return error;
    }

    size_ = size;
    available_ = size;

    pool_ = static_cast<decltype(pool_)>(pool_ptr);
    tracker_ = static_cast<decltype(tracker_)>(tracker_ptr);

    for (size_t i = 0; i < size; ++i) {
      tracker_[i] = &pool_[i];
    }

    return PAL_SUCCESS;
  }

  /**
   * Destroys the memory pool, as in it releases the allocated memory back to
   * the platform.
   *
   * f the memory pool has not been allocated and the destroy method is called
   * an error code of PAL_SUCCESS is returned. If any memory block is
   * still being used, because there is still a SharedPointer referencing an
   * memory block within this memory pool, than the method will return
   * PAL_ERROR.
   *
   * @note This method IS NOT thread safe.
   *
   * @return platform error code
   */
  pal_error deinit() noexcept {
    if (pool_ == nullptr || tracker_ == nullptr) {
      return PAL_SUCCESS;
    }

    if (available_ < size_) {
      return PAL_ERROR;
    }

    void *pool_ptr = pool_;
    void *tracker_ptr = tracker_;

    pal_error mutex_error = mutex_.deinit();
    pal_error condition_variable_error = condition_variable_.deinit();

    pal_error pool_error = pal_mem_free(&pool_ptr);
    pal_error tracker_error = pal_mem_free(&tracker_ptr);

    size_ = 0;
    available_ = 0;

    pool_ = nullptr;
    tracker_ = nullptr;

    pal_error error = PAL_SUCCESS;

    for (pal_error entry :
         {mutex_error, condition_variable_error, pool_error, tracker_error}) {
      if (entry != PAL_SUCCESS) {
        error = entry;
        break;
      }
    }

    return error;
  }

  /**
   * Checks to make sure the class has been successfully initialized and capable
   * of responding to a request to allocate memory.
   *
   * @return true if the class is valid, otherwise false.
   */
  bool is_valid() const noexcept { return size_ > 0; }

  /**
   * Requests a memory block region. The arguments passed through here are
   * forwarded to the constructor of the class's data type.
   *
   * This method will block on the user until a memory region is available
   * as such the return shared pointer will always have an associated memory
   * block that it references.
   *
   * @tparam Args list of arguments types
   * @param shared_pointer pointer to the shared pointer for which to save the
   * allocated pointer onto. only in the case where PAL_SUCCESS is returned
   * will the parameter be updated.
   * @param args list of arguments
   *
   * @return platform error code, PAL_INVALID if the \p shared_pointer is
   * nullptr.
   */
  template <typename... Args>
  pal_error alloc(SharedPointer *shared_pointer, Args &&... args) noexcept(
      noexcept(std::declval<SharedMemoryPool<T>>().lockless_alloc(
          shared_pointer, std::forward<Args>(args)...))) {
    if (shared_pointer == nullptr) {
      return PAL_INVALID;
    }

    if (!is_valid()) {
      return PAL_INVALID;
    }

    pal_error error;
    {
      pal::ScopedLock<pal::Mutex> lock(mutex_);
      condition_variable_.wait(lock, [this]() { return available_ > 0; });
      error = lockless_alloc(shared_pointer, std::forward<Args>(args)...);
    }
    condition_variable_.notify_all();

    return error;
  }

  /**
   * Requests a memory block region. The arguments passed through here are
   * forwarded to the constructor of the class's data type.
   *
   * @tparam Args list of arguments types
   * @param shared_pointer pointer to the shared pointer for which to save the
   * allocated pointer onto. if the result of the call is anything other then
   * PAL_SUCCESS, the shared pointer is reset.
   * @param args list of arguments
   *
   * @return platform error code. PAL_INVALID if the \p shared_pointer is
   * nullptr, PAL_OOM if there aren't any available memory locations.
   */
  template <typename... Args>
  pal_error try_alloc(SharedPointer *shared_pointer, Args &&... args) noexcept(
      noexcept(std::declval<SharedMemoryPool<T>>().lockless_alloc(
          shared_pointer, std::forward<Args>(args)...))) {
    if (shared_pointer == nullptr) {
      return PAL_INVALID;
    }

    if (!is_valid()) {
      return PAL_INVALID;
    }

    shared_pointer->reset();

    pal_error error;
    {
      pal::ScopedLock<pal::Mutex> lock(mutex_);
      error = lockless_alloc(shared_pointer, std::forward<Args>(args)...);
    }
    condition_variable_.notify_all();

    return error;
  }

  /**
   * Block the caller until all currently allocated resources have been released
   *
   * TODO(ESD-2235): This is being added to address a blocking issue to do with
   * complex swiftlets routing rules in replay mode. It should be removed once a
   * long term solution to the problem is found. See
   * https://snav.slack.com/archives/CHZ7V19RA/p1632754809114300
   */
  pal_error wait_until_completely_unused() noexcept {
    pal::ScopedLock<pal::Mutex> lock(mutex_);
    condition_variable_.wait(lock, [this]() { return available() == size(); });
    return PAL_SUCCESS;
  }

 private:
  /**
   * Requests a memory block region. Unlike its publicly accessible
   * counterparts, this method doesn't acquire the class's mutex.
   *
   * @tparam Args list of arguments types
   * @param shared_pointer pointer to the shared pointer for which to save the
   * allocated pointer onto. if the result of the call is anything other then
   * PAL_SUCCESS, the variable is untouched, otherwise the value is set.
   * Assumption is that the pointer is not nullptr.
   * @param args list of arguments
   *
   * @return platform error code. it returns PAL_OOM if there aren't any
   * possible memory locations available.
   */
  template <typename... Args>
  pal_error
  lockless_alloc(SharedPointer *shared_pointer, Args &&... args) noexcept(
      noexcept(MemoryBlock(nullptr, std::forward<Args>(args)...))) {
    if (available_ == 0) {
      return PAL_OOM;
    }

    MemoryBlock *memory_block = *tracker_;
    new (memory_block) MemoryBlock(this, std::forward<Args>(args)...);

    --available_;
    ++tracker_;

    *shared_pointer = SharedPointer(memory_block);

    return PAL_SUCCESS;
  }

  /**
   * Releases the memory block region. Essentially this is the inverse of what
   * SharedMemoryPool::alloc<SharedPointer*, Args...>(Args &&... args) and
   * SharedMemoryPool::try_alloc<SharedPointer*, Args...>(Args &&... args)
   * performs.
   *
   * @param memory_block pointer to the memory block to deallocate (aka release)
   * from the memory pool
   * @return platform error code
   */
  pal_error free(MemoryBlock *memory_block) noexcept(
      noexcept(std::declval<T>().~T())) {
    {
      pal::ScopedLock<pal::Mutex> lock(mutex_);

      if (memory_block < pool_ || memory_block >= pool_ + size_.load() ||
          !detail::is_valid_memory_block_location(pool_, memory_block)) {
        return PAL_INVALID;
      }

      memory_block->~MemoryBlock();

      ++available_;
      --tracker_;

      *tracker_ = memory_block;
    }
    condition_variable_.notify_all();
    return PAL_SUCCESS;
  }

 private:
  mutable pal::Mutex mutex_;
  mutable pal::ConditionVariable condition_variable_;
  std::atomic<size_t> size_;
  std::atomic<size_t> available_;
  MemoryBlock *pool_;
  MemoryBlock **tracker_;
};
}  // namespace pal

#endif  // LIBPAL_CPP_MEMORY_POOL_H
