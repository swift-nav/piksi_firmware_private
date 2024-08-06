#ifndef LIBPAL_CPP_THREAD_H
#define LIBPAL_CPP_THREAD_H

#include <libpal++/tuple.h>
#include <libpal++/type_traits.h>
#include <libpal++/unique_pointer.h>
#include <libpal++/validation.h>
#include <libpal/require.h>
#include <libpal/thread/thread.h>

#include <cassert>
#include <chrono>
#include <tuple>
#include <type_traits>

namespace pal {
namespace detail {
class ThreadBase {
 protected:
  struct State {
    virtual ~State() = default;
    virtual void operator()() = 0;
  };

  template <typename Callable, typename... Args>
  class StateImpl final : public State {
   public:
    template <typename UCallable, typename... UArgs>
    explicit StateImpl(UCallable &&callable, UArgs &&... args)
        : State(),
          callable_(std::forward<UCallable>(callable)),
          args_(std::forward<UArgs>(args)...) {}

    ~StateImpl() override = default;

    void operator()() override {
      pal::apply(std::move(callable_), std::move(args_));
    }

   private:
    typename std::decay<Callable>::type callable_;
    std::tuple<typename std::decay<Args>::type...> args_;
  };

 public:
  ThreadBase() noexcept : pal_thread_() {}

  ThreadBase(const ThreadBase &other) = delete;
  ThreadBase(ThreadBase &&other) noexcept : pal_thread_() {
    std::swap(pal_thread_, other.pal_thread_);
  }

  ThreadBase &operator=(const ThreadBase &other) = delete;
  ThreadBase &operator=(ThreadBase &&other) noexcept {
    std::swap(pal_thread_, other.pal_thread_);
    return *this;
  }

  /**
   * Deconstructor
   *
   * PRECONDITION: ThreadBase::joinable must return false
   */
  ~ThreadBase() {
    if (joinable()) {
      assert(false && "Destroying thread class before joining");
    }
  };

  /**
   * @return true if a thread has been spawned and is awaiting to be joined,
   * false otherwise (maybe thread never got created or has already been joined)
   */
  bool joinable() const noexcept { return pal_thread_ != pal_thread_t(); }

  /**
   * Blocks the caller until the created thread is joined.
   *
   * PRECONDITIONS: ThreadBase::joinable() must return true
   *
   * @return platform error code
   */
  pal_error join() noexcept {
    pal_error ret = pal_require(joinable());
    if (ret != PAL_SUCCESS) {
      return ret;
    }

    ret = ::pal_thread_join(pal_thread_);
    if (ret == PAL_SUCCESS) {
      pal_thread_ = pal_thread_t();
    }

    return ret;
  }

  /**
   * Signals the thread to stop whatever blocking operation it currently is on
   * and or potentially any that will come across in the future and immediately
   * return with a platform error of #PAL_INTERRUPT. If the user calls this
   * function while while the thread is attending to a non blocking function,
   * the next blocking function will receive this signal and respond to as
   * mentioned previously. Subsequent blocking functions will not be interrupted
   * until a new signal is emitted via this function.
   *
   * It is worth mentioning that all blocking operations within the platform are
   * interruptable except for the following:
   *
   *   - pal::Mutex
   *   - pal::ConditionVariable
   *
   * The above synchronization classes cannot be interrupted, otherwise they
   * would fail to provide their respective purpose of guaranteeing correct
   * execution within critical regions.
   *
   * @return PAL error code
   */
  pal_error interrupt() noexcept { return ::pal_thread_interrupt(pal_thread_); }

 protected:
  pal_thread_t pal_thread_;
};
}  // namespace detail

/**
 * Provides the ability for users to create a thread, which works more or less
 * to how std::thread works, in that the class itself is movable. Unlike
 * std::thread, there is no way to create a running thread via the constructor
 * directly, one would need to invoke the DynamicThread::create function
 * directly.
 *
 * The class should only be used in an environment where dynamic memory
 * allocation is enabled. If that is not the case, please use the StaticThread
 * class.
 */
class DynamicThread final : public detail::ThreadBase {
 private:
  static void thread_entry(void *ctx) {
    pal::UniquePointer<State> state(static_cast<State *>(ctx));
    (*state)();
  }

 public:
  /**
   * Constructor
   *
   * NOTE: this will not create the thread, that will need to be done via the
   * DynamicThread::create method
   *
   * @param stack_size desired static size in units of bytes
   */
  explicit DynamicThread(
      size_t stack_size = PAL_THREAD_DEFAULT_STACKSIZE) noexcept
      : ThreadBase(), stack_size_(stack_size) {}

  DynamicThread(const DynamicThread &other) = delete;
  DynamicThread(DynamicThread &&other) noexcept = default;

  DynamicThread &operator=(const DynamicThread &other) = delete;
  DynamicThread &operator=(DynamicThread &&other) noexcept = default;

  /**
   * Invoke this method whenever you'd like to spawn off a thread.
   *
   * PRECONDITIONS: DynamicThread::joinable() must return false
   *
   * \code
   * int b = 20;
   * const int c = 30;
   * std::unique_ptr<int> d = std::make_unique<int>(40);
   *
   * DynamicThread thread;
   * thread.create_with_name("name", [](int a, int& b, const int&  c,
   * std::unique_ptr<int> d)
   * {
   *   // perform task
   * }, 10, std::ref(b), std::cref(c), std::move(d));
   * thread.join();
   * \endcode
   *
   * Notice that in order to pass a reference object, one would need to wrap the
   * argument in a std::reference_wrapper class. This is pretty much how
   * std::thread works as well.
   *
   * @tparam Callable the callable's type (ie. lambda, functor, function
   * pointer, pointer to member function, pointer to data member)
   * @tparam Args the callable's parameter types
   *
   * @param name Thread name
   * @param callable callable instance which will be invoked with the \p args as
   * arguments
   * @param args list of arguments that will be passed onto the callable
   *
   * @return platform error code
   */
  template <typename Callable, typename... Args>
  pal_error create_with_name(const char *name, Callable &&callable,
                             Args &&... args) {
    static_assert(
        pal::is_invocable<Callable, Args...>::value,
        "The Callable's type is unable to accept the specified Args...");
    static_assert(
        std::is_void<typename std::result_of_t<Callable(Args...)>>::value,
        "The return type of the Callable type must be void");

    pal_error ret = pal_require(!joinable());
    if (ret != PAL_SUCCESS) {
      return ret;
    }

    auto state = pal::make_unique<StateImpl<Callable, Args...>>(
        std::forward<Callable>(callable), std::forward<Args>(args)...);
    if (state.get() == nullptr) {
      return PAL_OOM;
    }

    ret = ::pal_thread_create(&pal_thread_, name, thread_entry, state.get(),
                              stack_size_);
    state.release();  // NOLINT: pointer ownership is transferred over to
                      // pal_thread_create

    return ret;
  }

  /**
   * Invoke this method whenever you'd like to spawn off a thread.
   *
   * PRECONDITIONS: DynamicThread::joinable() must return false
   *
   * \code
   * int b = 20;
   * const int c = 30;
   * std::unique_ptr<int> d = std::make_unique<int>(40);
   *
   * DynamicThread thread;
   * thread.create([](int a, int& b, const int&  c, std::unique_ptr<int> d)
   * {
   *   // perform task
   * }, 10, std::ref(b), std::cref(c), std::move(d));
   * thread.join();
   * \endcode
   *
   * Notice that in order to pass a reference object, one would need to wrap the
   * argument in a std::reference_wrapper class. This is pretty much how
   * std::thread works as well.
   *
   * @tparam Callable the callable's type (ie. lambda, functor, function
   * pointer, pointer to member function, pointer to data member)
   * @tparam Args the callable's parameter types
   *
   * @param callable callable instance which will be invoked with the \p args as
   * arguments
   * @param args list of arguments that will be passed onto the callable
   *
   * @return platform error code
   */
  template <typename Callable, typename... Args>
  pal_error create(Callable &&callable, Args &&... args) {
    return create_with_name(nullptr, std::forward<Callable>(callable),
                            std::forward<Args>(args)...);
  }

 private:
  size_t stack_size_;
};

/**
 * Much like the DynamicThread class, this thread provides the ability to create
 * threads, the major difference is that this class does not perform any dynamic
 * memory allocation. The content that is passed onto the thread is saved within
 * this class as member variables, which unfortunately restricts it from being
 * moved or copied.
 *
 * @tparam Callable the callable's type (ie. lambda, functor, function pointer,
 * pointer to member function, pointer to data member)
 * @tparam Args the callable's parameter types
 */
template <typename Callable, typename... Args>
class StaticThread final : public detail::ThreadBase {
// TODO(rodrigor): fix up pal:is_invocable (ticket: ESD-1856)
#if !defined(_LIBCPP_VERSION) || _LIBCPP_VERSION > 6000
  static_assert(
      pal::is_invocable<Callable, Args...>::value,
      "The Callable's type is unable to accept the specified Args...");
  static_assert(
      std::is_void<typename std::result_of_t<Callable(Args...)>>::value,
      "The return type of the Callable type must be void");
#endif

 private:
  struct ScopeGuard {
    explicit ScopeGuard(State &state_arg) noexcept : state(state_arg) {}
    ~ScopeGuard() { state.~State(); }
    State &state;
  };

  static void thread_entry(void *ctx) {
    ScopeGuard scope(*static_cast<State *>(ctx));
    scope.state();
  }

 public:
  /**
   * Constructor
   *
   * NOTE: this will not create the thread, that will need to be done via the
   * StaticThread::create method
   *
   * @param stack_size desired static size in units of bytes
   */
  explicit StaticThread(
      size_t stack_size = PAL_THREAD_DEFAULT_STACKSIZE) noexcept
      : ThreadBase(), stack_size_(stack_size), state_() {}

  StaticThread(const StaticThread &other) = delete;
  StaticThread(StaticThread &&other) noexcept = delete;

  StaticThread &operator=(const StaticThread &other) = delete;
  StaticThread &operator=(StaticThread &&other) noexcept = delete;

  /**
   * Invoke this method whenever you'd like to spawn off a thread.
   *
   * PRECONDITIONS: StaticThread::joinable() must return false
   *
   * \code
   * int b = 20;
   * const int c = 30;
   * std::unique_ptr<int> d = std::make_unique<int>(40);
   *
   * auto lambda = [](int a, int &b, const int &c, std::unique_ptr<int> d) {
   *
   * };
   *
   * StaticThread<decltype(lambda), int, std::reference_wrapper<int>,
   * std::reference_wrapper<const int>, std::unique_ptr<int>> thread;
   *
   * thread.create_with_name("name", std::move(lambda), 10, std::ref(b),
   * std::cref(c), std::move(d));
   *
   * thread.join();
   * \endcode
   *
   * Notice that in order to pass a reference, one would need to wrap the
   * argument in a std::reference::wrapper class using either std::ref or
   * std::cref helper functions within the create class but also specify the
   * std::reference_wrapper type whenever instantiating the thread.
   *
   * @tparam UCallable the callable's type as a universal reference
   * @tparam UArgs the callable's parameter types as a universal reference
   *
   * @param name Thread name
   * @param callable callable instance which will be invoked with the \p args as
   * arguments
   * @param args list of arguments that will be passed onto the callable
   *
   * @return platform error code
   */
  template <typename UCallable, typename... UArgs>
  pal_error create_with_name(const char *name, UCallable &&callable,
                             UArgs &&... args) {
    static_assert(
        pal::is_invocable<UCallable, UArgs...>::value,
        "The UCallable's type is unable to accept the specified UArgs...");
    static_assert(
        std::is_void<typename std::result_of_t<UCallable(UArgs...)>>::value,
        "The return type of the UCallable type must be void");
    static_assert(
        std::is_constructible<std::decay_t<Callable>,
                              std::decay_t<UCallable>>::value,
        "Callback type specified in the constructor is incompatible with that "
        "specified in the create function");

    pal_error ret = pal_require(!joinable());
    if (ret != PAL_SUCCESS) {
      return ret;
    }

    new (&state_)(StateImpl<Callable, Args...>)(
        std::forward<UCallable>(callable), std::forward<UArgs>(args)...);
    return pal_thread_create(&pal_thread_, name, thread_entry, &state_,
                             stack_size_);
  }

  /**
   * Invoke this method whenever you'd like to spawn off a thread.
   *
   * PRECONDITIONS: StaticThread::joinable() must return false
   *
   * \code
   * int b = 20;
   * const int c = 30;
   * std::unique_ptr<int> d = std::make_unique<int>(40);
   *
   * auto lambda = [](int a, int &b, const int &c, std::unique_ptr<int> d) {
   *
   * };
   *
   * StaticThread<decltype(lambda), int, std::reference_wrapper<int>,
   * std::reference_wrapper<const int>, std::unique_ptr<int>> thread;
   *
   * thread.create(std::move(lambda), 10, std::ref(b), std::cref(c),
   * std::move(d));
   *
   * thread.join();
   * \endcode
   *
   * Notice that in order to pass a reference, one would need to wrap the
   * argument in a std::reference::wrapper class using either std::ref or
   * std::cref helper functions within the create class but also specify the
   * std::reference_wrapper type whenever instantiating the thread.
   *
   * @tparam UCallable the callable's type as a universal reference
   * @tparam UArgs the callable's parameter types as a universal reference
   *
   * @param callable callable instance which will be invoked with the \p args as
   * arguments
   * @param args list of arguments that will be passed onto the callable
   *
   * @return platform error code
   */
  template <typename UCallable, typename... UArgs>
  pal_error create(UCallable &&callable, UArgs &&... args) {
    return create_with_name(nullptr, std::forward<UCallable>(callable),
                            std::forward<UArgs>(args)...);
  }

 private:
  size_t stack_size_;
  std::aligned_storage_t<sizeof(StateImpl<Callable, Args...>),
                         alignof(StateImpl<Callable, Args...>)>
      state_;
};
}  // namespace pal

#endif  // LIBPAL_CPP_THREAD_H
