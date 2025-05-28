#ifndef LIBPAL_CPP_MQ_H
#define LIBPAL_CPP_MQ_H

#include <chrono>
#include <utility>

#include <libpal++/chrono.h>
#include <libpal++/validation.h>
#include <libpal/ipc/mq.h>
#include <libpal/require.h>

namespace pal {
/**
 * Provides a message queue object
 *
 * This class wraps the message queue implementation from libpal. The API has
 * defined this interface as a thread safe message queue of a maximum size
 * defined at creation. Objects of type void* can be pushed or popped from any
 * thread. The internal implementation of the message queue resides within
 * libpal-impl.
 *
 * Users of this class should assume that a message pushed on to the queue is
 * owned by the queue until it is popped. The pusher loses ownership, and the
 * popper later gains it. If a attempt to push a message on to the queue fails
 * the caller retains ownership of the message
 *
 * An instance of this class must be explicitly initialized (#init) before being
 * used.
 */
class MQ final {
 public:
  MQ() noexcept : pal_handle_() {}

  MQ(const MQ &other) = delete;
  MQ(MQ &&other) noexcept : pal_handle_() {
    std::swap(pal_handle_, other.pal_handle_);
  }

  MQ &operator=(const MQ &other) = delete;
  MQ &operator=(MQ &&other) noexcept {
    std::swap(pal_handle_, other.pal_handle_);
    return *this;
  }

  ~MQ() { deinit(); }

  /**
   * Initialize message queue ready for use
   *
   * Allocate resources from libpal. Once this class has been inistantiated it
   * must be initialized by calling this function. Only after this function
   * returns PAL_SUCCESS will the message queue be available to use by calling
   * other member functions.
   *
   * @param max_len Maximum number of elements to be held in this queue
   * @return PAL error code
   */
  pal_error init(size_t max_len) noexcept;

  /**
   * Deinitialize message queue
   *
   * Release all PAL resources used by this instance. After this function
   * returns PAL_SUCCESS the message queue must not be used unless it is
   * reinitialized by a call to init().
   *
   * This function will be called automatically when the instance of MQ is
   * destroyed.
   *
   * Return PAL error code
   */
  pal_error deinit() noexcept;

  /**
   * Test whether this instance is valid, whether it is available for use
   *
   * A message queue is valid (ie, ready to be used) once it has been
   * initialized by a call to init(). When this function returns true it is safe
   * to use other functions in the class. If this function returns false this
   * instance must not be used.
   *
   * @return true if the instance is valid (ready to use), false otherwise
   */
  bool is_valid() const noexcept { return pal_handle_ != nullptr; }

  /**
   * Non blocking push
   *
   * Tries to push a message on to the queue. If the queue is already full when
   * this function is called it will not be able to push immediately. It will
   * return PAL_WOULD_BLOCK without taking any action
   *
   * @param msg Message to push on to the queue
   * @return platform error code
   */
  inline pal_error try_push(void *msg) {
    return push(msg, PAL_NONBLOCKING, std::chrono::seconds(0));
  }

  /**
   * Overload of MQ::push(void*, pal_blocking_mode, std::chrono::duration<Rep,
   * Period>) whereby it blocks indefinitely.
   *
   * @param msg Message to push on to the queue
   * @return platform error code
   */
  inline pal_error push(void *msg) {
    return push(msg, PAL_BLOCKING, std::chrono::microseconds(0));
  }

  /**
   * Overload of MQ::push(void *, pal_blocking_mode, std::chrono::duration<Rep,
   * Period>) whereby function blocks for an amount of time.
   *
   * Push a message on to the queue, blocking the caller up to a limit. If the
   * queue is already full when this function is called it will not be able to
   * immediately push another message. It will not return until either space
   * becomes available on the queue or the timeout expires. The second parameter
   * to this function specifies the amount of time this function will wait
   * before giving up and returning false. The timeout can be any value. The
   * special value of 0 (default) means block indefinitely.
   *
   * @warning This overload does not conform to established PAL conventions
   * regarding timeouts. Normally a timeout equal to 0 is taken to mean block
   * indefinitely. In this overload a timeout equal to 0 will invoke
   * non-blocking behaviour, it will behave identically to MQ::try_push(void *).
   * If you wish to use a timeout equal to 0 to cause indefinite blocking you
   * must call MQ::push(void *, pal_blocking_mode, std::chrono::duration<Rep,
   * Period>) and specify PAL_BLOCKING in the second parameter.
   *
   * @param msg Message to push on to the queue
   * @param timeout Amount of time to wait before returning without taking
   * action. See warning above.
   * @return platform error code
   */
  template <typename Rep, typename Period>
  pal_error push(void *msg, std::chrono::duration<Rep, Period> timeout) {
    if (timeout == std::chrono::microseconds::zero()) {
      return try_push(msg);
    }

    return push(msg, PAL_BLOCKING, timeout);
  }

  /**
   * Non blocking pop
   *
   * Tries to pop a message from the queue. If the queue is empty, that is to
   * say if the call is not able to complete immediately, this function will
   * return PAL_WOULD_BLOCK immediately without taking any action.
   *
   * @param msg On success will be updated to point to the popped message
   * @return platform error code
   */
  inline pal_error try_pop(void **msg) {
    return pop(msg, PAL_NONBLOCKING, std::chrono::microseconds(0));
  }

  /**
   * Overload function of MQ::pop(void**, pal_blocking_mode,
   * std::chrono::duration<Rep, Period>) whereby it blocks indefinitely.
   *
   * @param msg On success will be updated to point to the popped message
   * @return platform error code
   */
  inline pal_error pop(void **msg) {
    return pop(msg, PAL_BLOCKING, std::chrono::microseconds(0));
  }

  /**
   * Overload of MQ::pop(void **, pal_blocking_mode, std::chrono::duration<Rep,
   * Period>) whereby it blocks for an amount of time
   *
   * Pop a message from the queue, blocking the caller up to a limit. If there
   * is already a message waiting on the queue when this function is called it
   * will be popped immediately. If the queue is empty this function will not
   * return until either a message arrives and can be popped, or the timeout
   * expires. If the timeout expires no action will be taken, the message will
   * not have been pushed on to the queue. The timeout can be any value, the
   * special value of 0 (default) means block indefinitely
   *
   * @warning This overload does not conform to established PAL conventions
   * regarding timeouts. Normally a timeout equal to 0 is taken to mean block
   * indefinitely. In this overload a timeout equal to 0 will invoke
   * non-blocking behaviour, it will behave identically to MQ::try_pop(void **).
   * If you wish to use a timeout equal to 0 to cause indefinite blocking you
   * must call MQ::pop(void **, pal_blocking_mode, std::chrono::duration<Rep,
   * Period>) and specify PAL_BLOCKING in the second parameter.
   *
   * @param msg On success will be updated to point to the popped message
   * @param timeout Amount of time to wait before giving up and returning
   * PAL_TIMEOUT. See warning above.
   * @return platform error code
   */
  template <typename Rep, typename Period>
  pal_error pop(void **msg, std::chrono::duration<Rep, Period> timeout) {
    if (timeout == std::chrono::microseconds::zero()) {
      return try_pop(msg);
    }
    return pop(msg, PAL_BLOCKING, timeout);
  }

  /**
   * Push a message on to the queue.
   *
   * The \p mode and \p timeout parameters can be used to control blocking
   * behaviour.
   *
   * If \p mode is PAL_NONBLOCKING this function will return PAL_WOULD_BLOCK if
   * the queue is already full and the given message can not be pushed
   * immediately.
   *
   * If \p mode is PAL_BLOCKING this function will block up to \p timeout
   * waiting for space to become available on the queue. If the timeout period
   * expires before space becomes available PAL_TIMEOUT will be returned. The
   * special timeout value of 0 is taken to mean block indefinitely. A timeout
   * value of less than 0 will cause this function to return PAL_TIMEOUT
   * immediately without taking any action
   *
   * In both cases when this function returns PAL_SUCCESS the message has been
   * successfully pushed on to the queue
   *
   * @param msg Message to push
   * @param mode Blocking mode
   * @param timeout Timeout period, 0 for indefinite blocking. Ignored when \p
   * mode is PAL_NONBLOCKING.
   * @return platform error code
   */
  template <typename Rep, typename Period>
  pal_error push(void *msg, pal_blocking_mode mode,
                 std::chrono::duration<Rep, Period> timeout) {
    if (timeout < std::chrono::microseconds::zero()) {
      pal_error err = pal_require(
          is_valid() && pal_validate_blocking_mode(mode) && pal_has_impl_mq());
      if (err == PAL_SUCCESS) {
        err = PAL_TIMEOUT;
      }
      return err;
    }

    auto microseconds = pal::chrono::ceil<std::chrono::microseconds>(timeout);

    return ::pal_mq_push(pal_handle_, msg, mode,
                         static_cast<uint64_t>(microseconds.count()));
  }

  /**
   * Pop a message from queue.
   *
   * The \p mode and \p timeout parameters can be used to control blocking
   * behaviour.
   *
   * If \p mode is PAL_NONBLOCKING this function will return PAL_WOULD_BLOCK if
   * the queue is empty and a message can not be popped immediately.
   *
   * If \p mode is PAL_BLOCKING this function will block up to \p timeout
   * waiting for a message to become available on the queue. If the timeout
   * period expires before a message becomes available PAL_TIMEOUT will be
   * returned. The special timeout value of 0 is taken to mean block
   * indefinitely. A timeout value of less than 0 will cause this function to
   * return PAL_TIMEOUT immediately without taking any action
   *
   * In both cases when this function returns PAL_SUCCESS the \p msg will be
   * updated to point to the popped message.
   *
   * @param msg When PAL_SUCCESS is returned will be updated to point to the
   * popped message
   * @param mode Blocking mode
   * @param timeout Timeout period, 0 for indefinite blocking. Ignored when \p
   * mode is PAL_NONBLOCKING.
   * @return platform error code
   */
  template <typename Rep, typename Period>
  pal_error pop(void **msg, pal_blocking_mode mode,
                std::chrono::duration<Rep, Period> timeout) {
    if (timeout < std::chrono::microseconds::zero()) {
      pal_error err =
          pal_require(is_valid() && msg != nullptr &&
                      pal_validate_blocking_mode(mode) && pal_has_impl_mq());
      if (err == PAL_SUCCESS) {
        err = PAL_TIMEOUT;
      }
      return err;
    }

    auto microseconds = pal::chrono::ceil<std::chrono::microseconds>(timeout);

    return ::pal_mq_pop(pal_handle_, msg, mode,
                        static_cast<uint64_t>(microseconds.count()));
  }

 private:
  pal_mq_t pal_handle_;
};

}  // namespace pal

#endif  // LIBPAL_CPP_MQ_H
