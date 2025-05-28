#ifndef LIBPAL_CPP_IO_DEVICE_H
#define LIBPAL_CPP_IO_DEVICE_H

#include <libpal++/chrono.h>
#include <libpal++/validation.h>
#include <libpal/require.h>

#include <chrono>
#include <ios>

namespace pal {
/**
 * Base class for all IO related interfaces of a platform.
 */
class IODevice {
 public:
  virtual ~IODevice() = default;

  /**
   * Informs if a device is "opened". An "opened" device is one which has
   * everything setup such that a caller may proceed to reading/writing from/to
   * a device.
   *
   * NOTE: expectation is that all derived classes will have an "open" method
   * which will place the device in an "opened" state.
   *
   * @return true if the device is "open", otherwise false
   */
  virtual bool is_open() const noexcept = 0;

  /**
   * Requests that that the device be closed (ie. not "opened").
   *
   * @return platform error code
   */
  virtual pal_error close() noexcept = 0;

  /**
   * Overload of IODevice::read(uint8_t *, std::size_t, std::size_t*,
   * pal_blocking_mode, std::chrono::duration<Rep, Period>) whereby function
   * blocks infinitely.
   *
   * @param buffer buffer which will be used to store the read in information
   * @param max_count maximum number of bytes to be read
   * @param read_count amount of bytes read into buffer
   * @return platform error code
   */
  inline pal_error read(uint8_t *buffer, std::size_t max_count,
                        std::size_t *read_count) noexcept {
    return read(buffer, max_count, read_count, PAL_BLOCKING,
                std::chrono::seconds(0));
  }

  /**
   * Overload of IODevice::write(const uint8_t *buffer, std::size_t,
   * std::size_t*, pal_blocking_mode, std::chrono::duration<Rep, Period>)
   * whereby function blocks infinitely.
   *
   * @param buffer buffer which holds the information to be written
   * @param max_count maximum number of bytes to write
   * @param written_count amount of bytes written into buffer
   * @return platform error code
   */
  inline pal_error write(const uint8_t *buffer, std::size_t max_count,
                         std::size_t *written_count) noexcept {
    return write(buffer, max_count, written_count, PAL_BLOCKING,
                 std::chrono::seconds(0));
  }

  /**
   * Overload of IODevice::read(uint8_t *, std::size_t, std::size_t*,
   * pal_blocking_mode, std::chrono::duration<Rep, Period>) whereby function
   * blocks for a given timeout period.
   *
   * @warning This overload does not conform to established PAL conventions
   * regarding timeouts. Normally a timeout equal to 0 is taken to mean block
   * indefinitely. In this overload a timeout equal to 0 will invoke
   * non-blocking behaviour, it will behave identically to
   * IODevice::try_read(uint8_t *, std::size_t, std::size_t *). If you want to
   * use a timeout equal to 0 to cause indefinite blocking you must call
   * IODevice::read(uint8_t *, std::size_t, std::size_t *, pal_blocking_mode,
   * std::chrono::duration<Rep, Period>) and specify PAL_BLOCKING in the fifth
   * parameter.
   *
   * @param buffer buffer which will be used to store the read in information
   * @param max_count maximum number of bytes to be read
   * @param read_count amount of bytes read into buffer
   * @param timeout Timeout period. See warning above.
   * @return platform error code
   */
  template <typename Rep, typename Period>
  pal_error read(uint8_t *buffer, std::size_t max_count,
                 std::size_t *read_count,
                 std::chrono::duration<Rep, Period> timeout) noexcept {
    if (timeout == std::chrono::microseconds::zero()) {
      return try_read(buffer, max_count, read_count);
    }
    return read(buffer, max_count, read_count, PAL_BLOCKING, timeout);
  }

  /**
   * Overload of IODevice::write(const uint8_t *buffer, std::size_t,
   * std::size_t*, pal_blocking_mode, std::chrono::duration<Rep, Period>)
   * whereby function blocks for a given timeout period.
   *
   * @warning This overload does not conform to established PAL conventions
   * regarding timeouts. Normally a timeout equal to 0 is taken to mean block
   * indefinitely. In this overload a timeout equal to 0 will invoke
   * non-blocking behaviour, it will behave identically to
   * IODevice::try_write(uint8_t *, std::size_t, std::size_t *). If you want to
   * use a timeout equal to 0 to cause indefinite blocking you must call
   * IODevice::write(const uint8_t *, std::size_t, std::size_t *,
   * pal_blocking_mode, std::chrono::duration<Rep, Period>) and specify
   * PAL_BLOCKING in the fifth parameter.
   *
   * @param buffer buffer which holds the information to be written
   * @param max_count maximum number of bytes to write
   * @param written_count amount of bytes written into buffer
   * @param timeout Timeout period. See warning above
   * @return platform error code
   */
  template <typename Rep, typename Period>
  pal_error write(const uint8_t *buffer, std::size_t max_count,
                  std::size_t *written_count,
                  std::chrono::duration<Rep, Period> timeout) noexcept {
    if (timeout == std::chrono::microseconds::zero()) {
      return try_write(buffer, max_count, written_count);
    }
    return write(buffer, max_count, written_count, PAL_BLOCKING, timeout);
  }

  /**
   * Overload of IODevice::read(uint8_t *, std::size_t, std::size_t*,
   * pal_blocking_mode, std::chrono::duration<Rep, Period>) whereby function
   * does not block at all
   *
   * @param buffer buffer which will be used to store the read in information
   * @param max_count maximum number of bytes to be read
   * @param read_count amount of bytes read into buffer
   * @return platform error code
   */
  pal_error try_read(uint8_t *buffer, std::size_t max_count,
                     std::size_t *read_count) noexcept {
    return read(buffer, max_count, read_count, PAL_NONBLOCKING,
                std::chrono::seconds(0));
  }

  /**
   * Overload of IODevice::write(const uint8_t *buffer, std::size_t,
   * std::size_t*, pal_blocking_mode, std::chrono::duration<Rep, Period>)
   * whereby function does not block at all
   *
   * @param buffer buffer which holds the information to be written
   * @param max_count maximum number of bytes to write
   * @param written_count amount of bytes written into buffer
   * @return platform error code
   */
  pal_error try_write(const uint8_t *buffer, std::size_t max_count,
                      std::size_t *written_count) noexcept {
    return write(buffer, max_count, written_count, PAL_NONBLOCKING,
                 std::chrono::seconds(0));
  }

  /**
   * Attempts to read at most \p max_count bytes from the device and records it
   * down onto the \p buffer.
   *
   * If either the \p buffer and/or \p read_count is set to nullptr or the
   * IODevice::is_open() returns false then the method will respond with an
   * error of PAL_INVALID. Should the function return PAL_INVALID, the caller
   * should not expect that \p read_count to be correctly set.
   *
   * Callers can set the \p max_count to zero, at which point
   * the method will check and return the status of the device.
   *
   * A device may read less then the specified \p max_count amount for a
   * multitude of reasons, some are listed below:
   *
   *   1. device has reached EOF
   *   2. connection between client/server has been closed by peer
   *   3. blocked call has been explicitly interrupted by user
   *
   * Method will always report how much data it has been able to read, even if
   * an error has occurred during the event.
   *
   * Blocking behaviour can be controlled with the \p mode and \p timeout
   * parameters. In non blocking mode this function will return PAL_WOULD_BLOCK
   * if it is unable to read anything immediately. In blocking mode the call
   * will block up to the period given by \p timeout before returning
   * PAL_TIMEOUT. The special value of 0 as a timeout means block indefiniteiyl
   *
   * @param buffer buffer which will be used to store the read in information
   * @param max_count maximum number of bytes to be read
   * @param read_count amount of bytes read into buffer
   * @param mode Blocking mode
   * @param timeout maximum amount of time for which the function will block
   *
   * @return platform error code
   */
  template <typename Rep, typename Period>
  pal_error read(uint8_t *buffer, std::size_t max_count,
                 std::size_t *read_count, pal_blocking_mode mode,
                 std::chrono::duration<Rep, Period> timeout) noexcept {
    if (timeout < std::chrono::microseconds::zero()) {
      // This is a duplicate of the parameter checking/initialisation from
      // libpal required here because timeouts are always unsigned in libpal but
      // libpal++ has to cope with potential negative values
      detail::init_output_parameter(read_count, std::size_t(0));
      pal_error ret =
          pal_require(is_open() && buffer != nullptr && read_count != nullptr &&
                      pal_validate_blocking_mode(mode));
      if (ret == PAL_SUCCESS) {
        ret = PAL_TIMEOUT;
      }
      return ret;
    }

    auto microseconds = pal::chrono::ceil<std::chrono::microseconds>(timeout);

    return read_impl(buffer, max_count, read_count, mode,
                     static_cast<uint64_t>(microseconds.count()));
  }

  /**
   * Attempts to write at most \p max_count bytes from the \p buffer onto the
   * device.
   *
   * If either the \p buffer and/or \p written_count is set to nullptr or the
   * IODevice::is_open() returns false then the method will respond with an
   * error of PAL_INVALID.  Should the function return PAL_INVALID, the caller
   * should not expect that \p written_count to be correctly set.
   *
   * Callers can set the \p max_count to zero, at which point
   * the method will check and return the status of the device.
   *
   * A device may write less then the specified \p max_count amount for a
   * multitude of reasons, some are listed below:
   *
   *   1. device has no capacity to store more data
   *   2. connection between client/server has been closed by peer
   *   3. blocked call has been explicitly interrupted by user
   *
   * Method will always report how much data it has been able to write, even if
   * an error has occurred during the event.
   *
   * Blocking behaviour can be controlled with the \p mode and \p timeout
   * parameters. In non blocking mode this function will return PAL_WOULD_BLOCK
   * if it is unable to write anything immediately. In blocking mode the call
   * will block up to the period given by \p timeout before returning
   * PAL_TIMEOUT. The special value of 0 as a timeout means block indefinitely.
   *
   * @param buffer buffer which holds the information to be written
   * @param max_count maximum number of bytes to write
   * @param written_count amount of bytes written onto device
   * @param mode Blocking mode
   * @param timeout maximum amount of time for which the function will block
   *
   * @return platform error code
   */
  template <typename Rep, typename Period>
  pal_error write(const uint8_t *buffer, std::size_t max_count,
                  std::size_t *written_count, pal_blocking_mode mode,
                  std::chrono::duration<Rep, Period> timeout) noexcept {
    if (timeout < std::chrono::microseconds::zero()) {
      // This is a duplicate of the parameter checking/initialisation from
      // libpal required here because timeouts are always unsigned in libpal but
      // libpal++ has to cope with potential negative values
      detail::init_output_parameter(written_count, std::size_t(0));
      pal_error ret = pal_require(is_open() && buffer != nullptr &&
                                  written_count != nullptr &&
                                  pal_validate_blocking_mode(mode));
      if (ret == PAL_SUCCESS) {
        ret = PAL_TIMEOUT;
      }
      return ret;
    }

    auto microseconds = pal::chrono::ceil<std::chrono::microseconds>(timeout);

    return write_impl(buffer, max_count, written_count, mode,
                      static_cast<uint64_t>(microseconds.count()));
  }

  /**
   * Test whether this device is in EOF condition.
   *
   * EOF does not apply only to files, it is a generic condition that means the
   * device is no longer available for IO operations.
   *
   * @return true if device is in EOF state or if device is in an error state,
   * false otherwise.
   */
  virtual bool eof() noexcept = 0;

 protected:
  IODevice() noexcept = default;

  IODevice(const IODevice &) = delete;
  IODevice(IODevice &&) noexcept = default;

  IODevice &operator=(const IODevice &) = delete;
  IODevice &operator=(IODevice &&) noexcept = default;

  /**
   * Identical function to IODevice::read(uint8_t *, std::size_t, std::size_t*
   * std::chrono::duration<Rep, Period>) except that the
   * timeout is represented as microseconds numeric value. A timeout of zero
   * indicates indefinite blocking of the function.
   */
  virtual pal_error read_impl(uint8_t *buffer, std::size_t max_count,
                              std::size_t *read_count, pal_blocking_mode mode,
                              uint64_t timeout_us) noexcept = 0;

  /**
   * Identical function to IODevice::write(const uint8_t *, std::size_t,
   * st::size_t* std::chrono::duration<Rep, Period>) except that the timeout
   * is represented as microseconds numeric value. A timeout of zero indicates
   * indefinite blocking of the function.
   */
  virtual pal_error write_impl(const uint8_t *buffer, std::size_t max_count,
                               std::size_t *write_count, pal_blocking_mode mode,
                               uint64_t timeout_us) noexcept = 0;
};
}  // namespace pal

#endif  // LIBPAL_CPP_IO_DEVICE_H
