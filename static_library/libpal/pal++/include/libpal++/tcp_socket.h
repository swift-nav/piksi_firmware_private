#ifndef LIBPAL_CPP_TCP_SOCKET_H
#define LIBPAL_CPP_TCP_SOCKET_H

#include <libpal++/io_device.h>
#include <libpal++/network.h>
#include <libpal/io/tcp.h>

#include <utility>

namespace pal {
/**
 * Provides the IO interface for TCP sockets.
 */
class TcpSocket final : public IODevice {
  friend class TcpServer;

 public:
  explicit TcpSocket() noexcept : IODevice(), pal_handle_() {}

  TcpSocket(const TcpSocket &other) = delete;
  TcpSocket(TcpSocket &&other) noexcept
      : IODevice(std::move(other)), pal_handle_() {
    std::swap(pal_handle_, other.pal_handle_);
  }

  TcpSocket &operator=(const TcpSocket &other) = delete;
  TcpSocket &operator=(TcpSocket &&other) noexcept {
    IODevice::operator=(std::move(other));
    std::swap(pal_handle_, other.pal_handle_);
    return *this;
  }

  ~TcpSocket() override { close(); }

  /**
   * Establishes a tcp connection to a specified host.
   *
   * @param hostname host identifier, it could be a DNS name or it could be an
   * IP address
   * @param port port number on the host
   * @param version preferred IP version to use
   *
   * @return platform error code
   */
  pal_error open(const char *hostname, uint16_t port,
                 Ip::Version version = Ip::Version::ANY) noexcept {
    pal_error err = pal_require(!is_open() && pal::Ip::valid(version));
    if (err != PAL_SUCCESS) {
      return err;
    }
    return ::pal_tcp_client_open(&pal_handle_, hostname, port,
                                 Ip::convert(version), 0);
  }

  /**
   * Establishes a tcp connection to a specified host with a timeout value.
   *
   * @param hostname host identifier, it could be a DNS name or it could be an
   * IP address
   * @param port port number on the host
   * @param timeout timeout maximum amount of time for which the function will
   * block for
   * @param version preferred IP version to use
   *
   * @return platform error code
   */
  template <typename Rep, typename Period>
  pal_error open(const char *hostname, uint16_t port,
                 std::chrono::duration<Rep, Period> timeout,
                 Ip::Version version = Ip::Version::ANY) noexcept {
    pal_error ret = pal_require(!is_open() && pal::Ip::valid(version));
    if (ret != PAL_SUCCESS) {
      return ret;
    }

    if (timeout <= std::chrono::microseconds::zero()) {
      ret = pal_require(hostname != nullptr && port != 0 && pal_has_impl_tcp());
      if (ret == PAL_SUCCESS) {
        ret = PAL_TIMEOUT;
      }
      return ret;
    }

    auto microseconds = pal::chrono::ceil<std::chrono::microseconds>(timeout);

    return ::pal_tcp_client_open(&pal_handle_, hostname, port,
                                 Ip::convert(version),
                                 static_cast<uint64_t>(microseconds.count()));
  }

  pal_error close() noexcept override;
  bool eof() noexcept override;
  bool is_open() const noexcept override { return pal_handle_ != nullptr; }

  /**
   * Enables TCP keepalive on the device.
   *
   * @param idle time the connection needs to remain idle before TCP starts
   * sending keepalive probes
   * @param interval time between individual keepalive probes
   * @param retries maximum number of keepalive probes TCP should send before
   * dropping the connection
   *
   * @note none of the parameters should be zero.
   *
   * @return platform error code
   */
  template <typename TimeRep, typename TimePeriod, typename IntervalRep,
            typename IntervalPeriod>
  pal_error enable_keep_alive(
      std::chrono::duration<TimeRep, TimePeriod> idle,
      std::chrono::duration<IntervalRep, IntervalPeriod> interval,
      uint16_t retries) noexcept {
    pal_error ret = pal_require(idle > std::chrono::microseconds::zero() &&
                                interval > std::chrono::microseconds::zero());
    if (ret != PAL_SUCCESS) {
      return ret;
    }

    auto idle_us = static_cast<uint64_t>(
        pal::chrono::ceil<std::chrono::microseconds>(idle).count());
    auto interval_us = static_cast<uint64_t>(
        pal::chrono::ceil<std::chrono::microseconds>(interval).count());

    return ::pal_tcp_keep_alive(pal_handle_, true, idle_us, interval_us,
                                retries);
  }

  /**
   * Disables TCP keepalive on the device.
   *
   * @return platform error code
   */
  pal_error disable_keep_alive() noexcept;

 protected:
  pal_error read_impl(uint8_t *buffer, std::size_t max_count,
                      std::size_t *read_count, pal_blocking_mode mode,
                      uint64_t timeout_us) noexcept override;
  pal_error write_impl(const uint8_t *buffer, std::size_t max_count,
                       std::size_t *written_count, pal_blocking_mode mode,
                       uint64_t timeout_us) noexcept override;

 private:
  /**
   * Creates a tcp socket with an established platform io handler.
   *
   * @param pal_handle platform io handle
   */
  explicit TcpSocket(pal_tcp_t pal_handle) noexcept
      : IODevice(), pal_handle_(pal_handle) {}

  pal_tcp_t pal_handle_;
};
}  // namespace pal

#endif  // LIBPAL_CPP_TCP_SOCKET_H
