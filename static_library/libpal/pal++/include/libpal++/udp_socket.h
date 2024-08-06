#ifndef LIBPAL_CPP_UDP_SOCKET_H
#define LIBPAL_CPP_UDP_SOCKET_H

#include <libpal++/io_device.h>
#include <libpal++/network.h>
#include <libpal/io/udp.h>

#include <utility>

namespace pal {
/**
 * Provides the IO interface for UDP sockets.
 */
class UdpSocket final : public IODevice {
 public:
  /**
   * Constructor
   *
   * UDP sockets can provide a buffering behaviour which makes them act more
   * like a byte stream than a packet stream. When the read buffer is enabled
   * the user can be sure that data will not be lost if they are not able to
   * guarantee reading a complete UDP packet in a single call. Should a call to
   * one of the "read" functions read less than the size of the received UDP
   * frame the rest will be buffered internally and returns on a subsequent call
   * to a read function.
   *
   * By default the read buffer is disabled, the user must as always make sure
   * to read the entire incoming UDP frame in a single call to a read function
   * otherwise data loss will occur.
   *
   * @param enable_read_buf Enable read buffer (default false)
   */
  explicit UdpSocket(bool enable_read_buf = false) noexcept
      : IODevice(),
        pal_handle_(),
        enable_read_buf_(enable_read_buf),
        read_buf_(),
        read_buf_count_(),
        read_buf_idx_(),
        read_buf_ep_() {}

  UdpSocket(const UdpSocket &other) = delete;
  UdpSocket(UdpSocket &&other) noexcept
      : IODevice(std::move(other)),
        pal_handle_(),
        enable_read_buf_(),
        read_buf_(),
        read_buf_count_(),
        read_buf_idx_(),
        read_buf_ep_() {
    std::swap(pal_handle_, other.pal_handle_);
    std::swap(enable_read_buf_, other.enable_read_buf_);
    std::swap(read_buf_, other.read_buf_);
    std::swap(read_buf_count_, other.read_buf_count_);
    std::swap(read_buf_idx_, other.read_buf_idx_);
    std::swap(read_buf_ep_, other.read_buf_ep_);
  }

  UdpSocket &operator=(const UdpSocket &other) = delete;
  UdpSocket &operator=(UdpSocket &&other) noexcept {
    IODevice::operator=(std::move(other));
    std::swap(pal_handle_, other.pal_handle_);
    std::swap(enable_read_buf_, other.enable_read_buf_);
    std::swap(read_buf_, other.read_buf_);
    std::swap(read_buf_count_, other.read_buf_count_);
    std::swap(read_buf_idx_, other.read_buf_idx_);
    std::swap(read_buf_ep_, other.read_buf_ep_);
    return *this;
  }

  ~UdpSocket() override;

  /**
   * Opens up a UDP socket.
   *
   * This function will open a UDP socket, allowing it to send datagrams out to
   * any number of different remote host via UdpSocket::send_to and able to
   * receive back datagrams via UpdSocket::receive_from, provided the server has
   * responded to the clients dynamically assigned port.
   *
   * @note following a successful call to this function, invoking
   * IODevice::read and IODevice::write will not work, users are restricted
   * to only being able to invoke UdpSocket::send_to and UpdSocket:receive_from
   * if one intends to receive/send datagrams.
   *
   * @param version IP version to use (Ip::Version::ANY is an invalid option)
   * @return platform error code
   */
  pal_error open(Ip::Version version = Ip::Version::V4) noexcept;

  /**
   * Creates a UDP socket and binds the socket to the local port.
   *
   * This function will create and initialize a UDP socket, allowing it to
   * send datagrams out to any number of different remote hosts via
   * UdpSocket::send_to and able to receive back datagrams via
   * UpdSocket::receive_from provided the remote host has sent the datagram to
   * the port specified by @p port.
   *
   * @note following a successful call to this function, IODevice::write will
   * not work, users are restricted to invoking UdpSocket::send_t to send out
   * datagrams.
   *
   * @param port local port number to bind socket onto
   * @param version IP version to use (Ip::Version::ANY is an invalid option)
   *
   * @return platform error code
   */
  pal_error open(uint16_t port, Ip::Version version = Ip::Version::V4) noexcept;

  /**
   * Establishes a UDP connection to a specified host.
   *
   * This is a "virtual" connection (since UDP is connection-less) which means
   * that any future calls to IODevice::write will know which host to send to.
   * Host will be able to reply back using the UDP socket's dynamically assigned
   * port, at which point user can call on IODevice::read to receive the
   * datagram.
   *
   * @note Users should be aware that with UDP sockets, content is sent/received
   * as datagrams, which means that there is a limit to the buffer size while
   * writing. While reading, if the user provides a buffer size smaller than the
   * datagram size that was sent, the remaining bytes will be discarded.
   *
   * @param hostname host identifier, it could be a DNS name or it could be an
   * IP address
   * @param port port number on the host
   * @param version IP version to use (Ip::Version::ANY is an invalid option)
   *
   * @return platform error code
   */
  pal_error open(const char *hostname, uint16_t port,
                 Ip::Version version = Ip::Version::V4) noexcept;

  pal_error close() noexcept override;
  bool eof() noexcept override;
  bool is_open() const noexcept override { return pal_handle_ != nullptr; }

  /**
   * Overload of UdpSocket::send_to(const uint8_t *, std::size_t, std::size_t *,
   * const Ip::Endpoint &, pal_blocking_mode, std::chrono::duration<Rep,
   * Period>) whereby function blocks indefinitely
   *
   * @param buffer buffer which holds the information to be sent out
   * @param max_count maximum number of bytes to send out from buffer
   * @param send_count actual amount of bytes sent out
   * @param endpoint details of who to send the datagram out to
   *
   * @return platform error code
   */
  pal_error send_to(const uint8_t *buffer, std::size_t max_count,
                    std::size_t *send_count,
                    const Ip::Endpoint &endpoint) noexcept {
    return send_to(buffer, max_count, send_count, endpoint, PAL_BLOCKING,
                   std::chrono::seconds(0));
  }

  /**
   * Overload of UdpSocket::receive_from(uint8_t *, std::size_t, std::size_t *,
   * Ip::Endpoint *, pal_blocking_mode, std::chrono::duration<Rep, Period>)
   * whereby function blocks indefinitely
   *
   * @param buffer buffer to record received content
   * @param max_count maximum number of bytes to receive from sender
   * @param receive_count actual amount of bytes received from sender
   * @param endpoint details of who sent the datagram, value can be nullptr
   *
   * @return platform error code
   */
  pal_error receive_from(uint8_t *buffer, std::size_t max_count,
                         std::size_t *receive_count,
                         Ip::Endpoint *endpoint) noexcept {
    return receive_from(buffer, max_count, receive_count, endpoint,
                        PAL_BLOCKING, std::chrono::seconds(0));
  }

  /**
   * Overload of UdpSocket::send_to(const uint8_t *, std::size_t, std::size_t *,
   * const Ip::Endpoint &, pal_blocking_mode, std::chrono::duration<Rep,
   * Period>) whereby function blocks for a specified period
   *
   * @warning This overload does not conform to established PAL conventions
   * regarding timeouts. Normally a timeout equal to 0 is taken to mean block
   * indefinitely. In this overload a timeout equal to 0 will invoke
   * non-blocking behaviour, it will behave identically to
   * UdpSocket::try_send_to(const uint8_t *, std::size_t, std::size_t *, const
   * Ip::Endpoint &). If you wish to use a timeout equal to 0 to cause
   * indefinite blocking you must call UdpSocket::send_to(const uint8_t *,
   * std::size_t, std::size_t *, const Ip::Endpoint &, pal_blocking_mode,
   * std::chrono::duration<Rep, Period>) and pass PAL_BLOCKING in the fifth
   * parameter.
   *
   * @param buffer buffer which holds the information to be sent out
   * @param max_count maximum number of bytes to send out from buffer
   * @param send_count actual amount of bytes sent out
   * @param endpoint details of who to send the datagram out to
   * @param timeout Timeout period. See warning above.
   * @return platform error code
   */
  template <typename Rep, typename Period>
  pal_error send_to(const uint8_t *buffer, std::size_t max_count,
                    std::size_t *send_count, const Ip::Endpoint &endpoint,
                    std::chrono::duration<Rep, Period> timeout) noexcept {
    if (timeout == std::chrono::microseconds::zero()) {
      return try_send_to(buffer, max_count, send_count, endpoint);
    }

    return send_to(buffer, max_count, send_count, endpoint, PAL_BLOCKING,
                   timeout);
  }

  /**
   * Overload of UdpSocket::receive_from(uint8_t *, std::size_t, std::size_t *,
   * Ip::Endpoint *, pal_blocking_mode, std::chrono::duration<Rep, Period>)
   * whereby function blocks for a specified period
   *
   * @warning This overload does not conform to established PAL conventions
   * regarding timeouts. Normally a timeout equal to 0 is taken to mean block
   * indefinitely. In this overload a timeout equal to 0 will invoke
   * non-blocking behaviour, it will behave identically to
   * UdpSocket::try_receive_from(const uint8_t *, std::size_t, std::size_t *,
   * Ip::Endpoint *). If you wish to use a timeout equal to 0 to cause
   * indefinite blocking you must call UdpSocket::receive_from(const uint8_t *,
   * std::size_t, std::size_t *, Ip::Endpoint *, pal_blocking_mode,
   * std::chrono::duration<Rep, Period>) and pass PAL_BLOCKING in the fifth
   * parameter.
   *
   * @param buffer buffer to record received content
   * @param max_count maximum number of bytes to receive from sender
   * @param receive_count actual amount of bytes received from sender
   * @param endpoint details of who sent the datagram, value can be nullptr
   * @param timeout Timeout period. See warning above.
   * @return platform error code
   */
  template <typename Rep, typename Period>
  pal_error receive_from(uint8_t *buffer, std::size_t max_count,
                         std::size_t *receive_count, Ip::Endpoint *endpoint,
                         std::chrono::duration<Rep, Period> timeout) noexcept {
    if (timeout == std::chrono::microseconds::zero()) {
      return try_receive_from(buffer, max_count, receive_count, endpoint);
    }

    return receive_from(buffer, max_count, receive_count, endpoint,
                        PAL_BLOCKING, timeout);
  }

  /**
   * Overload of UdpSocket::send_to(const uint8_t *, std::size_t, std::size_t *,
   * const Ip::Endpoint &, pal_blocking_mode, std::chrono::duration<Rep,
   * Period>) whereby function does not block at all
   *
   * @param buffer buffer which holds the information to be sent out
   * @param max_count maximum number of bytes to send out from buffer
   * @param send_count actual amount of bytes sent out
   * @param endpoint details of who to send the datagram out to
   * @return platform error code
   */
  pal_error try_send_to(const uint8_t *buffer, std::size_t max_count,
                        std::size_t *send_count,
                        const Ip::Endpoint &endpoint) noexcept {
    return send_to(buffer, max_count, send_count, endpoint, PAL_NONBLOCKING,
                   std::chrono::seconds(0));
  }

  /**
   * Overload of UdpSocket::receive_from(uint8_t *, std::size_t, std::size_t *,
   * Ip::Endpoint *, pal_blocking_mode, std::chrono::duration<Rep, Period>)
   * whereby function does not block at all
   *
   * @param buffer buffer to record received content
   * @param max_count maximum number of bytes to receive from sender
   * @param receive_count actual amount of bytes received from sender
   * @param endpoint details of who sent the datagram, value can be nullptr
   * @return platform error code
   */
  pal_error try_receive_from(uint8_t *buffer, std::size_t max_count,
                             std::size_t *receive_count,
                             Ip::Endpoint *endpoint) noexcept {
    return receive_from(buffer, max_count, receive_count, endpoint,
                        PAL_NONBLOCKING, std::chrono::seconds(0));
  }

  /**
   * Sends out a UDP datagram to the specified endpoint.
   *
   * @param buffer buffer which holds the information to be sent out
   * @param max_count maximum number of bytes to send out from buffer
   * @param send_count actual amount of bytes sent out
   * @param endpoint details of who to send the datagram out to
   * @param mode Blocking mode
   * @param timeout maximum time to wait for sending datagram
   *
   * @return platform error code
   */
  template <typename Rep, typename Period>
  pal_error send_to(const uint8_t *buffer, std::size_t max_count,
                    std::size_t *send_count, const Ip::Endpoint &endpoint,
                    pal_blocking_mode mode,
                    std::chrono::duration<Rep, Period> timeout) noexcept {
    if (timeout < std::chrono::microseconds::zero()) {
      detail::init_output_parameter(send_count, std::size_t(0));
      pal_error err =
          pal_require(is_open() && buffer != nullptr && send_count != nullptr &&
                      pal::Ip::valid(endpoint.version) &&
                      endpoint.version != pal::Ip::Version::ANY &&
                      pal_validate_blocking_mode(mode) && pal_has_impl_udp());
      if (err == PAL_SUCCESS) {
        err = PAL_TIMEOUT;
      }
      return err;
    }

    auto microseconds = pal::chrono::ceil<std::chrono::microseconds>(timeout);

    return ::pal_udp_send_to(pal_handle_, buffer, max_count, send_count,
                             endpoint.address, endpoint.port,
                             Ip::convert(endpoint.version), mode,
                             static_cast<uint64_t>(microseconds.count()));
  }

  /**
   * Receives UDP datagram address to client.
   *
   * @param buffer buffer to record received content
   * @param max_count maximum number of bytes to receive from sender
   * @param receive_count actual amount of bytes received from sender
   * @param endpoint details of who sent the datagram, value can be nullptr
   * @param Blocking mode
   * @param timeout maximum time to wait for receiving datagram
   *
   * @return platform error code
   */
  template <typename Rep, typename Period>
  pal_error receive_from(uint8_t *buffer, std::size_t max_count,
                         std::size_t *receive_count, Ip::Endpoint *endpoint,
                         pal_blocking_mode mode,
                         std::chrono::duration<Rep, Period> timeout) noexcept {
    detail::init_output_parameter(receive_count, std::size_t(0));
    pal_error err = pal_require(
        is_open() && buffer != nullptr && receive_count != nullptr &&
        endpoint != nullptr && pal_validate_blocking_mode(mode));

    if (err == PAL_SUCCESS) {
      if (timeout < std::chrono::microseconds::zero()) {
        err = PAL_TIMEOUT;
      }
    }

    if (err != PAL_SUCCESS) {
      return err;
    }

    auto microseconds = pal::chrono::ceil<std::chrono::microseconds>(timeout);

    return receive_from(buffer, max_count, receive_count, endpoint, mode,
                        static_cast<uint64_t>(microseconds.count()));
  }

 protected:
  pal_error read_impl(uint8_t *buffer, std::size_t max_count,
                      std::size_t *read_count, pal_blocking_mode mode,
                      uint64_t timeout_us) noexcept override;
  pal_error write_impl(const uint8_t *buffer, std::size_t max_count,
                       std::size_t *written_count, pal_blocking_mode mode,
                       uint64_t timeout_us) noexcept override;

 private:
  /**
   * Read data from socket or read buffer
   *
   * All public overloads of #receive_from should ultimately call this version
   * to perform the actual read.
   *
   * If the read buffer is enabled this function will handle reading from the
   * buffer and refilling the buffer when required. Errors will be passed back
   * as appropriate, remote endpoint information wil be provided on each call
   * and will match the data returned in \p buffer.
   *
   * If the read buffer is disabled this function will pass the call through to
   * the appropriate libpal function
   *
   * @param buffer buffer to record received content
   * @param max_count maximum number of bytes to receive from sender
   * @param receive_count actual amount of bytes received from sender
   * @param endpoint details of who sent the datagram, value can be nullptr
   * @param Blocking mode
   * @param timeout maximum time to wait for receiving datagram
   *
   * @return platform error code
   */
  pal_error receive_from(uint8_t *buffer, std::size_t max_count,
                         std::size_t *read_count, Ip::Endpoint *endpoint,
                         pal_blocking_mode mode, uint64_t timeout_us) noexcept;
  pal_error setup_read_buf() noexcept;

 private:
  pal_udp_t pal_handle_;
  bool enable_read_buf_;
  uint8_t *read_buf_;
  size_t read_buf_count_;
  size_t read_buf_idx_;
  Ip::Endpoint read_buf_ep_;
};
}  // namespace pal

#endif  // LIBPAL_CPP_UDP_SOCKET_H
