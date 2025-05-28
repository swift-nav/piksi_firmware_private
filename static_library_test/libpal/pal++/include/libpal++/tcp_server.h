#ifndef LIBPAL_CPP_TCP_SERVER_H
#define LIBPAL_CPP_TCP_SERVER_H

#include <libpal++/chrono.h>
#include <libpal++/io_device.h>
#include <libpal++/network.h>
#include <libpal++/tcp_socket.h>
#include <libpal/io/tcp.h>

#include <chrono>
#include <utility>

namespace pal {
/**
 * Provides the ability to startup a TCP server which can accept and provide
 * TcpSocket connections.
 */
class TcpServer final {
 public:
  /**
   * Default maximum number of clients queued up to be accepted by server
   */
  static constexpr size_t kDefaultBacklog = 128;

 public:
  TcpServer() noexcept : pal_handle_() {}

  TcpServer(const TcpServer &other) = delete;
  TcpServer(TcpServer &&other) noexcept : pal_handle_() {
    std::swap(pal_handle_, other.pal_handle_);
  }

  TcpServer &operator=(const TcpServer &other) = delete;
  TcpServer &operator=(TcpServer &&other) noexcept {
    std::swap(pal_handle_, other.pal_handle_);
    return *this;
  }

  ~TcpServer() { close(); }

  /**
   * Starts up the server to start listening for incoming connections on the
   * specified port and potentially on a specified protocol version.
   *
   * @param port IP port that the server should be listening in on
   * @param version Specific IP version to listen to, by default it will
   * listen to all versions
   * @param backlog maximum number of clients in a queue pending connection
   *
   * @return platform error code
   */
  pal_error listen(uint16_t port, Ip::Version version = Ip::Version::ANY,
                   size_t backlog = kDefaultBacklog) noexcept;

  /**
   * Checks if the server is currently listening out for clients trying to
   * connect to server.
   *
   * @return true if server is currently running, otherwise return false
   */
  bool is_listening() noexcept;

  /**
   * Stops the server from listening.
   *
   * @return platform error code
   */
  pal_error close() noexcept;

  /**
   * Overload of TcpServer::accept(TcpSocket*, pal_blocking_mode,
   * std::chrono::duration<Rep, Period>) whereby function blocks infinitely.
   *
   * @param tcp_socket pointer to a tcp socket in which the method should
   * populate the server-client socket information. any previously existing
   * connection will be closed on a successful call to this method. this value
   * must not be set to nullptr.
   * @param endpoint On success will be filled out with details of the new
   * connection
   *
   * @return platform error code
   */
  pal_error accept(TcpSocket *tcp_socket, Ip::Endpoint *endpoint) noexcept {
    return accept_impl(tcp_socket, endpoint, PAL_BLOCKING, 0);
  }

  /**
   * Overload of TcpServer::accept(TcpSocket*, Ip::Endpoint*, pal_blocking_mode,
   * std::chrono::duration<Rep, Period>) whereby function blocks for a specified
   * period.
   *
   * @warning This overload does not conform to established PAL conventions
   * regarding timeouts. Normally a timeout equal to 0 is taken to mean block
   * indefinitely. In this overload a timeout equal to 0 will invoke
   * non-blocking behaviour, it will behave identically to
   * TcpServer::try_accept(TcpSocket *, Ip::Endpoint *). If you wish to use a
   * timeout equal to 0 to cause indefinite blocking you must call
   * TcpServer::accept(TcpSocket *, Ip::Endpoint *, pal_blocking_mode,
   * std::chrono::duration<Rep, Period>) and specify PAL_BLOCKING in the second
   * parameter
   *
   * @param tcp_socket pointer to a tcp socket in which the method should
   * populate the server-client socket information. any previously existing
   * connection will be closed on a successful call to this method. this value
   * must not be set to nullptr.
   * @param endpoint On success will be filled out with details of the new
   * connection
   * @param timeout Timeout period. See warning above
   * @return platform error code
   */
  template <typename Rep, typename Period>
  pal_error accept(TcpSocket *tcp_socket, Ip::Endpoint *endpoint,
                   std::chrono::duration<Rep, Period> timeout) noexcept {
    if (timeout == std::chrono::microseconds::zero()) {
      return try_accept(tcp_socket, endpoint);
    }
    return accept(tcp_socket, endpoint, PAL_BLOCKING, timeout);
  }

  /**
   * Overload of TcpServer::accept(TcpSocket*, Ip::Endpoint *,
   * pal_blocking_mode, std::chrono::duration<Rep, Period>) whereby function
   * does not block at all.
   *
   * @param tcp_socket pointer to a tcp socket in which the method should
   * populate the server-client socket information. any previously existing
   * connection will be closed on a successful call to this method. this value
   * must not be set to nullptr.
   * @param endpoint On success will be filled out with details of the new
   * connection
   * @return platform error code
   */
  pal_error try_accept(TcpSocket *tcp_socket, Ip::Endpoint *endpoint) noexcept {
    return accept(tcp_socket, endpoint, PAL_NONBLOCKING,
                  std::chrono::seconds(0));
  }

  /**
   * Accepts the request of any clients attempting to connect to server. The
   * server should be "listening" before calling this method.
   *
   * @param tcp_socket pointer to a tcp socket in which the method should
   * populate the server-client socket information. any previously existing
   * connection will be closed on a successful call to this method. this value
   * must not be set to nullptr.
   * @param endpoint On success will be filled out with details of the new
   * connection
   * @param Blocking mode
   * @param timeout maximum amount of time for which the function will block
   * for. a timeout of zero is interpreted as waiting infinitely.
   * @return platform error code
   */
  template <typename Rep, typename Period>
  pal_error accept(TcpSocket *tcp_socket, Ip::Endpoint *endpoint,
                   pal_blocking_mode mode,
                   std::chrono::duration<Rep, Period> timeout) noexcept {
    if (timeout < std::chrono::microseconds::zero()) {
      pal_error err = pal_require(
          is_listening() && tcp_socket != nullptr && endpoint != nullptr &&
          pal_validate_blocking_mode(mode) && !tcp_socket->is_open());
      if (err == PAL_SUCCESS) {
        err = PAL_TIMEOUT;
      }
      return err;
    }

    auto microseconds = pal::chrono::ceil<std::chrono::microseconds>(timeout);

    return accept_impl(tcp_socket, endpoint, mode,
                       static_cast<uint64_t>(microseconds.count()));
  }

 private:
  /**
   * Identical function to TcpServer::accept(TcpSocket *, Ip::Endpoint *,
   * std::chrono::duration<Rep, Period>) except that the parameter validations
   * for \p tcp_socket should be ignored and that the timeout is represented as
   * microseconds numeric value. A timeout of zero indicates indefinite blocking
   * of the function.
   */
  pal_error accept_impl(TcpSocket *tcp_socket, Ip::Endpoint *endpoint,
                        pal_blocking_mode mode, uint64_t timeout_us) noexcept;

 private:
  pal_tcp_t pal_handle_;
};
}  // namespace pal

#endif  // LIBPAL_CPP_TCP_SERVER_H
