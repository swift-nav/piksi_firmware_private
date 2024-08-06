/*
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBPAL_IMPL_IO_TCP_H
#define LIBPAL_IMPL_IO_TCP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <libpal/error.h>
#include <libpal/impl/blocking_mode.h>
#include <libpal/impl/io/network.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void *pal_tcp_t;

/**
 * Open a TCP client connection
 *
 * This function must establish a TCP connection to a remote server. Starling
 * will provide a host, port, and TCP/IP version to use. The \p host may be
 * expressed as either a FQDN or an IPV4/IPV6 address. If \p version is set to
 * PAL_IP_ANY the platform implementation is allowed to use whicheven TCP/IP
 * version it prefers. When set to PAL_IP_V4 or PAL_IP_V6 it must use the
 * requested version.
 *
 * If the connection was successfull the platform implementation must create a
 * TCP socket handle and write it to the value pointed at by \p sock. This
 * handle will be used in later calls to pal_tcp_read/pal_tcp_write, and finally
 * pal_tcp_close.
 *
 * This function may block for an amount of time up to the microsecond duration
 * given in \p timeout_us. If the timeout expires before a connection can be
 * established this function must return PAL_TIMEOUT without updating any other
 * parameters. The special value of 0 indicates indefinite blocking.
 *
 * Should any error occur during the connection attempt this function must
 * return an appropriate error code without updating any other parameters.
 *
 * @param sock On success must be set to TCP socket handle which can be used for
 * later read/write calls
 * @param host Host name/address
 * @param port Port number
 * @param version IP version
 * @param timeout_us Maximum duration to block
 * @return PAL error code
 */
typedef enum pal_error (*pal_tcp_client_open_t)(pal_tcp_t *sock,
                                                const char *host, uint16_t port,
                                                enum pal_ip_version version,
                                                uint64_t timeout_us);

/**
 * Open a TCP server port
 *
 * This function must bind to and listen on a TCP port for incoming client
 * connections. Starling will provide a TCP port number and IP version to use.
 * If \p version is set to PAL_IP_ANY the server must be capable of receiving
 * incoming client connections using either IPV4 or IPV6. If it is set to
 * PAL_IP_V4 or PAL_IP_V6 incoming connections must be limited to that version.
 *
 * On success this function must create an TCP socket handle and return it in
 * the \p sock parameter. This handle will be used for later called to
 * pal_tcp_accept and finally pal_tcp_close.
 *
 * @param sock On success must be set to TCP socket handle which can be used for
 * later pal_tcp_accept calls
 * @param port TCP port number
 * @param version IP version
 * @param backlog Maximum pending connections
 * @return PAL error code
 */
typedef enum pal_error (*pal_tcp_server_open_t)(pal_tcp_t *sock, uint16_t port,
                                                enum pal_ip_version version,
                                                size_t backlog);

/**
 * Accept an incoming client connection
 *
 * This function will be called for TCP server type socket handle to accept a
 * new incoming client connection. This function must create a new TCP socket
 * handle for the client and return it in the \p cli parameter. The client
 * handle must be valid for later calls to pal_tcp_read/pal_tcp_write and
 * finally pal_tcp_close.
 *
 * If a new client connection is available and returned from this function
 * information must be provided about the new connection in the \p host, \p
 * port, and \p ip_version parameters. \p host will be point to a buffer of at
 * least 46 bytes, this function must fill the buffer with the IPV4 or IPV6 IP
 * address of the new client formatted as a string with a NULL terminator. \p
 * port and \p ip_version must be set accordingly.
 *
 * The \p mode parameter controls the blocking behaviour of this function. If
 * set to PAL_NONBLOCKING this function must return PAL_WOULD_BLOCK without
 * taking any action if there are no pending client connections which can be
 * accepted immediately.
 *
 * When set to PAL_BLOCKING this function can block the caller until a new
 * client connection becomes available. It may block for up to the microsecond
 * duration given in the \p timeout_us parameter. If the timeout expires without
 * a new client connection becoming available this function must return
 * PAL_TIMEOUT without updating any other parameters.
 *
 * The special timeout value of 0 means to block indefinitely
 *
 * @param srv TCP server socket handle returned from pal_tcp_server_open
 * @param cli On success must be set to a new TCP socket handle representing the
 * client connection
 * @param host IP address of new client
 * @param port Port number of new client
 * @param ip_version IP version of new client connection
 * @param mode Blocking mode
 * @param timeout_us Maximum timeout period
 * @return PAL error code
 */
typedef enum pal_error (*pal_tcp_accept_t)(pal_tcp_t srv, pal_tcp_t *cli,
                                           char *host, uint16_t *port,
                                           enum pal_ip_version *version,
                                           enum pal_blocking_mode mode,
                                           uint64_t timeout_us);

/**
 * Set TCP keep alive parameters
 *
 * This function will be called for a TCP socket handles returned from either
 * pal_tcp_client_open or pal_tcp_accept to enable or disable TCP keep alive.
 *
 * When \p enabled is true the platform implementation must use the given values
 * for TCP keep alive. When set to false keep alive must be disabled for this
 * connection.
 *
 * @param sock TCP socket handle
 * @param enable indicates if TCP keep alive should be enabled for connection
 * @param idle_us time the connection needs to remain idle before TCP starts
 * sending keepalive probes (microseconds)
 * @param interval_us time between individual keepalive probes (microseconds)
 * @param retries maximum number of keepalive probes TCP should send before
 * dropping the connection
 * @return PAL error code
 */
typedef enum pal_error (*pal_tcp_keep_alive_t)(pal_tcp_t sock, bool enabled,
                                               uint64_t idle_us,
                                               uint64_t interval_us,
                                               uint16_t retries);

/**
 * Read from a TCP client socket
 *
 * This function will be called to read data from a previously opened TCP
 * socket. The handle passed in the \p io parameter will be that which was
 * returned from an earlier call to either #pal_tcp_client_open_t or
 * #pal_tcp_accept_t.
 *
 * This function must read up to \p n bytes from the TCP socket and write them
 * in to the buffer pointed at by \p buf. Should a call to this function succeed
 * in reading any amount of data at all the actual number of bytes read from the
 * device must be written in to the value pointed at by the \p nread parameter.
 *
 * It is acceptable for this function to not read any data at all. In this case
 * it may return PAL_SUCCESS without altering any other parameters providing
 * that the reason for not reading anything is not due to an error.
 *
 * \p mode controls the blocking behaviour of this function. If set to
 * PAL_NONBLOCKING a call to this function must return PAL_WOULD_BLOCK
 * immediately if it is unable to read any data from the TCP socket. If it is
 * able to read at least 1 byte it should do so, updating the \p nread parameter
 * as required.
 *
 * When \p mode is set to PAL_BLOCKING this function may block the caller for up
 * to the microsecond duration specified in the \p timeout_us parameter. If at
 * any point before the timeout expires some data becomes available on the TCP
 * socket this function must write it in to the \p buf, update \p nread, and
 * return PAL_SUCCESS. Should the timeout period expire without any data
 * becoming available this function must return PAL_TIMEOUT without altering any
 * other parameters.
 *
 * The special timeout value of 0 means block indefinitely.
 *
 * @param sock TCP socket handle
 * @param buf Buffer into which to write received data
 * @param n Maximum number of bytes to read from the TCP socket
 * @param nread On success must be updated with the actual number of bytes
 * written in the \p buf
 * @param mode Blocking mode
 * @param timeout_us Maximum blocking duration, only valid when \p mode is
 * PAL_BLOCKING
 * @return PAL error code
 */
typedef enum pal_error (*pal_tcp_read_t)(pal_tcp_t sock, uint8_t *buf, size_t n,
                                         size_t *nread,
                                         enum pal_blocking_mode mode,
                                         uint64_t timeout_us);

/**
 * Write to a TCP socket
 *
 * This function will be called to write data to a previously opened TCP socket.
 * The handle passed in the \p io parameter will be that which was returned from
 * an earlier call to either #pal_tcp_client_open_t or #pal_tcp_accept_t.
 *
 * This function must write up to \p n bytes to the TCP socket taken from the
 * buffer pointed at by \p buf. Should a call to this function succeed in
 * writing any amount of data at all the actual number of bytes written to the
 * device must be saved in to the value pointed at by the \p nwritten parameter.
 *
 * It is acceptable for this function to not write any data at all. In this case
 * it may return PAL_SUCCESS without altering any other parameters.
 *
 * \p mode controls the blocking behaviour of this function. If set to
 * PAL_NONBLOCKING a call to this function must return PAL_WOULD_BLOCK
 * immediately if it is unable to write any data to the TCP socket. If it is
 * able to write at least 1 byte it should do so, updating the \p nwritten
 * parameter as required.
 *
 * When \p mode is set to PAL_BLOCKING this function may block the caller for up
 * to the microsecond duration specified in the \p timeout_us parameter. If at
 * any point before the timeout expires some data can be written to the TCP
 * socket this function must do so, update \p nwritten, and return PAL_SUCCESS.
 * Should the timeout period expire without it being possible to write any data
 * this function must return PAL_TIMEOUT without altering any other parameters.
 *
 * The special timeout value of 0 means block indefinitely.
 *
 * @param sock TCP socket handle
 * @param buf Buffer from which to write data
 * @param n Maximum number of bytes to write to the TCP socket
 * @param nwritten On success must be updated with the actual number of bytes
 * written to the TCP socket
 * @param mode Blocking mode
 * @param timeout_us Maximum blocking duration, only valid when \p mode is
 * PAL_BLOCKING
 * @return PAL error code
 *
 */
typedef enum pal_error (*pal_tcp_write_t)(pal_tcp_t sock, const uint8_t *buf,
                                          size_t n, size_t *nwritten,
                                          enum pal_blocking_mode mode,
                                          uint64_t timeout_us);

/**
 * Close an open TCP socket
 *
 * When the user of libpal no longer requires an opened TCP socket it will call
 * this function. The platform implementation can finalise any pending write
 * operations, discard any data in read buffers (if any), free associated
 * resources, and release the handle.
 *
 * @param sock TCP socket handle
 * @return PAL error code
 */
typedef enum pal_error (*pal_tcp_close_t)(pal_tcp_t sock);

/**
 * Test a TCP socket for EOF condition
 *
 * An EOF condition is one where the TCP socket has become invalid and no
 * further read or write operations may be performed on it. Such a condition
 * might be a caused by the remote host terminating the connection, or any other
 * terminal condition for the underlying platform handle. When a TCP socket is
 * in the EOF state the only valid operation which can be performed on it is
 * close.
 *
 * This function must set the value pointer to by \p eof to true if the TCP
 * socket is in an EOF state.
 *
 * @param sock TCP socket handle
 * @param eof On success this function must indicate the EOF state of the device
 * @return PAL error code
 */
typedef enum pal_error (*pal_tcp_eof_t)(pal_tcp_t sock, bool *eof);

/**
 * PAL TCP IO implementation definition
 */
struct pal_impl_tcp {
  pal_tcp_client_open_t client_open;
  pal_tcp_server_open_t server_open;
  pal_tcp_accept_t accept;
  pal_tcp_keep_alive_t keep_alive;
  pal_tcp_read_t read;
  pal_tcp_write_t write;
  pal_tcp_close_t close;
  pal_tcp_eof_t eof;
};

/**
 * Install PAL TCP IO implementation in to API
 *
 * Call this function during pal_impl_init to register the implementation's
 * TCP IO module with the libpal API
 *
 * @param impl TCP IO implementation definition
 * @return PAL error code
 */
enum pal_error pal_set_impl_tcp(struct pal_impl_tcp *impl);

#ifdef __cplusplus
}
#endif

#endif
