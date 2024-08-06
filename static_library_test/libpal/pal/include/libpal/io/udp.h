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

#ifndef LIBpal_udp_UDP_H
#define LIBpal_udp_UDP_H

#include <libpal/impl/io/udp.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Check global PAL UDP implementation
 *
 * @return true if global PAL UDP implementation has been set, otherwise false
 */
bool pal_has_impl_udp(void);

/**
 * Creates a UDP socket
 *
 * This function will create and initialize a UDP connection, allowing it to
 * only send datagrams out to potentially any number of different remote
 * servers. Once the connection is no longer required is must be passed to
 * #pal_udp_close which will tear down the connection.
 *
 * @note to send a datagram out to a host, one is restricted to using
 * #pal_udp_send_to over #pal_udp_write.
 *
 * @note due to UDP's inability to negotiate a connection with server and
 * platforms universally being unable to support dual stack sockets (ie support
 * ipv4 and ipv6 on a single socket), users will be required to be explicit
 * about their intended ip version (@p version).
 *
 * @param sock on success will contain the connection socket handle
 * @param version IP version (PAL_IP_ANY is an invalid option)
 *
 * @return PAL error code
 */
enum pal_error pal_udp_open(pal_udp_t *sock, enum pal_ip_version version);

/**
 * Creates a UDP IO and binds the IO to the local port.
 *
 * This function will create and initialize a UDP connection, allowing it to
 * send datagrams out to potentially a number of different remote servers while
 * being able to read packages coming into the specified port. Once the
 * connection is no longer required is must be passed to #pal_udp_close which
 * will tear down the connection.
 *
 * @note to send a datagram out to a host, one is restricted to using
 * #pal_udp_send_to over #pal_udp_write.  Likewise, to receive a datagram, one
 * is restricted to using #pal_udp_receive_from over #pal_udp_read.
 *
 * @note due to UDP's inability to negotiate a connection with server and
 * platforms universally being unable to support dual stack sockets (ie support
 * ipv4 and ipv6 on a single socket), users will be required to be explicit
 * about their intended ip version (@p version).
 *
 * @param sock on success will contain the connection socket handle
 * @param port local port to bind to
 * @param version IP version (PAL_IP_ANY is an invalid option)
 *
 * @return PAL error code
 */
enum pal_error pal_udp_bind_to(pal_udp_t *sock, uint16_t port,
                               enum pal_ip_version version);

/**
 * Creates a UDP IO and connects to a remote UDP socket
 *
 * This function will create and initialize a UDP connection with a remote
 * server. On successful return this function will provide a UDP socket handle
 * in the
 * @p sock parameter which can be used for future reading/writing via
 * #pal_udp_read and pal_io_write respectively. Once the connection is no longer
 * required is must be passed to #pal_udp_close which will tear down the
 * connection.
 *
 * @note due to UDP's inability to negotiate a connection with server and
 * platforms universally being unable to support dual stack sockets (ie support
 * ipv4 and ipv6 on a single socket), users will be required to be explicit
 * about their intended ip version (@p version).
 *
 * @param sock on success will contain the connection socket handle
 * @param host hostname or formatted IP address of remote server
 * @param port UDP port of remote server
 * @param version IP version (PAL_IP_ANY is an invalid option)
 *
 * @return PAL error code
 */
enum pal_error pal_udp_connect_to(pal_udp_t *sock, const char *host,
                                  uint16_t port, enum pal_ip_version version);

/**
 * Sends out a UDP datagram to the specified host.
 *
 * This function can operate in blocking or non blocking mode. In non blocking
 * mode this function shall return PAL_WOULD_BLOCK if it is not able to complete
 * immediately. In blocking mode this function shall block until either it is
 * able to send data to the socket, an error occurs on the socket, or the
 * timeout period as given in \p timeout_us expires. The timeout must be
 * specified in microseconds, the special value of 0 means block indefinitely.
 * Should the timeout expire this function will return PAL_TIMEOUT.
 *
 * @param sock UDP socket handle
 * @param buffer buffer which holds the information to be sent out
 * @param max_count maximum number of bytes to send out from buffer
 * @param send_count actual amount of bytes sent out
 * @param ip_address formatted IP address of remote server
 * @param port UDP port of remote server
 * @param version IP version (PAL_IP_ANY is an invalid option)
 * @param mode Blocking mode
 * @param timeout_us maximum time to wait for sending datagram (value of zero
 * indicates to wait indefinitely)
 *
 * @return PAL error code
 */
enum pal_error pal_udp_send_to(pal_udp_t sock, const uint8_t *buffer,
                               size_t max_count, size_t *send_count,
                               const char *ip_address, uint16_t port,
                               enum pal_ip_version version,
                               enum pal_blocking_mode mode,
                               uint64_t timeout_us);

/**
 * Receives UDP datagram address to client.
 *
 * This function can operate in blocking or non blocking mode. In non blocking
 * mode this function shall return PAL_WOULD_BLOCK if it is not able to complete
 * immediately. In blocking mode this function shall block until either it is
 * able to receive data from the socket, an error occurs on the socket, or the
 * timeout period as given in \p timeout_us expires. The timeout must be
 * specified in microseconds, the special value of 0 means block indefinitely.
 * Should the timeout expire this function will return PAL_TIMEOUT.
 *
 * @param sock UDP socket handle
 * @param buffer buffer to record received content
 * @param max_count maximum number of bytes to receive from sender
 * @param receive_count actual amount of bytes received from sender
 * @param ip_address buffer to save sender's ip address into (must be at least
 * 46 bytes long to support all possible addresses)
 * @param port UDP port of sender
 * @param version IP version (PAL_IP_ANY is an invalid option)
 * @param timeout_us maximum time to wait for receiving datagram (value of zero
 * indicates to wait indefinitely)
 *
 * @return PAL error code
 */
enum pal_error pal_udp_receive_from(pal_udp_t sock, uint8_t *buffer,
                                    size_t max_count, size_t *receive_count,
                                    char *ip_address, uint16_t *port,
                                    enum pal_ip_version *version,
                                    enum pal_blocking_mode mode,
                                    uint64_t timeout_us);

/**
 * Read data from a UDP socket
 *
 * This function will read up to \p n bytes from the specified UDP socket and
 * write them into the provided buffer.
 *
 * Reading can be done either as a blocking or non blocking operation as
 * controlled by the \p mode parameter. When operating in non blocking mode this
 * function shall return immediately with a code of #PAL_WOULD_BLOCK if it is
 * not able to immediately read any data from the UDP socket. In blocking
 * mode the call shall block until either data becomes available to read, an
 * error occurs on the UDP socket, or the timeout period given in \p timeout_us
 * expires. The timeout must be specified in microseconds, the special value of
 * 0 means block indefinitely. Should the timeout expire the function will
 * return PAL_TIMEOUT.
 *
 *
 * If the caller sets the \p n parameter as zero, than the function will return
 * the current error status of the UDP socket.
 *
 * The provided buffer must be at least \p n bytes in size to prevent overflows.
 * This function may read fewer than the requested bytes, this is not
 * necessarily an error. The number of bytes actually read will be saved in to
 * the \p nread memory address, as a such, the caller must always provide a
 * valid address otherwise a #PAL_INVALID error will be returned to the caller.
 * The \p nread will always be updated with the correct information, regardless
 * of the situation, if an error (other than #PAL_INVALID) has taken place and
 * no information was read, the memory address will be updated with the value of
 * zero.
 *
 * @param sock UDP socket handle
 * @param buf Buffer to read data in to
 * @param n Number of bytes to read
 * @param nread Number of bytes actually read
 * @param mode Blocking mode
 * @param timeout_us Timeout in microseconds
 * @return PAL error code
 */
enum pal_error pal_udp_read(pal_udp_t sock, uint8_t *buf, size_t n,
                            size_t *nread, enum pal_blocking_mode mode,
                            uint64_t timeout_us);

/**
 * Write data to a UDP socket
 *
 * This function will write up to \p n bytes from the specified buffer to the
 * UDP socket
 *
 * Writing can be done either as a blocking or non blocking operation as
 * controlled by the \p mode parameter. When operating in non blocking mode this
 * function shall return immediately with a code of #PAL_WOULD_BLOCK if it is
 * not able to immediately read any data from the UDP socket. In blocking
 * mode the call shall block until either some or all of the data has been
 * written, an error occurs on the UDP socket, or the timeout period given in \p
 * timeout_us expires. The timeout must be specified in microseconds, the
 * special value of 0 means block indefinitely. Should the timeout expire the
 * function will return PAL_TIMEOUT.
 *
 * If the caller sets the \p n parameter as zero, then the function will return
 * the current error status of the UDP socket.
 *
 * The provided buffer \p buf must be at least \p n bytes in size. This function
 * may write fewer than the requested bytes, this is not necessarily an error.
 * The number of bytes actually written will be saved in to the \p nwritten
 * memory address, as a such, the caller must always provide a valid address
 * otherwise a #PAL_INVALID error will be returned to the caller. The \p
 * nwritten will always be updated with the correct information, regardless of
 * the situation, if an error (other than #PAL_INVALID) has taken place and no
 * information was written, the memory address will be updated with the value of
 * zero.
 *
 * @param sock UDP socket handle
 * @param buf Buffer from which to write data
 * @param n Number of bytes to write
 * @param nwritten Number of bytes actually written
 * @param mode Blocking mode
 * @param timeout_us Timeout in microseconds
 * @return PAL error code
 */
enum pal_error pal_udp_write(pal_udp_t sock, const uint8_t *buf, size_t n,
                             size_t *nwritten, enum pal_blocking_mode mode,
                             uint64_t timeout_us);

/**
 * Close an open UDP socket
 *
 * Calling this function will make the UDP socket unavailable for more read or
 * writes.
 * After this function returns success the handle will be set to NULL must not
 * be reused.
 *
 * @param sock Pointer to an open UDP socket handle
 * @return PAL error code
 */
enum pal_error pal_udp_close(pal_udp_t *sock);

/**
 * Test whether a UDP socket is in EOF condition
 *
 * The term EOF means that this UDP socket is unavailable for further IO
 * activity but has not yet been closed. The only valid operation on a UDP
 * socket in this state is to call #pal_udp_close to clean up any resources.
 *
 * @param sock UDP socket handle
 * @param eof On success will be set true if the UDP socket is in EOF state,
 * false otherwise
 * @return PAL error code
 */
enum pal_error pal_udp_eof(pal_udp_t sock, bool *eof);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // LIBpal_udp_UDP_H
