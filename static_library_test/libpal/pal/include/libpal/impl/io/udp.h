/**
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

#ifndef LIBPAL_IMPL_IO_UDP_H
#define LIBPAL_IMPL_IO_UDP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <libpal/error.h>
#include <libpal/impl/blocking_mode.h>
#include <libpal/impl/io/network.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void *pal_udp_t;

/**
 * Create a UDP socket
 *
 * This function must create and initialize a UDP socket which can be used to
 * send datagrams to remote systems. If successful this function must create a
 * new UDP socket handle and set it to the value pointed at by the \p sock
 * parameter. This handle will be used for later calls to pal_udp_send_to.
 *
 * @param sock On success must be updated to a new UDP socket handle
 * @param version IP version
 * @return PAL error code
 */
typedef enum pal_error (*pal_udp_open_t)(pal_udp_t *sock,
                                         enum pal_ip_version version);

/**
 * Create and bind a UDP socket
 *
 * This function must create and initialize a UDP socket bound to the given port
 * able to send and receive datagrams to/from remote systems. If successful this
 * function must create a new UDP socket handle and set it to the value pointed
 * at by the \p sock parameter. This handle will be used for later calls to
 * pal_udp_send_to, pal_udp_receive_from, and pal_udp_read.
 *
 * @param sock On success must be updated to a new UDP socket handle
 * @param port IP port number
 * @param version IP version
 * @return PAL error code
 */
typedef enum pal_error (*pal_udp_bind_to_t)(pal_udp_t *sock, uint16_t port,
                                            enum pal_ip_version version);

/**
 * Create a UDP socket and connect to a remote UDP socket
 *
 * This function must create and initialize a UDP socket bound to a remote
 * server. If successful this function must create a new UDP socket handle and
 * set it to the value pointed at by the \p sock parameter.
 *
 * A UDP socket opened by this function will only ever communicate with a single
 * remote system. The handle provided by this function will be passed back to
 * pal_udp_read/pal_udp_write.
 *
 * @param sock On success must be updated to a new UDP socket handle
 * @param port IP port number
 * @param version IP version
 * @return PAL error code
 */
typedef enum pal_error (*pal_udp_connect_to_t)(pal_udp_t *sock,
                                               const char *host, uint16_t port,
                                               enum pal_ip_version version);

/**
 * Send a UDP datagram to the specified host
 *
 * This function must send data from the buffer pointed at by \p buffer as a UDP
 * datagram to the given host. The size of the buffer is given in the \p
 * max_count parameter.
 *
 * The destination system is specified in the \p ip_address, \p port, and \p
 * version parameters. The address may be expressed as either a FQDN or an
 * IPV4/IPV6 address.
 *
 * If a call to this function succeeds in sending at least 1 byte to the
 * destination system this function must update the \p send_count parameter with
 * the number of bytes actually sent.
 *
 * The \p mode parameter contols the blocking behaviour of this function. If set
 * to PAL_NONBLOCKING this function must either immediately send some data and
 * return PAL_SUCCESS (updating \p send_count as appropriate), or return
 * PAL_WOULD_BLOCK without sending any data and without updating any parameters.
 *
 * If \p mode is set to PAL_BLOCKING this function may block the caller for up
 * to the microsecond duration given in the \p timeout_us parameter. If the
 * timeout period expires before this function is able to send any data it must
 * return PAL_TIMEOUT without updating any other parameters.
 *
 * The special timeout value of 0 means block indefinitely.
 *
 * @param sock UDP socket handle
 * @param buffer Buffer from which to send data
 * @param max_count Maximum number of bytes to send
 * @param send_count On success must be updated to the number of bytes actually
 * sent
 * @param host Destination socket IP address
 * @param port Destination socket port number
 * @param version IP version
 * @param mode Blocking mode
 * @param timeout_us Maximum blocking duration
 * @return PAL error code
 */
typedef enum pal_error (*pal_udp_send_to_t)(
    pal_udp_t sock, const uint8_t *buffer, size_t max_count, size_t *send_count,
    const char *host, uint16_t port, enum pal_ip_version version,
    enum pal_blocking_mode mode, uint64_t timeout_us);

/**
 * Receive a UDP datagram from a remote system
 *
 * This function must receive data sent in to a UDP socket from a remote system.
 * When data is receive this function must fill the output parameters with the
 * details of the sending system.
 *
 * This function must receive data from a remote system and write it in to the
 * buffer pointed at by the \p buffer parameter. Up to \p max_count bytes may be
 * read. It is acceptable to read fewer than the requested number of bytes. When
 * one or more bytes has been received and written in to the buffer this
 * function must return PAL_SUCCESS and update \p receive_count with the number
 * of bytes received.
 *
 * When 1 or more bytes has been receive this function must update the \p
 * ip_address, \p port, and \p version parameters with the details of the socket
 * which sent the received data. The \p ip_address parameter points to a buffer
 * which is at least 46 bytes long. This function must write in to this buffer
 * the sender's address formatted at an IPV4/IPV6 address as appropriate.
 *
 * The \p mode parameter controls the blocking behaviour of this function. If
 * set to PAL_NONBLOCKING this function must either imemdiately receive some
 * data and return PAL_SUCCESS (updating \p receive_count as appropriate), or
 * return PAL_WOULD_BLOCK without updating any other parameters.
 *
 * If \p mode is set to PAL_BLOCKING this function may block the caller for up
 * to the microsecond duration given in the \p timeout_us parameter. If the
 * timeout period expires before this function is able to send any data it must
 * return PAL_TIMEOUT without updating any other parameters.
 *
 * The special timeout value of 0 means block indefinitely.
 *
 * @param sock UDP socket handle
 * @param buffer Buffer into which to receive data
 * @param max_count Maximum number of bytes to receive and store in \p buffer
 * @param receive_count On success must be updated with the number of bytes
 * actually received
 * @param ip_address On success must be updated with the IP address of the
 * sending system
 * @param port On success must be updated with the port number of the sending
 * system
 * @param version On success must be update with the IP version of the sending
 * system
 * @param mode Blocking mode
 * @param timeout_us Maximum blocking duration
 * @return PAL error code
 */
typedef enum pal_error (*pal_udp_receive_from_t)(
    pal_udp_t sock, uint8_t *buffer, size_t max_count, size_t *receive_count,
    char *ip_address, uint16_t *port, enum pal_ip_version *version,
    enum pal_blocking_mode mode, uint64_t timeout_us);

/**
 * Read from a UDP socket
 *
 * This function will be called to read data from a previously opened UDP
 * socket.
 *
 * This function must read up to \p n bytes from the UDP socket and write them
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
 * immediately if it is unable to read any data from the UDP socket. If it is
 * able to read at least 1 byte it should do so, updating the \p nread parameter
 * as required.
 *
 * When \p mode is set to PAL_BLOCKING this function may block the caller for up
 * to the microsecond duration specified in the \p timeout_us parameter. If at
 * any point before the timeout expires some data becomes available on the UDP
 * socket this function must write it in to the \p buf, update \p nread, and
 * return PAL_SUCCESS. Should the timeout period expire without any data
 * becoming available this function must return PAL_TIMEOUT without altering any
 * other parameters.
 *
 * The special timeout value of 0 means block indefinitely.
 *
 * @param sock UDP socket handle
 * @param buf Buffer into which to write received data
 * @param n Maximum number of bytes to read from the UDP socket
 * @param nread On success must be updated with the actual number of bytes
 * written in the \p buf
 * @param mode Blocking mode
 * @param timeout_us Maximum blocking duration, only valid when \p mode is
 * PAL_BLOCKING
 * @return PAL error code
 */
typedef enum pal_error (*pal_udp_read_t)(pal_udp_t sock, uint8_t *buf, size_t n,
                                         size_t *nread,
                                         enum pal_blocking_mode mode,
                                         uint64_t timeout_us);

/**
 * Write to a UDP socket
 *
 * This function will be called to write data to a previously opened UDP socket.
 *
 * This function must write up to \p n bytes to the UDP socket taken from the
 * buffer pointed at by \p buf. Should a call to this function succeed in
 * writing any amount of data at all the actual number of bytes written to the
 * device must be saved in to the value pointed at by the \p nwritten parameter.
 *
 * It is acceptable for this function to not write any data at all. In this case
 * it may return PAL_SUCCESS without altering any other parameters.
 *
 * \p mode controls the blocking behaviour of this function. If set to
 * PAL_NONBLOCKING a call to this function must return PAL_WOULD_BLOCK
 * immediately if it is unable to write any data to the UDP socket. If it is
 * able to write at least 1 byte it should do so, updating the \p nwritten
 * parameter as required.
 *
 * When \p mode is set to PAL_BLOCKING this function may block the caller for up
 * to the microsecond duration specified in the \p timeout_us parameter. If at
 * any point before the timeout expires some data can be written to the UDP
 * socket this function must do so, update \p nwritten, and return PAL_SUCCESS.
 * Should the timeout period expire without it being possible to write any data
 * this function must return PAL_TIMEOUT without altering any other parameters.
 *
 * The special timeout value of 0 means block indefinitely.
 *
 * @param sock UDP socket handle
 * @param buf Buffer from which to write data
 * @param n Maximum number of bytes to write to the UDP socket
 * @param nwritten On success must be updated with the actual number of bytes
 * written to the UDP socket
 * @param mode Blocking mode
 * @param timeout_us Maximum blocking duration, only valid when \p mode is
 * PAL_BLOCKING
 * @return PAL error code
 */
typedef enum pal_error (*pal_udp_write_t)(pal_udp_t sock, const uint8_t *buf,
                                          size_t n, size_t *nwritten,
                                          enum pal_blocking_mode mode,
                                          uint64_t timeout_us);

/**
 * Close an open UDP socket
 *
 * When the user of libpal no longer requires an opened UDP socket it will call
 * this function. The platform implementation can finalise any pending write
 * operations, discard any data in read buffers (if any), free associated
 * resources, and release the handle.
 *
 * @param sock UDP socket handle
 * @return PAL error code
 */
typedef enum pal_error (*pal_udp_close_t)(pal_udp_t sock);

/**
 * Test a UDP Socket for EOF condition
 *
 * An EOF condition is one where the UDP Socket has become invalid and no
 * further read or write operations may be performed on it. Such a condition
 * might be cause by any terminal condition for the underlying platform handle.
 * When a UDP socket is in the EOF state the only valid operation which can be
 * performed on it is close.
 *
 * This function must set the value pointer to by \p eof to true if the UDP
 * socket is in an EOF state.
 *
 * @param sock UDP socket handle
 * @param eof On success this function must indicate the EOF state of the device
 * @return PAL error code
 */
typedef enum pal_error (*pal_udp_eof_t)(pal_udp_t sock, bool *eof);

/**
 * PAL UDP IO implementation definition
 */
struct pal_impl_udp {
  pal_udp_open_t open;
  pal_udp_bind_to_t bind_to;
  pal_udp_connect_to_t connect_to;
  pal_udp_send_to_t send_to;
  pal_udp_receive_from_t receive_from;
  pal_udp_read_t read;
  pal_udp_write_t write;
  pal_udp_close_t close;
  pal_udp_eof_t eof;
};

/**
 * Install PAL UDP IO implementation in to API
 *
 * Call this function during pal_impl_init to register the implementation's UDP
 * IO module with the libpal API.
 *
 * @param impl UDP IO implementation definition
 * @return PAL error code
 */
enum pal_error pal_set_impl_udp(struct pal_impl_udp *impl);

#ifdef __cplusplus
}
#endif

#endif
