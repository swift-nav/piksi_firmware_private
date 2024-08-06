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

#ifndef LIBPAL_IO_TCP_H
#define LIBPAL_IO_TCP_H

#include <libpal/impl/io/tcp.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Check global PAL TCP implementation
 * @return true if global PAL TCP implementation has been set, otherwise false
 */
bool pal_has_impl_tcp(void);

/**
 * Open a client connection to a remote TCP socket
 *
 * This function will create and initialize a TCP socket and establish a
 * connection to a remote server. On successful return this function will
 * provide a TCP socket handle in the sock parameter which can be used for
 * future reading/writing. Once the connection is no longer required is must be
 * passed to pal_io_close which will tear down the connection.
 *
 * @param sock On success will contain the connection socket handle
 * @param host Hostname or formatted IP address of remote server
 * @param port IP port of remote server
 * @param version IP version
 * @param timeout_us Maximum time to wait for connection in microseconds, value
 * of zero implies indefinite wait.
 * @return PAL error code
 */
enum pal_error pal_tcp_client_open(pal_tcp_t *sock, const char *host,
                                   uint16_t port, enum pal_ip_version version,
                                   uint64_t timeout_us);

/**
 * Open a TCP port which listens for incoming connections
 *
 * This function will create and initialize a TCP port which listens for
 * incoming connections on the given port. On successful return this function
 * will provide TCP server socket handle in the sock parameter which can be used
 * to accept incoming connections via a call to pal_tcp_accept. The socket
 * handle set up by this function may not be used for reading or writing, these
 * are invalid operations. Once the server port is no longer required it can be
 * passed to pal_io_close which will tear down the TCP socket and stop
 * listening.
 *
 * @param sock On success will contain the IO device used to accept incoming
 * connections
 * @param port Server port
 * @param version IP version
 * @param backlog Maximum number of queued incoming connections
 * @return PAL error code
 */
enum pal_error pal_tcp_server_open(pal_tcp_t *sock, uint16_t port,
                                   enum pal_ip_version version, size_t backlog);

/**
 * Accept an incoming TCP connection
 *
 * This function can operate in either blocking or non blocking mode as
 * controlled by the \p mode parameter. In non blocking mode this function shall
 * return PAL_WOULD_BLOCK immediately if an incoming connection is not available
 * at the time it is called. In blocking mode this function shall block until
 * either an incoming connection becomes available, an error occurs on the
 * socket, or the timeout period as given in \p timeout_us expires. The timeout
 * must be specified in microseconds, the special value of 0 means block
 * indefinitely. Should the timeout expire this function will return
 * PAL_TIMEOUT.
 *
 * If this function returns PAL_SUCCESS then a new client connection has been
 * accepted. The output parameter \p cli will be filled with a TCP socket handle
 * which can be used for future calls to #pal_io_read or #pal_io_write to
 * communicate with the client. It must be passed to #pal_io_close when the
 * caller no longer needs it.
 *
 * When a new client connection is established this function will return
 * information about the client in the \p host, \p port, and \p ip_version
 * parameters. The caller must set \p host to a buffer of at least 46 bytes, on
 * success this function will fill the buffer with the IPV4 or IPV6 address of
 * the new client as appropriate. \p port and \p ip_version will be set
 * similarly.
 *
 * @param srv Server TCP socket handle created by pal_tcp_server_open
 * @param cli On success will contain the newly create TCP socket handle for the
 * client connection
 * @param host On success will be filled with the IP address of the new client
 * @param port On success will be filled with the port number of the new client
 * @param ip_version On success will be filled with the IP version of the new
 * client connection
 * @mode Blocking mode
 * @param timeout_us Maximum time to wait for incoming connection in
 * microseconds
 * @return PAL error code
 */
enum pal_error pal_tcp_accept(pal_tcp_t srv, pal_tcp_t *cli, char *host,
                              uint16_t *port, enum pal_ip_version *ip_version,
                              enum pal_blocking_mode mode, uint64_t timeout_us);

/**
 * Read data from a TCP socket
 *
 * This function will read up to \p n bytes from the specified TCP socket and
 * write them into the provided buffer.
 *
 * Reading can be done either as a blocking or non blocking operation as
 * controlled by the \p mode parameter. When operating in non blocking mode this
 * function shall return immediately with a code of #PAL_WOULD_BLOCK if it is
 * not able to immediately read any data from the TCP socket. In blocking
 * mode the call shall block until either data becomes available to read, an
 * error occurs on the TCP socket, or the timeout period given in \p timeout_us
 * expires. The timeout must be specified in microseconds, the special value of
 * 0 means block indefinitely. Should the timeout expire the function will
 * return PAL_TIMEOUT.
 *
 *
 * If the caller sets the \p n parameter as zero, than the function will return
 * the current error status of the TCP socket.
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
 * @param sock TCP socket
 * @param buf Buffer to read data in to
 * @param n Number of bytes to read
 * @param nread Number of bytes actually read
 * @param mode Blocking mode
 * @param timeout_us Timeout in microseconds
 * @return PAL error code
 */
enum pal_error pal_tcp_read(pal_tcp_t sock, uint8_t *buf, size_t n,
                            size_t *nread, enum pal_blocking_mode mode,
                            uint64_t timeout_us);

/**
 * Write data to a TCP socket
 *
 * This function will write up to \p n bytes from the specified buffer to the
 * TCP socket
 *
 * Writing can be done either as a blocking or non blocking operation as
 * controlled by the \p mode parameter. When operating in non blocking mode this
 * function shall return immediately with a code of #PAL_WOULD_BLOCK if it is
 * not able to immediately read any data from the TCP socket. In blocking
 * mode the call shall block until either some or all of the data has been
 * written, an error occurs on the TCP socket, or the timeout period given in \p
 * timeout_us expires. The timeout must be specified in microseconds, the
 * special value of 0 means block indefinitely. Should the timeout expire the
 * function will return PAL_TIMEOUT.
 *
 * If the caller sets the \p n parameter as zero, then the function will return
 * the current error status of the TCP socket.
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
 * @param sock TCP socket handle
 * @param buf Buffer from which to write data
 * @param n Number of bytes to write
 * @param nwritten Number of bytes actually written
 * @param mode Blocking mode
 * @param timeout_us Timeout in microseconds
 * @return PAL error code
 */
enum pal_error pal_tcp_write(pal_tcp_t sock, const uint8_t *buf, size_t n,
                             size_t *nwritten, enum pal_blocking_mode mode,
                             uint64_t timeout_us);

/**
 * Close an open TCP socket
 *
 * Calling this function will make the TCP socket unavailable for more IO
 * operations of any kind such as read and writes (for client type sockets) and
 * accepting new clients (for server type sockets). After this function returns
 * success the handle will be set to NULL must not be reused.
 *
 * @param sock Pointer to open TCP socket
 * @return PAL error code
 */
enum pal_error pal_tcp_close(pal_tcp_t *sock);

/**
 * Test whether a TCP socket is in EOF condition
 *
 * The term EOF means that this TCP socket is unavailable for further IO
 * activity but has not yet been closed. This may be because the TCP connection
 * has been torn down by the other end, or that some sort of network error has
 * occured. The only valid operation on a TCP socket in this state is to call
 * #pal_tcp_close to clean up any resources.
 *
 * @param sock TCP socket handle
 * @param eof On success will be set true if the TCP socket is in EOF state,
 * false otherwise
 * @return PAL error code
 */
enum pal_error pal_tcp_eof(pal_tcp_t sock, bool *eof);

/**
 * Specifies the TCP keepalive options for connection.
 *
 * Parameters that are pointers, NULL value will indicate that it should use
 * whatever settings was previously assigned to it. For instance, if a TCP
 * connection was setup, followed by a call to this method whereby the enabled
 * flag was set to TRUE, with all other parameters set to NULL, than their
 * respective values will be based on the OS's default value for them.
 *
 * @param sock TCP socket handle
 * @param enable indicates if TCP keep alive should be enabled for connection
 * @param idle_us time the connection needs to remain idle before TCP starts
 * sending keepalive probes (microseconds)
 * @param interval_us time between individual keepalive probes (microseconds)
 * @param retries maximum number of keepalive probes TCP should send before
 * dropping the connection
 *
 * @note none of the parameters should be zero.
 *
 * @return PAL error code
 */
enum pal_error pal_tcp_keep_alive(pal_tcp_t sock, bool enable, uint64_t idle_us,
                                  uint64_t interval_us, uint16_t retries);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // LIBPAL_IO_TCP_H
