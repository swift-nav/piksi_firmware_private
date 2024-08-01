/**
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBPAL_IO_TCP_H
#define LIBPAL_IO_TCP_H

#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>

#include <libpal/io/io.h>

#ifdef __cplusplus
extern "C" {
#endif

// PAL TCP protocol versions
enum pal_tcp_protocol {
  // Use any available TCP version
  PAL_TCP_ANY,
  // Use only TCP version 4
  PAL_TCP4,
  // Use only TCP version 6
  PAL_TCP6,
};

/**
 * Check global PAL TCP implementation
 * @return true if global PAL TCP implementation has been set, otherwise false
 */
bool pal_has_impl_tcp(void);

/**
 * Open a client connection to a remote TCP socket
 *
 * This function will create and initialize a TCP IO handle and establish a
 * connection to a remote server. On successful return this function will
 * provide an IO handle in the io_out parameter which can be used for future
 * reading/writing, or passed to pal_io_watch. Once the connection is no longer
 * required is must be passed to pal_io_close which will tear down the
 * connection.
 *
 * @param host Hostname or formatted IP address of remote server
 * @param port TCP port of remote server
 * @param protocol TCP protocol version
 * @param io_out On success will contain the connection IO handle
 * @return PAL error code
 */
enum pal_error pal_tcp_client_open(const char *host, uint16_t port,
                                   enum pal_tcp_protocol protocol,
                                   pal_io_t *sock);

/**
 * Open a TCP port which listens for incoming connections
 *
 * This function will create and initialize a TCP port which listens for
 * incoming connections on the given port. On successful return this function
 * will provide an IO handle in the io_out parameter which can be used to accept
 * incoming connections via a call to pal_tcp_accept. The IO handle set up by
 * this function may not be used for reading or writing, these are invalid
 * operations. Once the server port is not longer required it can be passed to
 * pal_io_close which will tear down the TCP socket and stop listening.
 *
 * @param port Server port
 * @param protocol TCP protocol version
 * @param backlog Maximum number of queued incoming connections
 * @param io_out On success will contain the IO device used to accept incoming
 * connections
 * @return PAL error code
 */
enum pal_error pal_tcp_server_open(uint16_t port,
                                   enum pal_tcp_protocol protocol,
                                   size_t backlog, pal_io_t *sock);

/**
 * Accept an incoming TCP connection
 *
 * @param _srv Server IO handle created by pal_tcp_server_open
 * @param cli_out On success will contain the newly create IO handle for the
 * client connection
 * @param timeout_us Maximum time to wait for incoming connection in
 * microseconds
 * @return PAL error code
 */
enum pal_error pal_tcp_accept(pal_io_t srv, pal_io_t *cli, uint64_t timeout_us);

/**
 * Checks whether a server port is available to take new connections
 *
 * This does not mean that a client is trying to connect, only that the server
 * is listening
 *
 * @param _io IO handle created by pal_tcp_server_open
 * @param listening On success will be set to true if the TCP socket is
 * listening for incoming connections, false otherwise
 * @return PAL error code
 */
enum pal_error pal_tcp_listening(pal_io_t io, bool *listening);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // LIBPAL_IO_TCP_H
