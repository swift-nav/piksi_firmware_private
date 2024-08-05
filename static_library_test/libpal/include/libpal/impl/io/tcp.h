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

#ifndef LIBPAL_IMPL_IO_TCP_H
#define LIBPAL_IMPL_IO_TCP_H

#include <libpal/io/io.h>
#include <libpal/io/tcp.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * PAL TCP IO Implementation
 *
 * This file defines the interface a PAL implementation must use to install its
 * own TCP handling ability in to the libpal API
 *
 * The function pointer names and signatures in this file match those in
 * libpal/io/tcp.h. The PAL implementation must provide a version of these
 * functions which meet the requirements stated in the documentation contained
 * in that file.
 */

/**
 * Open a TCP client connection
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_tcp_client_open() (see libpal/io/tcp.h)
 *
 */
typedef enum pal_error (*pal_tcp_client_open_t)(const char *host, uint16_t port,
                                                enum pal_tcp_protocol protocol,
                                                pal_io_t *sock);
/**
 * Open a TCP server port
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_tcp_server_open() (see libpal/io/tcp.h)
 *
 */
typedef enum pal_error (*pal_tcp_server_open_t)(uint16_t port,
                                                enum pal_tcp_protocol protocol,
                                                size_t backlog, pal_io_t *sock);

/**
 * Accept an incoming connection
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_tcp_accept() (see libpal/io/tcp.h)
 *
 */
typedef enum pal_error (*pal_tcp_accept_t)(pal_io_t srv, pal_io_t *cli,
                                           uint64_t timeout_us);

/**
 * Test whether a server socket is listening
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_tcp_listening() (see libpal/io/tcp.h)
 *
 */
typedef enum pal_error (*pal_tcp_listening_t)(pal_io_t io, bool *listening);

/**
 * PAL TCP IO implementation definition
 */
struct pal_impl_tcp {
  /// Implementation TCP IO open client connection routine
  pal_tcp_client_open_t client_open;
  /// Implementation TCP IO open server socket routine
  pal_tcp_server_open_t server_open;
  /// Implementation TCP accept incoming client connection routine
  pal_tcp_accept_t accept;
  /// Implementation TCP test server listening routine
  pal_tcp_listening_t listening;
};

/**
 * Install PAL TCP IO implementation in to API
 *
 * Call this function during pal_impl_init to register the implentation's TCP IO
 * module with the libpal API
 *
 * @param impl TCP IO implementation definition
 */
void pal_set_impl_tcp(struct pal_impl_tcp *impl);

#ifdef __cplusplus
}
#endif

#endif
