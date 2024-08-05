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

#include <stddef.h>

#include <libpal/error.h>
#include <libpal/impl/io/tcp.h>
#include <libpal/io/tcp.h>
#include <pal_private.h>

#include "not_implemented.h"

/**
 * Default TCP open client connection routine
 * @param host Unused
 * @param port Unused
 * @param protocol Unused
 * @param sock Unused
 * @return PAL_INVALID
 */
static enum pal_error default_tcp_client_open(const char *host, uint16_t port,
                                              enum pal_tcp_protocol protocol,
                                              pal_io_t *sock) {
  NOT_IMPLEMENTED();
  (void)host;
  (void)port;
  (void)protocol;
  (void)sock;
  return PAL_INVALID;
}

/**
 * Default TCP open server routine
 * @param port Unused
 * @param protocol Unused
 * @param backlog Unused
 * @param sock Unused
 * @return PAL_INVALID
 */
static enum pal_error default_tcp_server_open(uint16_t port,
                                              enum pal_tcp_protocol protocol,
                                              size_t backlog, pal_io_t *sock) {
  NOT_IMPLEMENTED();
  (void)port;
  (void)protocol;
  (void)backlog;
  (void)sock;
  return PAL_INVALID;
}

/**
 * Default TCP accept incoming client connection routine
 * @param srv Unused
 * @param cli Unused
 * @param timeout_us Unuwed
 * @return PAL_INVALID
 */
static enum pal_error default_tcp_accept(pal_io_t srv, pal_io_t *cli,
                                         uint64_t timeout_us) {
  NOT_IMPLEMENTED();
  (void)srv;
  (void)cli;
  (void)timeout_us;
  return PAL_INVALID;
}

/**
 * Default TCP check server port readiness routine
 * @param io Unused
 * @param listening Unused
 * @return PAL_INVALID
 */
static enum pal_error default_tcp_listening(pal_io_t io, bool *listening) {
  (void)io;
  (void)listening;
  NOT_IMPLEMENTED();
  return PAL_INVALID;
}

static struct pal_impl_tcp tcp_impl = {
    .client_open = default_tcp_client_open,
    .server_open = default_tcp_server_open,
    .accept = default_tcp_accept,
    .listening = default_tcp_listening,
};

void pal_reset_impl_tcp(void) {
  tcp_impl.client_open = default_tcp_client_open;
  tcp_impl.server_open = default_tcp_server_open;
  tcp_impl.accept = default_tcp_accept;
  tcp_impl.listening = default_tcp_listening;
}

void pal_set_impl_tcp(struct pal_impl_tcp *impl) {
  assert(NULL != impl);
  assert(NULL != impl->client_open);
  assert(NULL != impl->server_open);
  assert(NULL != impl->accept);
  assert(NULL != impl->listening);

  tcp_impl = *impl;
}

bool pal_has_impl_tcp() {
  return tcp_impl.client_open != default_tcp_client_open;
}

enum pal_error pal_tcp_client_open(const char *host, uint16_t port,
                                   enum pal_tcp_protocol protocol,
                                   pal_io_t *sock) {
  return tcp_impl.client_open(host, port, protocol, sock);
}

enum pal_error pal_tcp_server_open(uint16_t port,
                                   enum pal_tcp_protocol protocol,
                                   size_t backlog, pal_io_t *sock) {
  return tcp_impl.server_open(port, protocol, backlog, sock);
}

enum pal_error pal_tcp_accept(pal_io_t srv, pal_io_t *cli,
                              uint64_t timeout_us) {
  return tcp_impl.accept(srv, cli, timeout_us);
}

enum pal_error pal_tcp_listening(pal_io_t io, bool *listening) {
  return tcp_impl.listening(io, listening);
}
