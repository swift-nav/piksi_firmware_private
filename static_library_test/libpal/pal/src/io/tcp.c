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

#include <string.h>

#include <libpal/error.h>
#include <libpal/impl/io/tcp.h>
#include <libpal/io/tcp.h>
#include <libpal/require.h>
#include <pal_private.h>

static struct pal_impl_tcp tcp_impl = {
    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
};

void pal_reset_impl_tcp(void) { memset(&tcp_impl, 0, sizeof(tcp_impl)); }

enum pal_error pal_set_impl_tcp(struct pal_impl_tcp *impl) {
  enum pal_error ret = pal_require(
      NULL != impl && NULL != impl->client_open && NULL != impl->server_open &&
      NULL != impl->accept && NULL != impl->keep_alive && NULL != impl->read &&
      NULL != impl->write && NULL != impl->close && NULL != impl->eof);
  if (ret == PAL_SUCCESS) {
    tcp_impl = *impl;
  }
  return ret;
}

bool pal_has_impl_tcp() { return tcp_impl.client_open != NULL; }

enum pal_error pal_tcp_client_open(pal_tcp_t *sock, const char *host,
                                   uint16_t port, enum pal_ip_version version,
                                   uint64_t timeout_us) {
  enum pal_error ret =
      pal_require(pal_has_impl_tcp() && sock != NULL && host != NULL &&
                  port != 0 && pal_validate_ip_version(version));
  if (ret == PAL_SUCCESS) {
    ret = tcp_impl.client_open(sock, host, port, version, timeout_us);
  }
  return ret;
}

enum pal_error pal_tcp_server_open(pal_tcp_t *sock, uint16_t port,
                                   enum pal_ip_version version,
                                   size_t backlog) {
  enum pal_error ret =
      pal_require(pal_has_impl_tcp() && sock != NULL && port != 0 &&
                  pal_validate_ip_version(version) && backlog != 0);
  if (ret == PAL_SUCCESS) {
    ret = tcp_impl.server_open(sock, port, version, backlog);
  }
  return ret;
}

enum pal_error pal_tcp_accept(pal_tcp_t srv, pal_tcp_t *cli, char *host,
                              uint16_t *port, enum pal_ip_version *ip_version,
                              enum pal_blocking_mode mode,
                              uint64_t timeout_us) {
  enum pal_error ret = pal_require(
      pal_has_impl_tcp() && srv != NULL && cli != NULL && host != NULL &&
      port != NULL && ip_version != NULL && pal_validate_blocking_mode(mode));
  if (ret == PAL_SUCCESS) {
    ret = tcp_impl.accept(srv, cli, host, port, ip_version, mode, timeout_us);
  }
  return ret;
}

enum pal_error pal_tcp_keep_alive(pal_tcp_t sock, bool enable, uint64_t idle_us,
                                  uint64_t interval_us, uint16_t retries) {
  enum pal_error ret = pal_require(pal_has_impl_tcp() && sock != NULL);
  if (ret == PAL_SUCCESS && enable) {
    ret = pal_require(idle_us != 0 && interval_us != 0 && retries != 0);
  }
  if (ret == PAL_SUCCESS) {
    ret = tcp_impl.keep_alive(sock, enable, idle_us, interval_us, retries);
  }
  return ret;
}

enum pal_error pal_tcp_read(pal_tcp_t sock, uint8_t *buf, size_t n,
                            size_t *nread, enum pal_blocking_mode mode,
                            uint64_t timeout_us) {
  INIT_OUTPUT_PARAM(nread, 0);
  enum pal_error ret =
      pal_require(pal_has_impl_tcp() && sock != NULL && buf != NULL &&
                  nread != NULL && pal_validate_blocking_mode(mode));
  if (ret == PAL_SUCCESS) {
    ret = tcp_impl.read(sock, buf, n, nread, mode, timeout_us);
  }
  return ret;
}

enum pal_error pal_tcp_write(pal_tcp_t sock, const uint8_t *buf, size_t n,
                             size_t *nwritten, enum pal_blocking_mode mode,
                             uint64_t timeout_us) {
  INIT_OUTPUT_PARAM(nwritten, 0);
  enum pal_error ret =
      pal_require(pal_has_impl_tcp() && sock != NULL && buf != NULL &&
                  nwritten != NULL && pal_validate_blocking_mode(mode));
  if (ret == PAL_SUCCESS) {
    ret = tcp_impl.write(sock, buf, n, nwritten, mode, timeout_us);
  }
  return ret;
}

enum pal_error pal_tcp_close(pal_tcp_t *sock) {
  enum pal_error ret = pal_require(sock != NULL);
  if (ret != PAL_SUCCESS) {
    return ret;
  }
  if (!*sock) {
    return PAL_SUCCESS;
  }
  ret = pal_require(pal_has_impl_tcp());
  if (ret == PAL_SUCCESS) {
    ret = tcp_impl.close(*sock);
    *sock = NULL;
  }
  return ret;
}

enum pal_error pal_tcp_eof(pal_tcp_t sock, bool *eof) {
  enum pal_error ret =
      pal_require(pal_has_impl_tcp() && sock != NULL && eof != NULL);
  if (ret == PAL_SUCCESS) {
    ret = tcp_impl.eof(sock, eof);
  }
  return ret;
}
