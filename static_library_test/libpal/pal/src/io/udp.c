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
#include <libpal/impl/io/udp.h>
#include <libpal/io/udp.h>
#include <libpal/require.h>
#include <pal_private.h>

static struct pal_impl_udp udp_impl = {
    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
};

void pal_reset_impl_udp(void) { memset(&udp_impl, 0, sizeof(udp_impl)); }

enum pal_error pal_set_impl_udp(struct pal_impl_udp *impl) {
  enum pal_error ret =
      pal_require(impl != NULL && impl->open != NULL && impl->bind_to != NULL &&
                  impl->connect_to != NULL && impl->send_to != NULL &&
                  impl->receive_from != NULL

      );
  if (ret == PAL_SUCCESS) {
    udp_impl = *impl;
  }
  return ret;
}

bool pal_has_impl_udp() { return udp_impl.open != NULL; }

enum pal_error pal_udp_open(pal_udp_t *sock, enum pal_ip_version version) {
  enum pal_error ret =
      pal_require(pal_has_impl_udp() && sock != NULL &&
                  pal_validate_ip_version(version) && version != PAL_IP_ANY);
  if (ret == PAL_SUCCESS) {
    ret = udp_impl.open(sock, version);
  }
  return ret;
}

enum pal_error pal_udp_bind_to(pal_udp_t *sock, uint16_t port,
                               enum pal_ip_version version) {
  enum pal_error ret =
      pal_require(pal_has_impl_udp() && sock != NULL && port != 0 &&
                  pal_validate_ip_version(version) && version != PAL_IP_ANY);
  if (ret == PAL_SUCCESS) {
    ret = udp_impl.bind_to(sock, port, version);
  }
  return ret;
}

enum pal_error pal_udp_connect_to(pal_udp_t *sock, const char *host,
                                  uint16_t port, enum pal_ip_version version) {
  enum pal_error ret = pal_require(
      pal_has_impl_udp() && sock != NULL && host != NULL && port != 0 &&
      pal_validate_ip_version(version) && version != PAL_IP_ANY);
  if (ret == PAL_SUCCESS) {
    ret = udp_impl.connect_to(sock, host, port, version);
  }
  return ret;
}

enum pal_error pal_udp_send_to(pal_udp_t sock, const uint8_t *buffer,
                               size_t max_count, size_t *send_count,
                               const char *ip_address, uint16_t port,
                               enum pal_ip_version version,
                               enum pal_blocking_mode mode,
                               uint64_t timeout_us) {
  INIT_OUTPUT_PARAM(send_count, 0);
  enum pal_error ret =
      pal_require(pal_has_impl_udp() && sock != NULL && buffer != NULL &&
                  send_count != NULL && ip_address != NULL && port != 0 &&
                  pal_validate_ip_version(version) && version != PAL_IP_ANY &&
                  pal_validate_blocking_mode(mode));
  if (ret == PAL_SUCCESS) {
    ret = udp_impl.send_to(sock, buffer, max_count, send_count, ip_address,
                           port, version, mode, timeout_us);
  }
  return ret;
}

enum pal_error pal_udp_receive_from(pal_udp_t sock, uint8_t *buffer,
                                    size_t max_count, size_t *receive_count,
                                    char *ip_address, uint16_t *port,
                                    enum pal_ip_version *version,
                                    enum pal_blocking_mode mode,
                                    uint64_t timeout_us) {
  INIT_OUTPUT_PARAM(receive_count, 0);
  enum pal_error ret =
      pal_require(pal_has_impl_udp() && sock != NULL && buffer != NULL &&
                  receive_count != NULL && ip_address != NULL && port != NULL &&
                  version != NULL && pal_validate_blocking_mode(mode));
  if (ret == PAL_SUCCESS) {
    ret = udp_impl.receive_from(sock, buffer, max_count, receive_count,
                                ip_address, port, version, mode, timeout_us);
  }
  return ret;
}

enum pal_error pal_udp_read(pal_udp_t sock, uint8_t *buf, size_t n,
                            size_t *nread, enum pal_blocking_mode mode,
                            uint64_t timeout_us) {
  INIT_OUTPUT_PARAM(nread, 0);
  enum pal_error ret =
      pal_require(pal_has_impl_udp() && sock != NULL && buf != NULL &&
                  nread != NULL && pal_validate_blocking_mode(mode));
  if (ret == PAL_SUCCESS) {
    ret = udp_impl.read(sock, buf, n, nread, mode, timeout_us);
  }
  return ret;
}

enum pal_error pal_udp_write(pal_udp_t sock, const uint8_t *buf, size_t n,
                             size_t *nwritten, enum pal_blocking_mode mode,
                             uint64_t timeout_us) {
  INIT_OUTPUT_PARAM(nwritten, 0);
  enum pal_error ret =
      pal_require(pal_has_impl_udp() && sock != NULL && buf != NULL &&
                  nwritten != NULL && pal_validate_blocking_mode(mode));
  if (ret == PAL_SUCCESS) {
    ret = udp_impl.write(sock, buf, n, nwritten, mode, timeout_us);
  }
  return ret;
}

enum pal_error pal_udp_close(pal_udp_t *sock) {
  enum pal_error ret = pal_require(sock != NULL);
  if (ret != PAL_SUCCESS) {
    return ret;
  }
  if (!*sock) {
    return PAL_SUCCESS;
  }
  ret = pal_require(pal_has_impl_udp());
  if (ret == PAL_SUCCESS) {
    ret = udp_impl.close(*sock);
    *sock = NULL;
  }
  return ret;
}

enum pal_error pal_udp_eof(pal_udp_t sock, bool *eof) {
  enum pal_error ret =
      pal_require(pal_has_impl_udp() && sock != NULL && eof != NULL);
  if (ret == PAL_SUCCESS) {
    ret = udp_impl.eof(sock, eof);
  }
  return ret;
}
