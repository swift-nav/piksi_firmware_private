/*
 * Copyright (C) 2021 Swift Navigation Inc.
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

#include <libpal/identifier/identifier.h>
#include <libpal/impl/identifier/identifier.h>
#include <libpal/require.h>

static struct pal_impl_identifier identifier_impl = {
    .identifier = NULL,
};

void pal_reset_impl_identifier(void) {
  memset(&identifier_impl, 0, sizeof(identifier_impl));
}

enum pal_error pal_set_impl_identifier(struct pal_impl_identifier *impl) {
  enum pal_error ret = pal_require(NULL != impl && NULL != impl->identifier);
  if (ret == PAL_SUCCESS) {
    identifier_impl = *impl;
  }
  return ret;
}

bool pal_has_impl_identifier() { return identifier_impl.identifier != NULL; }

enum pal_error pal_identifier(const char **id) {
  enum pal_error ret = pal_require(pal_has_impl_identifier() && (id != NULL));
  if (ret == PAL_SUCCESS) {
    ret = identifier_impl.identifier(id);
  }
  return ret;
}
