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

#ifndef LIBPAL_IMPL_IDENTIFIER_IDENTIFIER_H
#define LIBPAL_IMPL_IDENTIFIER_IDENTIFIER_H

#include <libpal/error.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Retrieve globally unique identifier for a system
 *
 * @param id On success must be updated to point to the unique identifier
 * @return PAL error code
 */
typedef enum pal_error (*pal_identifier_t)(const char **id);

/**
 * PAL identifier implementation definition
 */
struct pal_impl_identifier {
  // Implementation of identifier routine
  pal_identifier_t identifier;
};

/**
 * Install PAL identifier implementation in to API
 *
 * Call this function during pal_impl_init to register the implementation's
 * identifier module with the libpal API
 *
 * @param impl Identifier implementation definition
 * @return PAL error code
 */
enum pal_error pal_set_impl_identifier(struct pal_impl_identifier *impl);

#ifdef __cplusplus
}
#endif

#endif  // LIBPAL_IMPL_IDENTIFIER_IDENTIFIER_H
