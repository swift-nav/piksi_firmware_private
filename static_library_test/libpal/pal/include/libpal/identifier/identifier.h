/*
 * Copyright (C) 2021 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBPAL_IDENTIFIER_H
#define LIBPAL_IDENTIFIER_H

#include <stdbool.h>
#include <stddef.h>

#include <libpal/error.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Check Global PAL Identifier Implementation
 * @return True if Global PAL Identifier Implementation has been set, otherwise
 * False.
 */
bool pal_has_impl_identifier(void);

/**
 * Retrieve globally unique identifier for a system
 *
 * @param id On success must be updated to point to the unique identifier
 * @return PAL error code
 */
enum pal_error pal_identifier(const char **id);

#ifdef __cplusplus
}
#endif

#endif  // LIBPAL_IDENTIFIER_H
