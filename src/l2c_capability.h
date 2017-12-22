/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_L2C_CAPABILITY_H
#define SWIFTNAV_L2C_CAPABILITY_H

#include <libswiftnav/common.h>

#include "nav_msg/nav_msg.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void decode_l2c_capability(const u32 *subframe4_words, u32 *l2c_cpbl);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* SWIFTNAV_L2C_CAPABILITY_H */
