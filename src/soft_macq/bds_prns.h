/*
 * Copyright (C) 2012,2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef BDS_PRNS_H
#define BDS_PRNS_H

#include <swiftnav/common.h>
#include <swiftnav/constants.h>
#include <swiftnav/signal.h>

#include "prns.h"

#define BDS2_B11_PRN_BYTES (INT_NUM_BYTES(BDS2_B11_CHIPS_NUM))
#define BDS3_B5Q_SC_MS 100
#define BDS3_SC100_BYTES (INT_NUM_BYTES(BDS3_B5Q_SC_MS))

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

extern const u32 bds2_prns_init_values[];
extern const u32 bds2_prns_last_values[];
extern const u8 bds2_codes[][BDS2_B11_PRN_BYTES];
extern const u32 bds3_b2a_prns_init_values[];

extern const u8 bds3_b2aq_sec_codes[]
                                   [BDS3_SC100_BYTES]; /* 100 chip, 12.5 Byte */

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* BDS_PRNS_H */
