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

#ifndef GAL_PRNS_H
#define GAL_PRNS_H

#include <swiftnav/common.h>
#include <swiftnav/constants.h>
#include <swiftnav/signal.h>

#include "prns.h"

#define GAL_E1B_PRN_BYTES (INT_NUM_BYTES(GAL_E1B_CHIPS_NUM))
#define GAL_E1C_PRN_BYTES (INT_NUM_BYTES(GAL_E1C_CHIPS_NUM))
#define GAL_E5_PRN_BYTES (INT_NUM_BYTES(GAL_E5_CHIPS_NUM))
#define GAL_E7_PRN_BYTES (INT_NUM_BYTES(GAL_E7_CHIPS_NUM))
#define GAL_CS100_BYTES 13
#define GAL_CS25_BYTES 4

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

extern const u8 gal_e1c_sec25[GAL_CS25_BYTES];

extern const u32 gal_e5i_prns_init_values[];
extern const u32 gal_e5i_prns_last_values[];

extern const u32 gal_e5q_prns_init_values[];
extern const u32 gal_e5q_prns_last_values[];

extern const u8 gal_e5q_sec_codes[][GAL_CS100_BYTES]; /* 100 chip, 12.5 Byte */

extern const u32 gal_e7i_prns_init_values[];
extern const u32 gal_e7i_prns_last_values[];

extern const u32 gal_e7q_prns_init_values[];
extern const u32 gal_e7q_prns_last_values[];

extern const u8 gal_e7q_sec_codes[][GAL_CS100_BYTES]; /* 100 chip, 12.5 Byte */

extern const u8 gal_e1b_codes[][GAL_E1B_PRN_BYTES];
extern const u8 gal_e1c_codes[][GAL_E1C_PRN_BYTES];

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* GAL_PRNS_H */
