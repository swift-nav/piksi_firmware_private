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

#ifndef QZSS_PRNS_H
#define QZSS_PRNS_H

#include <swiftnav/common.h>
#include <swiftnav/constants.h>
#include <swiftnav/signal.h>

#include "prns.h"

#define QZS_L1CA_CODE_BYTES (INT_NUM_BYTES(QZS_L1CA_CHIPS_NUM))

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

extern const u32 qzss_l1ca_prns_init_values[];
extern const u32 qzss_l1ca_prns_last_values[];

extern const u32 qzss_l2cm_prns_init_values[];
extern const u32 qzss_l2cm_prns_last_values[];

extern const u32 qzss_l2cl_prns_init_values[10][75];
extern const u32 qzss_l2cl_prns_last_values[];

extern const u8 qzs_l1ca_codes[][QZS_L1CA_CODE_BYTES];

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* QZSS_PRNS_H */
