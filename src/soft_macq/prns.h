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

#ifndef SWIFTNAV_PRNS_H
#define SWIFTNAV_PRNS_H

#include <swiftnav/common.h>
#include <swiftnav/signal.h>
#include "signal_db/signal_db.h"

#define INT_NUM_BYTES(arg) (((arg) + 7) / 8)

#define PRN_CODE_LENGTH_BYTES (INT_NUM_BYTES(GPS_L1CA_CHIPS_NUM))
#define PRN_GLO_CODE_LENGTH_BYTES (INT_NUM_BYTES(GLO_CA_CHIPS_NUM))

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

const u8* ca_code(const me_gnss_signal_t mesid);
s8 get_chip(const u8* const code, const u32 chip_num);

u32 mesid_to_lfsr0_init(const me_gnss_signal_t mesid);
u32 mesid_to_lfsr1_init(const me_gnss_signal_t mesid, const u8 index);

u32 mesid_to_lfsr0_last(const me_gnss_signal_t mesid);
u32 mesid_to_lfsr1_last(const me_gnss_signal_t mesid);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* SWIFTNAV_PRNS_H */
