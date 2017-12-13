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

#include <libswiftnav/common.h>
#include <libswiftnav/signal.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

const u8* ca_code(const me_gnss_signal_t mesid);
s8 get_chip(u8* code, u32 chip_num);

u32 mesid_to_lfsr0_init(const me_gnss_signal_t mesid, u8 index);
u32 mesid_to_lfsr1_init(const me_gnss_signal_t mesid, u8 index);

u32 mesid_to_lfsr0_last(const me_gnss_signal_t mesid);
u32 mesid_to_lfsr1_last(const me_gnss_signal_t mesid);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* SWIFTNAV_PRNS_H */
