/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Michele Bavaro <michele@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_GNSS_CAPABILITIES_H
#define SWIFTNAV_GNSS_CAPABILITIES_H

#include <libswiftnav/common.h>
#include <libswiftnav/signal.h>

#include "nav_msg/nav_msg.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

typedef struct _gnss_capabilities_t {
  u64 gps_active;
  u64 gps_l2c;
  u64 gps_l5;

  u32 glo_active;
  u32 glo_l2of;
  u32 glo_l3;

  u32 sbas_active;
  u32 sbas_l5;

  u64 bds_active;
  u64 bds_d2nav;
  u64 bds_b2;

  u32 qzss_active;

  u64 gal_active;
  u64 gal_e5;
  u64 gps_e6;
} gnss_capabilities_t;

extern gnss_capabilities_t gnss_capab;

bool glo_active(const gnss_signal_t sid);

bool sbas_active(const me_gnss_signal_t mesid);

bool bds_active(const me_gnss_signal_t mesid);
bool bds_d2nav(const me_gnss_signal_t mesid);
bool bds_b2(const me_gnss_signal_t mesid);

bool qzss_active(const me_gnss_signal_t mesid);

bool gal_active(const me_gnss_signal_t mesid);

void decode_l2c_capability(const u32 *subframe4_words, u32 *l2c_cpbl);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* SWIFTNAV_GNSS_CAPABILITIES_H */
