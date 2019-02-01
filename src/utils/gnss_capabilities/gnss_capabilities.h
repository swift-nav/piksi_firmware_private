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

#include <stdbool.h>
#include <swiftnav/common.h>
#include <swiftnav/signal.h>

#include "nav_msg/nav_msg.h"
#include "signal_db/signal_db.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

bool gps_l2c_active(me_gnss_signal_t mesid);

bool glo_active(gnss_signal_t sid);
bool glo_l2of_active(gnss_signal_t sid);

bool sbas_active(me_gnss_signal_t mesid);

bool bds_active(me_gnss_signal_t mesid);
bool bds_d2nav(me_gnss_signal_t mesid);
bool bds_b2(me_gnss_signal_t mesid);

bool qzss_active(me_gnss_signal_t mesid);

bool gal_active(me_gnss_signal_t mesid);

void decode_l2c_capability(const u32 *subframe4_words, u32 *l2c_cpbl);

void gnss_capb_send_over_sbp(void);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* SWIFTNAV_GNSS_CAPABILITIES_H */
