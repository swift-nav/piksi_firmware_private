/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Adel Mamin <adel.mamin@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_DECODE_COMMON_H
#define SWIFTNAV_DECODE_COMMON_H

#include "track.h"

void nav_msg_init_glo_with_cb(nav_msg_glo_t *n, me_gnss_signal_t mesid);
bool is_glo_decode_ready(nav_msg_glo_t *n,
                         me_gnss_signal_t mesid,
                         const nav_bit_fifo_element_t *nav_bit);
void save_glo_eph(nav_msg_glo_t *n, me_gnss_signal_t mesid);
bool glo_data_sync(nav_msg_glo_t *n,
                   me_gnss_signal_t mesid,
                   u8 tracking_channel);
void send_glo_fcn_mapping(gps_time_t t);

#endif /* #ifndef SWIFTNAV_DECODE_COMMON_H */
