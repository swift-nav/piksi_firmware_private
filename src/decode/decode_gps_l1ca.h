/*
 * Copyright (C) 2015 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#ifndef SWIFTNAV_DECODE_GPS_L1CA_H
#define SWIFTNAV_DECODE_GPS_L1CA_H

#include "decode_common.h"
#include "nav_msg/nav_msg.h"

#ifdef __cplusplus
extern "C" {
#endif

/** GPS L1 C/A decoder data */
typedef struct {
  nav_msg_t nav_msg;
  u16 bit_cnt; /**< For navbit data integrity checks */
} gps_l1ca_decoder_data_t;

void decoder_gps_l1ca_init(const decoder_channel_info_t *channel_info,
                           decoder_data_t *decoder_data);

void decoder_gps_l1ca_process(const decoder_channel_info_t *channel_info,
                              decoder_data_t *decoder_data);

void decode_gps_l1ca_register(void);

#ifdef __cplusplus
}
#endif

#endif
