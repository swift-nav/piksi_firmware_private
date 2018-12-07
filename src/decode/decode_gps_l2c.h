/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Adel Mamin <adel.mamin@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#ifndef SWIFTNAV_DECODE_GPS_L2C_H
#define SWIFTNAV_DECODE_GPS_L2C_H

#include "decode_common.h"
#include "nav_msg/nav_msg.h"

#ifdef __cplusplus
extern "C" {
#endif

/** GPS L2 C decoder data */
typedef struct {
  cnav_msg_t cnav_msg;
  cnav_msg_decoder_t cnav_msg_decoder;
  u16 bit_cnt; /**< For navbit data integrity checks */
} gps_l2c_decoder_data_t;

void decoder_gps_l2c_init(const decoder_channel_info_t *channel_info,
                          decoder_data_t *decoder_data);

void decoder_gps_l2c_process(const decoder_channel_info_t *channel_info,
                             decoder_data_t *decoder_data);

void decode_gps_l2c_register(void);

#ifdef __cplusplus
}
#endif

#endif /* SWIFTNAV_DECODE_GPS_L2C_H */
