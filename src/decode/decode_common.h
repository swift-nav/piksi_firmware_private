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

/** GLO data decoding status */
typedef enum {
  GLO_DECODE_ERROR,       /**< Decoding restart due to error */
  GLO_DECODE_SENSITIVITY, /**< Decoding restart due to sensitivity mode */
  GLO_DECODE_WAIT,        /**< Decoding in progress */
  GLO_DECODE_STRING,      /**< Single string decoded */
  GLO_DECODE_DONE,        /**< GLO strings 1 - 5 decoded */
} glo_decode_status_t;

void nav_msg_init_glo_with_cb(nav_msg_glo_t *n, me_gnss_signal_t mesid);
glo_decode_status_t glo_data_decoding(nav_msg_glo_t *n,
                                      me_gnss_signal_t mesid,
                                      const nav_bit_fifo_element_t *nav_bit);
void save_glo_eph(nav_msg_glo_t *n, me_gnss_signal_t mesid);
bool glo_data_sync(nav_msg_glo_t *n,
                   me_gnss_signal_t mesid,
                   u8 tracking_channel,
                   bool polarity_update_only);

#endif /* #ifndef SWIFTNAV_DECODE_COMMON_H */
