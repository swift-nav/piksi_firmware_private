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

#include <libswiftnav/nav_msg_glo.h>

#include "cnav_msg_storage.h"
#include "track.h"

/** GLO data decoding status */
typedef enum {
  GLO_DECODE_WAIT,            /**< Decoding in progress */
  GLO_DECODE_SENSITIVITY,     /**< Decoding in sensitivity mode */
  GLO_DECODE_POLARITY_LOSS,   /**< Data polarity is lost (polarity update) */
  GLO_DECODE_POLARITY_UPDATE, /**< String decoded (polarity update) */
  GLO_DECODE_TOW_UPDATE,      /**< TOW decoded (TOW + polarity update) */
  GLO_DECODE_EPH_UPDATE,      /**< All GLO strings decoded
                                   (ephemeris + TOW + polarity update) */
} glo_decode_status_t;

void nav_msg_init_glo_with_cb(nav_msg_glo_t *n, me_gnss_signal_t mesid);
glo_decode_status_t glo_data_decoding(nav_msg_glo_t *n,
                                      me_gnss_signal_t mesid,
                                      const nav_bit_fifo_element_t *nav_bit);
decode_sync_flags_t get_data_sync_flags(const nav_msg_glo_t *n,
                                        me_gnss_signal_t mesid,
                                        glo_decode_status_t status);
void save_glo_eph(const nav_msg_glo_t *n, me_gnss_signal_t mesid);
bool glo_data_sync(nav_msg_glo_t *n,
                   me_gnss_signal_t mesid,
                   u8 tracking_channel,
                   glo_decode_status_t status);
void erase_cnav_data(gnss_signal_t target_sid, gnss_signal_t src_sid);

#endif /* #ifndef SWIFTNAV_DECODE_COMMON_H */
