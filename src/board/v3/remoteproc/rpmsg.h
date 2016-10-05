/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_RPMSG_H
#define SWIFTNAV_RPMSG_H

#include <libswiftnav/common.h>

typedef enum {
  RPMSG_ENDPOINT_A,
  RPMSG_ENDPOINT_B,
  RPMSG_ENDPOINT_C,
  RPMSG_ENDPOINT__COUNT
} rpmsg_endpoint_t;

void rpmsg_setup(void);

u32 rpmsg_rx_fifo_length(rpmsg_endpoint_t rpmsg_endpoint);
u32 rpmsg_tx_fifo_space(rpmsg_endpoint_t rpmsg_endpoint);
u32 rpmsg_rx_fifo_read(rpmsg_endpoint_t rpmsg_endpoint, u8 *buffer, u32 length);
u32 rpmsg_tx_fifo_write(rpmsg_endpoint_t rpmsg_endpoint,
                        const u8 *buffer, u32 length);

#endif /* SWIFTNAV_RPMSG_H */
