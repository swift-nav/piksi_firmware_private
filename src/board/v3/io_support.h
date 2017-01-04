/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Gareth McMullin <gareth@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_IO_SUPPORT_H
#define SWIFTNAV_IO_SUPPORT_H

#include "remoteproc/rpmsg.h"

#define RPMSG_EPT_TO_SD(rpmsg_ept) ((void *)((int)(rpmsg_ept) + 1))
#define SD_TO_RPMSG_EPT(sd) ((rpmsg_endpoint_t)((int)(sd) - 1))

#define SD_SBP        RPMSG_EPT_TO_SD(RPMSG_ENDPOINT_A)
#define SD_UARTA      NULL
#define SD_UARTB      NULL

#define SD_SETTINGS   RPMSG_EPT_TO_SD(RPMSG_ENDPOINT_B)
#define SD_FLASH      RPMSG_EPT_TO_SD(RPMSG_ENDPOINT_C)

void io_support_init(void);
void io_support_set_parameters(void *sd, u32 baud);
void io_support_disable(void *sd);
u32 io_support_n_read(void *sd);
u32 io_support_tx_n_free(void *sd);
u32 io_support_read_timeout(void *sd, u8 data[], u32 len, u32 timeout);
u32 io_support_write(void *sd, const u8 data[], u32 len);

#endif /* SWIFTNAV_IO_SUPPORT_H */
