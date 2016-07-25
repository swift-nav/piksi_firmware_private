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

#ifndef SWIFTNAV_USART_SUPPORT_H
#define SWIFTNAV_USART_SUPPORT_H

#include "remoteproc/rpmsg.h"

#define RPMSG_EPT_TO_SD(rpmsg_ept) ((void *)((int)(rpmsg_ept) + 1))
#define SD_TO_RPMSG_EPT(sd) ((rpmsg_endpoint_t)((int)(sd) - 1))

#define SD_FTDI       RPMSG_EPT_TO_SD(RPMSG_ENDPOINT_A)
#define SD_UARTA      NULL
#define SD_UARTB      NULL

#define SD_SETTINGS   RPMSG_EPT_TO_SD(RPMSG_ENDPOINT_B)
#define SD_FLASH      RPMSG_EPT_TO_SD(RPMSG_ENDPOINT_C)

#endif /* SWIFTNAV_USART_SUPPORT_H */
