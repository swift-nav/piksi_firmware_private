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

#ifndef SWIFTNAV_PLATFORM_SIGNAL_H
#define SWIFTNAV_PLATFORM_SIGNAL_H

extern struct usart_support_s SD1, SD3, SD6;
#define SD_FTDI  (&SD6)
#define SD_UARTA (&SD1)
#define SD_UARTB (&SD3)

#endif /* SWIFTNAV_PLATFORM_SIGNAL_H */
