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

#include "usart_support.h"
#include "peripherals/usart.h"
#include "remoteproc/rpmsg.h"

void usart_support_init(void)
{
}

void usart_support_set_parameters(void *sd, u32 baud)
{
  (void)sd;
  (void)baud;
}

void usart_support_disable(void *sd)
{
  (void)sd;
}

u32 usart_support_n_read(void *sd)
{
  return rpmsg_rx_fifo_length(SD_TO_RPMSG_EPT(sd));
}

u32 usart_support_tx_n_free(void *sd)
{
  return rpmsg_tx_fifo_space(SD_TO_RPMSG_EPT(sd));
}

u32 usart_support_read_timeout(void *sd, u8 data[], u32 len, u32 timeout)
{
  (void)timeout;
  return rpmsg_rx_fifo_read(SD_TO_RPMSG_EPT(sd), data, len);
}

u32 usart_support_write(void *sd, const u8 data[], u32 len)
{
  return rpmsg_tx_fifo_write(SD_TO_RPMSG_EPT(sd), data, len);
}

