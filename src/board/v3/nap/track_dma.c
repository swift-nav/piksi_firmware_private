/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Michael Wurm <mwurm@swift-nav.com>
 *          Johannes Walter <johannes@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "track_dma.h"
#include "pl330_dmac.h"

static pl330_transfer_t xfer = {0};

void track_dma_clear_irq() { pl330_irq_clear(PL330_CHANNEL); }

void track_dma_start(void) { pl330_trigger(&xfer); }

void track_dma_init(irq_handler_t isr, u32 src, u32 dst, u32 len) {
  xfer.src_addr = src;
  xfer.dst_addr = dst;
  xfer.bytes = len;
  xfer.channel = PL330_CHANNEL;

  pl330_init(&xfer);

  /* Activate DMA peripheral clock */
  *PL330_APER_CLK_CTRL |= 1;

  /* Set interrupt target to NO_CPU */
  *PL330_TARGET_REG &= ~(PL330_GIC_TARGET_BOTH_CPU << PL330_TARGET_ID);
  /* Set interrupt target to CPU1 (Firmware) */
  *PL330_TARGET_REG |= (PL330_GIC_TARGET_CPU1_FW << PL330_TARGET_ID);

  /* Enable interrupt */
  pl330_irq_enable(PL330_CHANNEL);
  *PL330_GIC_DIST_EN |= 1;

  gic_handler_register(PL330_IRQ_ID, isr, NULL);
  gic_irq_sensitivity_set(PL330_IRQ_ID, IRQ_SENSITIVITY_EDGE);
  gic_irq_priority_set(PL330_IRQ_ID, PL330_IRQ_PRIORITY);
  gic_irq_enable(PL330_IRQ_ID);
}
