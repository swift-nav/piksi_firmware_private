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

#ifndef SWIFTNAV_TRACK_DMA_H
#define SWIFTNAV_TRACK_DMA_H

#include <libswiftnav/common.h>

#include "board.h"

#define PL330_SLCR_BASE (0xF8000000U)
#define PL330_MPCORE_BASE (0xF8F00000U)

#define PL330_APER_CLK_CTRL (volatile u32*)(PL330_SLCR_BASE + 0x0000012CU)
#define PL330_GIC_DIST_EN (volatile u32*)(PL330_MPCORE_BASE + 0x00001000U)

#define PL330_GIC_SPI_TARGET_REG11 \
  (volatile u32*)(PL330_MPCORE_BASE + 0x0000182CU)
#define PL330_GIC_SPI_TARGET_REG12 \
  (volatile u32*)(PL330_MPCORE_BASE + 0x00001830U)
#define PL330_GIC_SPI_TARGET_REG18 \
  (volatile u32*)(PL330_MPCORE_BASE + 0x00001848U)

#define PL330_GIC_SPI_TARGET_ID46 (16)  // REG11
#define PL330_GIC_SPI_TARGET_ID47 (24)  // REG11
#define PL330_GIC_SPI_TARGET_ID48 (0)   // REG12
#define PL330_GIC_SPI_TARGET_ID49 (8)   // REG12
#define PL330_GIC_SPI_TARGET_ID72 (0)   // REG18
#define PL330_GIC_SPI_TARGET_ID73 (8)   // REG18
#define PL330_GIC_SPI_TARGET_ID74 (16)  // REG18
#define PL330_GIC_SPI_TARGET_ID75 (24)  // REG18

#define PL330_GIC_TARGET_NO_CPU (0)
#define PL330_GIC_TARGET_CPU0_LINUX (1)
#define PL330_GIC_TARGET_CPU1_FW (2)
#define PL330_GIC_TARGET_BOTH_CPU (3)

// Use DMAC channel 7
#define PL330_CHANNEL (0x7U)
#define PL330_TARGET_ID PL330_GIC_SPI_TARGET_ID75
#define PL330_TARGET_REG PL330_GIC_SPI_TARGET_REG18
#define PL330_IRQ_ID IRQ_ID_DMAC_7
#define PL330_IRQ_PRIORITY 4

void track_dma_init(irq_handler_t isr, u32 src, u32 dst, u32 len);
void track_dma_clear_irq(void);
void track_dma_start(void);

#endif /* SWIFTNAV_TRACK_DMA_H */
