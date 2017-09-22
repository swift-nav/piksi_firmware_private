/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Michael Wurm <mwurm@swiftnav.com>
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

#include <libswiftnav/logging.h>

#define PL330_BASE_S                 (0xF8003000U)
#define PL330_BASE_NS                (0xF8004000U)

#define PL330_SLCR_BASE              (0xF8000000U)
#define PL330_DEVCFG_BASE            (0xF8007000U)
#define PL330_DMAC0_BASE             (PL330_BASE_S)

#define PL330_APER_CLK_CTRL          (volatile u32*)(PL330_SLCR_BASE + 0x0000012CU)
#define PL330_DMA_CPU_2XCLKACT_Pos   (0x0U)

#define PL330_DMAC_RST_CTRL          (volatile u32*)(PL330_SLCR_BASE + 0x0000020CU)
#define PL330_DMAC_RST_Pos           (0x0U)

#define PL330_INTEN                  (volatile u32*)(PL330_DMAC0_BASE + 0x00000020U)
#define PL330_INTEN_0                (0x0U)

#define PL330_INT_EVENT_RIS          (volatile u32*)(PL330_DMAC0_BASE + 0x00000024U)
#define PL330_INT_EVENT_0            (0x0U)

#define PL330_INT_STATUS             (volatile u32*)(PL330_DMAC0_BASE + 0x00000028U)
#define PL330_INT_STATUS_0           (0x0U)

#define PL330_INTCLR                 (volatile u32*)(PL330_DMAC0_BASE + 0x0000002CU)
#define PL330_INTCLR_0               (0x0U)

#define PL330_CHANNEL_TO_USE         (0x1U)                     // channel nr 0-7
#define PL330_IRQ_ID_TO_USE          IRQ_ID_DMAC_1              // must correspond to channel nr 0-7
#define PL330_TARGET_ID_TO_USE       PL330_GIC_SPI_TARGET_ID47  // must correspond to channel nr
#define PL330_TARGET_REG_TO_USE      PL330_GIC_SPI_TARGET_REG11 // must correspond to target_id
                                                                // --> see define of target_id
#define PL330_MPCORE_BASE            (0xF8F00000U)
#define PL330_GIC_DIST_EN            (volatile u32*)(PL330_MPCORE_BASE + 0x00001000U)
#define PL330_GIC_CONTROL            (volatile u32*)(PL330_MPCORE_BASE + 0x00000100U)
#define PL330_GIC_ENABLE_SET0        (volatile u32*)(PL330_MPCORE_BASE + 0x00001100U)
#define PL330_GIC_ENABLE_SET1        (volatile u32*)(PL330_MPCORE_BASE + 0x00001104U)
#define PL330_GIC_ENABLE_SET2        (volatile u32*)(PL330_MPCORE_BASE + 0x00001108U)
#define PL330_GIC_EN_INT_Pos         (0x0U)
#define PL330_ICDICFR2               (volatile u32*)(PL330_MPCORE_BASE + 0x00001C08U)
#define PL330_GIC_DISABLE            (volatile u32*)(PL330_MPCORE_BASE + 0x00001180U)
#define PL330_GIC_SPI_TARGET_REG11   (volatile u32*)(PL330_MPCORE_BASE + 0x0000182CU)
#define PL330_GIC_SPI_TARGET_REG12   (volatile u32*)(PL330_MPCORE_BASE + 0x00001830U)
#define PL330_GIC_SPI_TARGET_REG18   (volatile u32*)(PL330_MPCORE_BASE + 0x00001848U)
#define PL330_GIC_SPI_TARGET_ID46    (16) // reg11
#define PL330_GIC_SPI_TARGET_ID47    (24) // reg11
#define PL330_GIC_SPI_TARGET_ID48    (0)  // reg12
#define PL330_GIC_SPI_TARGET_ID49    (8)  // reg12
#define PL330_GIC_SPI_TARGET_ID72    (0)  // reg48
#define PL330_GIC_SPI_TARGET_ID73    (8)  // reg48
#define PL330_GIC_SPI_TARGET_ID74    (16) // reg48
#define PL330_GIC_SPI_TARGET_ID75    (24) // reg48
#define PL330_GIC_TARGET_NO_CPU      (0)
#define PL330_GIC_TARGET_CPU0_LINUX  (1)  // Linux can't handle the IRQ - results in crash (Oops Bug 17)
#define PL330_GIC_TARGET_CPU1_FW     (2)
#define PL330_GIC_TARGET_BOTH_CPU    (3)  // Linux can't handle the IRQ - results in crash (Oops Bug 17)

#define PL330_DBGSTATUS              (volatile u32*)(PL330_DMAC0_BASE + 0x00000D00U)
#define PL330_DBGSTATUS_BUSY_Pos     (0x0U)
#define PL330_DSR                    (volatile u32*)(PL330_DMAC0_BASE)
#define PL330_DSR_DMA_STATUS_Mask    (0xF)
#define PL330_DSR_DNS_Mask           (0x200)
#define PL330_FSRD                   (volatile u32*)(PL330_DMAC0_BASE + 0x00000030U)
#define PL330_FSRD_FS_MGR_Mask       (0x1U)

#define PL330_DBGCMD                 (volatile u32*)(PL330_DMAC0_BASE + 0x00000D04U)
#define PL330_DBGINST0               (volatile u32*)(PL330_DMAC0_BASE + 0x00000D08U)
#define PL330_DBGINST1               (volatile u32*)(PL330_DMAC0_BASE + 0x00000D0CU)

void track_dma_init(void);
void track_dma_start(u32* const s_addr, u32* const d_addr);
void track_dma_set_irq_number(u32 nr);
void pl330_dma_irq_handler(void *context);

#endif /* SWIFTNAV_TRACK_DMA_H */
