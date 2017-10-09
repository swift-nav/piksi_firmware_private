/*
 * Copyright (C) 2011-2017 Swift Navigation Inc.
 * Contact: Johannes Walter <johannes@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_NAP_REGS_H
#define SWIFTNAV_NAP_REGS_H

#include <libswiftnav/config.h>

/* Include register maps */
#include "swiftnap.h"

#define NAP_TRK_SPACING_CHIPS_Pos (6U)
#define NAP_TRK_SPACING_CHIPS_Msk (0x1U)

#define NAP_TRK_SPACING_SAMPLES_Pos (0U)
#define NAP_TRK_SPACING_SAMPLES_Msk (0x3FU)

typedef struct {
  const volatile u32 TRK_IRQS0;
  const volatile u32 TRK_IRQS1;
  const volatile u32 TRK_IRQ_ERRORS0;
  const volatile u32 TRK_IRQ_ERRORS1;
  swiftnap_tracking_rd_t TRK_CH_RD[NAP_NUM_TRACKING_CHANNELS];
} nap_dma_t;

/* Instances */
#define NAP ((swiftnap_t*)0x43C00000)
#define NAP_DMA ((nap_dma_t*)0x1ff00000)

#endif /* SWIFTNAV_NAP_REGS_H */
