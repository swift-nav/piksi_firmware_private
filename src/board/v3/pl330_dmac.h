/*
 * Copyright (C) 2011-2017 Swift Navigation Inc.
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

#ifndef SWIFTNAV_PL330_DMAC_H
#define SWIFTNAV_PL330_DMAC_H

#include <libswiftnav/common.h>

typedef struct {
  u8 channel;
  u32 src_addr;
  u32 dst_addr;
  u32 bytes;
  u8 prog[128];
} pl330_transfer_t;

s32 pl330_init(pl330_transfer_t* xfer);
s32 pl330_start(pl330_transfer_t* xfer);
s32 pl330_trigger(pl330_transfer_t* xfer);

void pl330_irq_enable(u8 channel);
void pl330_irq_disable(u8 channel);
void pl330_irq_clear(u8 channel);

#endif /* SWIFTNAV_PL330_DMAC_H */
