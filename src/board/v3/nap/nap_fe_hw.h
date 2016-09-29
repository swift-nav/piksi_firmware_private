/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Johannes Walter <johannes@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_NAP_FE_REGS_H
#define SWIFTNAV_NAP_FE_REGS_H

#include <stdint.h>

typedef struct {
  volatile uint32_t STATUS;
  volatile uint32_t CONTROL;
  volatile uint32_t VERSION;
  volatile uint32_t IRQ;
  volatile uint32_t IRQ_ERROR;
  volatile uint32_t ACQ_STATUS;
  volatile uint32_t TRK_STATUS;
  volatile uint32_t PINC[4];
} nap_fe_t;

/* Bitfields */
#define NAP_FE_STATUS_CLOCK_MISSING_Pos (0U)
#define NAP_FE_STATUS_CLOCK_MISSING_Msk (0x1U << NAP_FE_STATUS_CLOCK_MISSING_Pos)

#define NAP_FE_STATUS_FRONTEND_CH_Pos (1U)
#define NAP_FE_STATUS_FRONTEND_CH_Msk (0xFU << NAP_FE_STATUS_FRONTEND_CH_Pos)

#define NAP_FE_CONTROL_VERSION_ADDR_Pos (0U)
#define NAP_FE_CONTROL_VERSION_ADDR_Msk (0xFU << NAP_FE_CONTROL_VERSION_ADDR_Pos)

#define NAP_FE_CONTROL_ENABLE_RF1_Pos (4U)
#define NAP_FE_CONTROL_ENABLE_RF1_Msk (0x1U << NAP_FE_CONTROL_ENABLE_RF1_Pos)

#define NAP_FE_CONTROL_ENABLE_RF2_Pos (5U)
#define NAP_FE_CONTROL_ENABLE_RF2_Msk (0x1U << NAP_FE_CONTROL_ENABLE_RF2_Pos)

#define NAP_FE_CONTROL_ENABLE_RF3_Pos (6U)
#define NAP_FE_CONTROL_ENABLE_RF3_Msk (0x1U << NAP_FE_CONTROL_ENABLE_RF3_Pos)

#define NAP_FE_CONTROL_ENABLE_RF4_Pos (7U)
#define NAP_FE_CONTROL_ENABLE_RF4_Msk (0x1U << NAP_FE_CONTROL_ENABLE_RF4_Pos)

/* Instances */
#define NAP_FE ((nap_fe_t *)0x43C10000)

#endif /* SWIFTNAV_NAP_FE_REGS_H */
