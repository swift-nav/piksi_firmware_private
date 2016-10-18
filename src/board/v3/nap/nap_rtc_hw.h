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

#ifndef SWIFTNAV_NAP_RTC_REGS_H
#define SWIFTNAV_NAP_RTC_REGS_H

#include <stdint.h>

typedef struct {
  volatile uint32_t CONTROL;
  volatile uint32_t VERSION;
  volatile uint32_t RTC_COUNT;
  volatile uint32_t SAMPLE_COUNT;
} nap_rtc_t;

/* Bitfields */
#define NAP_RTC_CONTROL_VERSION_ADDR_Pos (0U)
#define NAP_RTC_CONTROL_VERSION_ADDR_Msk (0xFU << NAP_RTC_CONTROL_VERSION_ADDR_Pos)

#define NAP_RTC_COUNT_Pos (0U)
#define NAP_RTC_COUNT_Msk (0x7FFFU << NAP_RTC_COUNT_Pos)

/* Instances */
#define NAP_RTC ((nap_rtc_t *)0x43C20000)

#endif /* SWIFTNAV_NAP_RTC_REGS_H */
