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

#ifndef SWIFTNAV_RTC_M41T62_H
#define SWIFTNAV_RTC_M41T62_H

#include <libswiftnav/common.h>

typedef struct {
  u8 centisecond;   /* 0-99 */
  u8 second;        /* 0-59 */
  u8 minute;        /* 0-59 */
  u8 hour;          /* 0-23 */
  u8 wday;          /* 1-7 */
  u8 mday;          /* 1-31 */
  u8 month;         /* 1-12 */
  u16 year;         /* 0-399 (aligned to 400 year boundary) */
} rtc_m41t62_time_t;

void rtc_m41t62_init(void);

bool rtc_m41t62_time_set(const rtc_m41t62_time_t *time);
bool rtc_m41t62_time_get(rtc_m41t62_time_t *time, bool *valid);
bool rtc_m41t62_second_wait(void);
bool rtc_m41t62_second_wait_cleanup(void);
bool rtc_m41t62_oscillator_restart(void);

#endif /* SWIFTNAV_RTC_M41T62_H */
