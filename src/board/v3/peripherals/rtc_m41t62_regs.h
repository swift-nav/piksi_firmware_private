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

#ifndef SWIFTNAV_RTC_M41T62_REGS_H
#define SWIFTNAV_RTC_M41T62_REGS_H

#include <stdint.h>

typedef struct __attribute__((packed)) {
  /* REG 0 */
  unsigned int centisecond_ones   : 4;
  unsigned int centisecond_tens   : 4;
  /* REG 1 */
  unsigned int second_ones        : 4;
  unsigned int second_tens        : 3;
  unsigned int ST                 : 1;
  /* REG 2 */
  unsigned int minute_ones        : 4;
  unsigned int minute_tens        : 3;
  unsigned int OFIE               : 1;
  /* REG 3 */
  unsigned int hour_ones          : 4;
  unsigned int hour_tens          : 2;
  unsigned int reserved0          : 2;
  /* REG 4 */
  unsigned int wday_ones          : 3;
  unsigned int reserved1          : 1;
  unsigned int RS                 : 4;
  /* REG 5 */
  unsigned int mday_ones          : 4;
  unsigned int mday_tens          : 2;
  unsigned int reserved2          : 2;
  /* REG 6 */
  unsigned int month_ones         : 4;
  unsigned int month_tens         : 1;
  unsigned int reserved3          : 1;
  unsigned int century_ones       : 2;
  /* REG 7 */
  unsigned int year_ones          : 4;
  unsigned int year_tens          : 4;
} rtc_m41t62_clock_regs_t;

typedef struct __attribute__((packed)) {
  /* REG 8 */
  unsigned int calibration        : 5;
  unsigned int S                  : 1;
  unsigned int reserved0          : 1;
  unsigned int OUT                : 1;
} rtc_m41t62_calibration_reg_t;

typedef struct __attribute__((packed)) {
  /* REG 9 */
  unsigned int RB0                : 1;
  unsigned int RB1                : 1;
  unsigned int BMB                : 5;
  unsigned int RB2                : 1;
} rtc_m41t62_watchdog_reg_t;

typedef struct __attribute__((packed)) {
  /* REG 10 */
  unsigned int month_ones         : 4;
  unsigned int month_tens         : 1;
  unsigned int reserved0          : 1;
  unsigned int SQWE               : 1;
  unsigned int AFE                : 1;
  /* REG 11 */
  unsigned int mday_ones          : 4;
  unsigned int mday_tens          : 2;
  unsigned int RPT5               : 1;
  unsigned int RPT4               : 1;
  /* REG 12 */
  unsigned int hour_ones          : 4;
  unsigned int hour_tens          : 2;
  unsigned int reserved1          : 1;
  unsigned int RPT3               : 1;
  /* REG 13 */
  unsigned int minute_ones        : 4;
  unsigned int minute_tens        : 3;
  unsigned int RPT2               : 1;
  /* REG 14 */
  unsigned int second_ones        : 4;
  unsigned int second_tens        : 3;
  unsigned int RPT1               : 1;
} rtc_m41t62_alarm_regs_t;

typedef struct __attribute__((packed)) {
  /* REG 15 */
  unsigned int reserved0          : 2;
  unsigned int OF                 : 1;
  unsigned int reserved1          : 3;
  unsigned int AF                 : 1;
  unsigned int WDF                : 1;
} rtc_m41t62_flags_reg_t;

typedef struct __attribute__((packed)) {
  rtc_m41t62_clock_regs_t         clock_regs;
  rtc_m41t62_calibration_reg_t    calibration_reg;
  rtc_m41t62_watchdog_reg_t       watchdog_reg;
  rtc_m41t62_alarm_regs_t         alarm_regs;
  rtc_m41t62_flags_reg_t          flags_reg;
} rtc_m41t62_regs_t;

#define RTC_M41T62_REG_CLOCK_START  (0U)
#define RTC_M41T62_REG_CALIBRATION  (8U)
#define RTC_M41T62_REG_WATCHDOG     (9U)
#define RTC_M41T62_REG_ALARM_START  (10U)
#define RTC_M41T62_REG_FLAGS        (15U)

#define RTC_M41T62_RS_DISABLED  (0U)
#define RTC_M41T62_RS_32k       (1U)
#define RTC_M41T62_RS_8k        (2U)
#define RTC_M41T62_RS_4k        (3U)
#define RTC_M41T62_RS_2k        (4U)
#define RTC_M41T62_RS_1k        (5U)
#define RTC_M41T62_RS_512       (6U)
#define RTC_M41T62_RS_256       (7U)
#define RTC_M41T62_RS_128       (8U)
#define RTC_M41T62_RS_64        (9U)
#define RTC_M41T62_RS_32        (10U)
#define RTC_M41T62_RS_16        (11U)
#define RTC_M41T62_RS_8         (12U)
#define RTC_M41T62_RS_4         (13U)
#define RTC_M41T62_RS_2         (14U)
#define RTC_M41T62_RS_1         (15U)

#define RTC_M41T62_RPT_YEAR     (0x00U)
#define RTC_M41T62_RPT_MONTH    (0x10U)
#define RTC_M41T62_RPT_DAY      (0x18U)
#define RTC_M41T62_RPT_HOUR     (0x1CU)
#define RTC_M41T62_RPT_MINUTE   (0x1EU)
#define RTC_M41T62_RPT_SECOND   (0x1FU)

#define RTC_M41T62_RPT1(rpt)    (((rpt) >> 0U) & 0x1U)
#define RTC_M41T62_RPT2(rpt)    (((rpt) >> 1U) & 0x1U)
#define RTC_M41T62_RPT3(rpt)    (((rpt) >> 2U) & 0x1U)
#define RTC_M41T62_RPT4(rpt)    (((rpt) >> 3U) & 0x1U)
#define RTC_M41T62_RPT5(rpt)    (((rpt) >> 4U) & 0x1U)

#endif /* SWIFTNAV_RTC_M41T62_REGS_H */
