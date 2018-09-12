/*
 * Copyright (C) 2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_SYSTEM_MONITOR_H
#define SWIFTNAV_SYSTEM_MONITOR_H

#include <swiftnav/common.h>

#define SPP_BASE_STATION_DIFFERENCE_WARNING_THRESHOLD 50.0

void system_monitor_setup(void);

/* Notification flags: system_monitor_thread will only clear the
 * hardware watchdog if watchdog_notify() is called with *each* of
 * these flags.  The idea is that every important thread will have at
 * least one flag listed here, corresponding to the important work
 * that that thread is doing.
 */
typedef enum {
  WD_NOTIFY_NAP_ISR,
  WD_NOTIFY_ACQ_MGMT,
  WD_NOTIFY_ME_CALC_PVT,
  WD_NOTIFY_STARLING,
  WD_NOTIFY_NUM_THREADS /* Maximum 32 of these! */
} watchdog_notify_t;

void watchdog_notify(watchdog_notify_t thread_id);
void frontend_error_notify_isr(void);
void frontend_error_notify_sys(void);

#endif
