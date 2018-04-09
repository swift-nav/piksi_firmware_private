/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact:Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#ifndef SBAS_WATCHDOG_H
#define SBAS_WATCHDOG_H

#include <libswiftnav/common.h>

typedef struct {
  u16 msg_mask;
  u16 symbol_cnt;
  bool triggered;
} sbas_watchdog_t;

void sbas_watchdog_init(sbas_watchdog_t *wdog);
void sbas_watchdog_hnd_message(sbas_watchdog_t *wdog);
void sbas_watchdog_hnd_symbol(sbas_watchdog_t *wdog);
bool sbas_watchdog_is_triggered(const sbas_watchdog_t *wdog);

#endif /* SBAS_WATCHDOG_H */
