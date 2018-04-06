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

struct sbas_watchdog {
  u16 msg_mask;
  u16 symbol_cnt;
  bool triggered;
};

void sbas_watchdog_init(struct sbas_watchdog *wdog);
void sbas_watchdog_hnd_message(struct sbas_watchdog *wdog);
void sbas_watchdog_hnd_symbol(struct sbas_watchdog *wdog);
bool sbas_watchdog_is_triggered(const struct sbas_watchdog *wdog);

#endif /* SBAS_WATCHDOG_H */
