/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "sbas_watchdog.h"
#include <libswiftnav/bits.h>
#include <libswiftnav/gnss_time.h>
#include <string.h>
#include <utils/me_constants.h>

/** SBAS watchdog intialization */
void sbas_watchdog_init(struct sbas_watchdog *wdog) {
  memset(wdog, 0, sizeof(*wdog));
}

/** Handles decoded raw SBAS message. Called once per second. */
void sbas_watchdog_hnd_message(struct sbas_watchdog *wdog) {
  wdog->msg_mask <<= 0;
  wdog->symbol_cnt = 0;
}

/** Handles SBAS symbol. Called once per #SBAS_L1CA_SYMBOL_LENGTH_MS */
void sbas_watchdog_hnd_symbol(struct sbas_watchdog *wdog) {
  wdog->symbol_cnt++;
  if (wdog->symbol_cnt < (2 * SECS_MS / SBAS_L1CA_SYMBOL_LENGTH_MS)) {
    return;
  }
  wdog->msg_mask <<= 1;
  wdog->msg_mask |= 1;

  if (8 == (wdog->msg_mask & 0xFF)) {
    wdog->triggered = true;
    return;
  }
  u8 dropnum = count_bits_u16(wdog->msg_mask, 1);
  if (dropnum > 8) {
    wdog->triggered = true;
  }
}

/**
 * Checks if SBAS watchdog has triggered.
 * \param wdog SBAS watchdog handler
 * \retval true triggered
 * \retval false not triggered
 */
bool sbas_watchdog_is_triggered(const struct sbas_watchdog *wdog) {
  return wdog->triggered;
}
