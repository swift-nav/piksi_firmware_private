/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Dmitry Tatarinov <dmitry.tatarinov@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#include <swiftnav/gnss_time.h>

#include "ch.h"

u32 stubs_now_ms = 0;

void chSysLock(){};

void chSysUnlock(){};

systime_t chVTGetSystemTimeX() { return 0; };

systime_t chThdSleep(systime_t time) {
  (void)time;
  return 1;
}

systime_t chThdSleepS(systime_t time) {
  (void)time;
  return 1;
}

systime_t chThdSleepMilliseconds(systime_t time) {
  stubs_now_ms += (u32)time;
  return 1;
}

void chMtxLock(mutex_t *mp) { (void)mp; }

void chMtxUnlock(mutex_t *mp) { (void)mp; }
