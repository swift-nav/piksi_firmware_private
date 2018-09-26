/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "track_timer.h"
#include <assert.h>
#include <string.h>

static u64 tracker_time_us = 0;

void tracker_time_set(u64 us) { tracker_time_us = us; }

u64 tracker_time_now_ms(void) { return (tracker_time_us + 999) / 1000; }

void tracker_timer_init(tracker_timer_t *tm) {
  assert(tm);
  memset(tm, 0, sizeof(*tm));
}

void tracker_timer_arm(tracker_timer_t *tm, s64 deadline_ms) {
  assert(tm);
  tm->deadline_ms = deadline_ms;
  tm->armed_at_ms = tracker_time_now_ms();
}

u64 tracker_timer_ms(tracker_timer_t *tm) {
  assert(tm);
  if (tm->deadline_ms) {
    return tracker_time_now_ms() - tm->armed_at_ms;
  }
  return 0;
}

bool tracker_timer_expired(tracker_timer_t *tm) {
  assert(tm);
  if (tm->deadline_ms < 0) {
    return false;
  }
  return (tracker_time_now_ms() >= (u64)tm->deadline_ms);
}
