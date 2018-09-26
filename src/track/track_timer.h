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

#ifndef SWIFTNAV_TRACK_TIMER_H
#define SWIFTNAV_TRACK_TIMER_H

#include <assert.h>
#include <stdbool.h>
#include <string.h>

#include <swiftnav/common.h>

typedef struct {
  s64 deadline_ms; /** -1 - no deadline */
  u64 armed_at_ms; /** the timestamp of tracker_timer_arm() call */
} tracker_timer_t;

extern u64 tracker_time_us;

/* Updates current global tracker time in microseconds */
static inline void tracker_time_set(u64 us) { tracker_time_us = us; }

/* Returns current global tracker time in milliseconds */
static inline u64 tracker_time_now_ms(void) {
  return (tracker_time_us + 999) / 1000;
}

/** Init tracker timer */
static inline void tracker_timer_init(tracker_timer_t *tm) {
  assert(tm);
  memset(tm, 0, sizeof(*tm));
}

/**
 * Arms tracker timer
 * The deadline at which the timer is expired in ms is defined by deadline_ms
 * If deadline_ms is -1 the timer never expires.
 * Use tracker_time_now_ms() + A_TIMEOUT_MS to define a deadline
 * in A_TIMEOUT_MS from now.
 */
static inline void tracker_timer_arm(tracker_timer_t *tm, s64 deadline_ms) {
  assert(tm);
  tm->deadline_ms = deadline_ms;
  tm->armed_at_ms = tracker_time_now_ms();
}

/** Return the time elapsed since tracker_timer_arm() call for this timer */
static inline u64 tracker_timer_ms(tracker_timer_t *tm) {
  assert(tm);
  if (tm->deadline_ms) {
    return tracker_time_now_ms() - tm->armed_at_ms;
  }
  return 0;
}

/** Checks fi time is expired. If timer was not armed, then it is expired. */
static inline bool tracker_timer_expired(tracker_timer_t *tm) {
  assert(tm);
  if (tm->deadline_ms < 0) {
    return false;
  }
  return (tracker_time_now_ms() >= (u64)tm->deadline_ms);
}

#endif /* #ifndef SWIFTNAV_TRACK_TIMER_H */
