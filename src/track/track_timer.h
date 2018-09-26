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

#include <stdbool.h>
#include <swiftnav/common.h>

typedef struct {
  s64 deadline_ms; /** -1 - no deadline */
  u64 armed_at_ms; /** the timestamp of tracker_timer_arm() call */
} tracker_timer_t;

void tracker_time_set(u64 us);
u64 tracker_time_now_ms(void);

void tracker_timer_init(tracker_timer_t *tm);
void tracker_timer_arm(tracker_timer_t *tm, s64 deadline_ms);
bool tracker_timer_expired(tracker_timer_t *tm);
u64 tracker_timer_ms(tracker_timer_t *tm);

#endif /* #ifndef SWIFTNAV_TRACK_TIMER_H */
