/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "track.h"

#include <assert.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/memcpy_s.h>
#include <stdlib.h>
#include <string.h>

#include <ch.h>

#include "peripherals/random.h"
#include "signal_db/signal_db.h"

/** \addtogroup tracking
 * \{ */

#define COMPILER_BARRIER() asm volatile("" : : : "memory")

/* signal lock counter
 * A map of signal to an initially random number that increments each time that
 * signal begins being tracked.
 */
static u16 tracking_lock_counters[PLATFORM_ACQ_TRACK_COUNT];

/** Set up internal tracker data. */
void track_internal_setup(void) {
  for (u32 i = 0; i < PLATFORM_ACQ_TRACK_COUNT; i++) {
    tracking_lock_counters[i] = rand();
  }
}

/** Increment and return the tracking lock counter for the specified mesid.
 *
 * \param mesid ME identifier to use.
 */
u16 tracking_lock_counter_increment(const me_gnss_signal_t mesid) {
  return ++tracking_lock_counters[mesid_to_global_index(mesid)];
}

/** \} */
