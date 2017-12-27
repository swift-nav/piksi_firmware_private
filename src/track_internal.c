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
#include "signal.h"

/** \addtogroup tracking
 * \{ */

#define COMPILER_BARRIER() asm volatile("" : : : "memory")

static tracker_interface_list_element_t *tracker_interface_list = 0;

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

/** Return a pointer to the tracker interface list. */
tracker_interface_list_element_t **tracker_interface_list_ptr_get(void) {
  return &tracker_interface_list;
}

/** Initialize a nav_data_sync_t struct.
 *
 * \param sync          nav_data_sync_t struct to use.
 */
void nav_data_sync_init(nav_data_sync_t *sync) { sync->valid = false; }

/** Write pending sync data from the decoder thread.
 *
 * \note This function should only be called externally by the decoder thread.
 *
 * \param to_tracker   struct to sync.
 * \param from_decoder struct to sync with.
 *
 * \return true if data was stored successfully, false otherwise.
 */
bool nav_data_sync_set(nav_data_sync_t *to_tracker,
                       const nav_data_sync_t *from_decoder) {
  assert(to_tracker);
  assert(from_decoder);
  bool result = false;

  if (!to_tracker->valid) {
    COMPILER_BARRIER(); /* Prevent compiler reordering */
    *to_tracker = *from_decoder;
    COMPILER_BARRIER(); /* Prevent compiler reordering */
    to_tracker->valid = true;
    result = true;
  }

  return result;
}

/** Read pending sync data provided by the decoder thread.
 *
 * \note This function should only be called internally by the tracking thread.
 *
 * \param to_tracker   struct to sync.
 * \param sync         struct to sync with.
 *
 * \return true if outputs are valid, false otherwise.
 */
bool nav_data_sync_get(nav_data_sync_t *to_tracker,
                       nav_data_sync_t *from_decoder) {
  assert(to_tracker);
  assert(from_decoder);
  bool result = false;

  if (from_decoder->valid) {
    COMPILER_BARRIER(); /* Prevent compiler reordering */
    *to_tracker = *from_decoder;
    COMPILER_BARRIER(); /* Prevent compiler reordering */
    from_decoder->valid = false;
    result = true;
  }

  return result;
}

/** Compress a 32 bit integration value down to 8 bits.
 *
 * \param bit_integrate   Signed bit integration value.
 */
s8 nav_bit_quantize(s32 bit_integrate) {
  //  0 through  2^24 - 1 ->  0 = weakest positive bit
  // -1 through -2^24     -> -1 = weakest negative bit

  if (bit_integrate >= 0)
    return bit_integrate / (1 << 24);
  else
    return ((bit_integrate + 1) / (1 << 24)) - 1;
}

/** Increment and return the tracking lock counter for the specified mesid.
 *
 * \param mesid ME identifier to use.
 */
u16 tracking_lock_counter_increment(const me_gnss_signal_t mesid) {
  return ++tracking_lock_counters[mesid_to_global_index(mesid)];
}

/** \} */
