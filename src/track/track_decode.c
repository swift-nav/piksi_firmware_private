/*
 * Copyright (C) 2011-2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "track_decode.h"

#include <string.h>

#include "track_state.h"

/** Read the next pending nav bit for a tracker channel.
 *
 * \param id       ID of the tracker channel to read from.
 * \param nav_bit  Struct containing nav_bit data.
 *
 * \return true if valid nav_bit is available, false otherwise.
 */
bool tracker_nav_bit_received(u8 id, nav_bit_t *nav_bit) {
  tracker_t *tracker = tracker_get(id);
  return nav_bit_fifo_read(&tracker->nav_bit_fifo, nav_bit);
}

/** Initializes the data structure used to sync data between decoder and tracker
 *
 * \param nav_data_sync struct used for sync
 */
void tracker_data_sync_init(nav_data_sync_t *nav_data_sync) {
  memset(nav_data_sync, 0, sizeof(*nav_data_sync));
  nav_data_sync->health = SV_UNHEALTHY;
  nav_data_sync->sync_flags = SYNC_POL | SYNC_TOW | SYNC_EPH;
}

/** Propagate decoded time of week, bit polarity and optional glo orbit slot
 *  back to a tracker channel.
 *
 * \note This function should be called from the same thread as
 * tracker_nav_bit_received().
 * \note It is assumed that the specified data is synchronized with the most
 * recent nav bit read from the FIFO using tracker_nav_bit_received().
 *
 * \param id           ID of the tracker channel to synchronize.
 * \param from_decoder struct to sync tracker with.
 */
static void data_sync(u8 id, nav_data_sync_t *from_decoder) {
  assert(from_decoder);

  tracker_t *tracker = tracker_get(id);
  from_decoder->read_index = tracker->nav_bit_fifo.read_index;
  if (!nav_data_sync_set(&tracker->nav_data_sync, from_decoder)) {
    log_warn_mesid(tracker->mesid, "Data sync failed");
  }
}

/** Propagate decoded information back to a tracker channel.
 *
 * \note This function should be called from the same thread as
 * tracker_nav_bit_received().
 * \note It is assumed that the specified data is synchronized with the most
 * recent nav bit read from the FIFO using tracker_nav_bit_received().
 *
 * \param id           ID of the tracker channel to synchronize.
 * \param from_decoder struct to sync tracker with.
 */
void tracker_data_sync(u8 id, nav_data_sync_t *from_decoder) {
  assert(from_decoder);

  if (SYNC_NONE == from_decoder->sync_flags) {
    /* Nothing to sync */
    return;
  }

  tracker_t *tracker = tracker_get(id);
  if (!IS_GPS(tracker->mesid) && !IS_QZSS(tracker->mesid)) {
    data_sync(id, from_decoder);
    return;
  }

  if ((SYNC_POL != from_decoder->sync_flags) &&
      ((from_decoder->TOW_ms < 0) ||
       (BIT_POLARITY_UNKNOWN == from_decoder->bit_polarity))) {
    return;
  }
  data_sync(id, from_decoder);
}
