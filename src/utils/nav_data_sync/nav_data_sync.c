/*
 * Copyright (C) 2011-2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "nav_data_sync/nav_data_sync.h"

/** Initialize a nav_data_sync_t struct.
 *
 * \param sync          nav_data_sync_t struct to use.
 */
void nav_data_sync_init(nav_data_sync_t *sync) { sync->valid = false; }

/** Write pending sync data from the decoder thread.
 *
 * \note This function should only be called externally by the decoder thread.
 *
 * \param to_tracker   structure to sync.
 * \param from_decoder structure to sync with.
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
 * \param to_tracker   structure to sync.
 * \param from_decoder structure to sync with.
 *
 * \return true if outputs are valid, false otherwise.
 *
 * \note This function should only be called internally by the tracking thread.
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
