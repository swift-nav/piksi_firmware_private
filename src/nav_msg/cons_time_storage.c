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

#include "cons_time_storage.h"

#include <math.h>

#include <ch.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/signal.h>

#include "utils/timing/timing.h"

static MUTEX_DECL(cons_time_mutex);

typedef struct {
  bool params_valid;
  cons_time_params_t params;
} cons_time_storage_t;

static cons_time_storage_t cons_time_storage[CONSTELLATION_COUNT];

/** Stores constellation time offset parameters
 *
 * \param sid originating signal ID
 * \param params pointer to parameters to be stored
 */
void store_cons_time_params(const gnss_signal_t sid,
                            const cons_time_params_t *params) {
  if (!sid_valid(sid)) {
    return;
  }
  bool params_updated = false;
  u8 cons_idx = (u8)code_to_constellation(sid.code);
  gps_time_t now = get_current_time();

  chMtxLock(&cons_time_mutex);
  cons_time_storage_t *storage_cell = &(cons_time_storage[cons_idx]);
  /* store the new parameters if the old ones are invalid or further away from
   * current time */
  if (!storage_cell->params_valid ||
      fabs(gpsdifftime(&params->t, &now)) <
          fabs(gpsdifftime(&storage_cell->params.t, &now))) {
    storage_cell->params = *params;
    storage_cell->params_valid = true;
    params_updated = true;
  }
  chMtxUnlock(&cons_time_mutex);

  if (params_updated) {
    log_info_sid(
        sid,
        "Constellation time offset saved, t=(%d,%.0f), a0=%.3g, a1=%.3g, "
        "current offset=%.1f ns.",
        params->t.wn,
        params->t.tow,
        params->a0,
        params->a1,
        (params->a1 * gpsdifftime(&now, &params->t) + params->a0) * SECS_NS);
  }
}

/** Retrieves constellation time offset parameters
 *
 * \param sid Signal ID
 * \param params pointer to parameters to be retrieved
 *
 * \returns true if requested parameters were found and copied to provided
 *               buffer, false otherwise.
 */
bool get_cons_time_params(const gnss_signal_t sid, cons_time_params_t *params) {
  bool res = false;

  if (sid_valid(sid)) {
    u8 cons_idx = (u8)code_to_constellation(sid.code);
    chMtxLock(&cons_time_mutex);
    cons_time_storage_t *storage_cell = &(cons_time_storage[cons_idx]);
    if (storage_cell->params_valid) {
      *params = storage_cell->params;
      res = true;
    }
    chMtxUnlock(&cons_time_mutex);
  }

  return res;
}
