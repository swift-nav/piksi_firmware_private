/*
 * Copyright (C) 2011-2017 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <assert.h>
#include <ch.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>

#include "board/nap/track_channel.h"
#include "manage.h"
#include "nap/nap_constants.h"
#include "ndb/ndb.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "settings/settings.h"
#include "signal.h"
#include "timing/timing.h"
#include "track.h"
#include "track/track_api.h"
#include "track/track_cn0.h"
#include "track/track_interface.h"
#include "track/track_params.h"
#include "track/track_sbp.h"
#include "track/track_sid_db.h"
#include "track/track_state.h"

/** \defgroup tracking Tracking
 * Track satellites via interrupt driven updates to SwiftNAP tracking channels.
 * Initialize SwiftNAP tracking channels. Run loop filters and update
 * channels' code / carrier frequencies each integration period. Update
 * tracking measurements each integration period.
 * \{ */

u16 max_pll_integration_time_ms = 20;

/** Calculate the future code phase after N samples.
 * Calculate the expected code phase in N samples time with carrier aiding.
 *
 * \param mesid        ME signal ID.
 * \param code_phase   Current code phase in chips.
 * \param carrier_freq Current carrier frequency (i.e. Doppler) in Hz used for
 *                     carrier aiding.
 * \param n_samples    N, the number of samples to propagate for.
 *
 * \return The propagated code phase in chips.
 */
double propagate_code_phase(const me_gnss_signal_t mesid,
                            double code_phase,
                            double carrier_freq,
                            u32 n_samples) {
  /* Calculate the code phase rate with carrier aiding. */
  double code_phase_rate = (1.0 + carrier_freq / mesid_to_carr_freq(mesid)) *
                           code_to_chip_rate(mesid.code);
  code_phase += n_samples * code_phase_rate / NAP_FRONTEND_SAMPLE_RATE_Hz;
  u32 cp_int = floor(code_phase);
  code_phase -= cp_int - (cp_int % code_to_chip_count(mesid.code));
  return code_phase;
}

/** Read the next pending nav bit for a tracker channel.
 *
 * \note This function should should be called from the same thread as
 * tracking_channel_time_sync().
 *
 * \param id       ID of the tracker channel to read from.
 * \param nav_bit  Struct containing nav_bit data.
 *
 * \return true if valid nav_bit is available, false otherwise.
 */
bool tracking_channel_nav_bit_get(tracker_id_t id,
                                  nav_bit_fifo_element_t *nav_bit) {
  tracker_t *tracker_channel = tracker_get(id);

  nav_bit_fifo_element_t element;
  if (nav_bit_fifo_read(&tracker_channel->nav_bit_fifo, &element)) {
    *nav_bit = element;
    return true;
  }
  return false;
}

/** Initializes the data structure used to sync data between decoder and tracker
 *
 * \param nav_data_sync struct used for sync
 */
void tracking_channel_data_sync_init(nav_data_sync_t *nav_data_sync) {
  memset(nav_data_sync, 0, sizeof(*nav_data_sync));
  nav_data_sync->glo_orbit_slot = GLO_ORBIT_SLOT_UNKNOWN;
  nav_data_sync->glo_health = GLO_SV_UNHEALTHY;
  nav_data_sync->sync_flags = SYNC_ALL;
}

/** Propagate decoded time of week, bit polarity and optional glo orbit slot
 *  back to a tracker channel.
 *
 * \note This function should be called from the same thread as
 * tracking_channel_nav_bit_get().
 * \note It is assumed that the specified data is synchronized with the most
 * recent nav bit read from the FIFO using tracking_channel_nav_bit_get().
 *
 * \param id           ID of the tracker channel to synchronize.
 * \param from_decoder struct to sync tracker with.
 */
static void data_sync(tracker_id_t id, nav_data_sync_t *from_decoder) {
  assert(from_decoder);

  tracker_t *tracker_channel = tracker_get(id);
  from_decoder->read_index = tracker_channel->nav_bit_fifo.read_index;
  if (!nav_data_sync_set(&tracker_channel->nav_data_sync, from_decoder)) {
    log_warn_mesid(tracker_channel->mesid, "Data sync failed");
  }
}

/** Propagate decoded GPS time of week and bit polarity back to a tracker
 * channel.
 *
 * \note This function should be called from the same thread as
 * tracking_channel_nav_bit_get().
 * \note It is assumed that the specified data is synchronized with the most
 * recent nav bit read from the FIFO using tracking_channel_nav_bit_get().
 *
 * \param id           ID of the GPS tracker channel to synchronize.
 * \param from_decoder struct to sync tracker with.
 */
void tracking_channel_data_sync(tracker_id_t id,
                                nav_data_sync_t *from_decoder) {
  assert(from_decoder);

  if ((from_decoder->TOW_ms < 0) ||
      (BIT_POLARITY_UNKNOWN == from_decoder->bit_polarity)) {
    return;
  }
  data_sync(id, from_decoder);
}

/** Propagate decoded GLO time of week, bit polarity and glo orbit slot
 *  back to a tracker channel.
 *
 * \note This function should be called from the same thread as
 * tracking_channel_nav_bit_get().
 * \note It is assumed that the specified data is synchronized with the most
 * recent nav bit read from the FIFO using tracking_channel_nav_bit_get().
 *
 * \param id           ID of the GLO tracker channel to synchronize.
 * \param from_decoder struct to sync tracker with.
 */
void tracking_channel_glo_data_sync(tracker_id_t id,
                                    nav_data_sync_t *from_decoder) {
  assert(from_decoder);

  data_sync(id, from_decoder);
}

/** Return the unsigned difference between update_count and *val for a
 * tracker channel.
 *
 * \note This function allows some margin to avoid glitches in case values
 * are not read atomically from the tracking channel data.
 *
 * \param tracker_channel   Tracker channel to use.
 * \param val               Pointer to the value to be subtracted
 *                          from update_count.
 *
 * \return The unsigned difference between update_count and *val.
 */
update_count_t update_count_diff(const tracker_t *tracker_channel,
                                 const update_count_t *val) {
  update_count_t result =
      (update_count_t)(tracker_channel->update_count - *val);
  COMPILER_BARRIER(); /* Prevent compiler reordering */
  /* Allow some margin in case values were not read atomically.
   * Treat a difference of [-10000, 0) as zero. */
  if (result > (update_count_t)(UINT32_MAX - 10000))
    return 0;
  else
    return result;
}

/** \} */
