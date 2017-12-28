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
#include "nav_meas_calc.h"
#include "ndb/ndb.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "settings.h"
#include "signal.h"
#include "timing/timing.h"
#include "track.h"
#include "track/track_api.h"
#include "track/track_cn0.h"
#include "track/track_interface.h"
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

/**
 * Atomically loads tracking channel public informational block.
 *
 * The channel locks public informational block and loads data from it into
 * output parameters.
 *
 * \param[in]  id           Tracking channel identifier.
 * \param[out] info         Optional destination for generic information.
 * \param[out] time_info    Optional destination for timing information.
 * \param[out] freq_info    Optional destination for frequency and phase
 *                          information.
 * \param[out] ctrl_params  Optional destination for loop controller
 * information.
 * \param[out] misc_params  Optional destination for misc information.
 * \param[in] reset_stats   Reset channel statistics
 *
 * \return None
 *
 * \sa tracking_channel_update_values
 */
void tracking_channel_get_values(tracker_channel_id_t id,
                                 tracking_channel_info_t *info,
                                 tracking_channel_time_info_t *time_info,
                                 tracking_channel_freq_info_t *freq_info,
                                 tracking_channel_ctrl_info_t *ctrl_params,
                                 tracking_channel_misc_info_t *misc_params) {
  tracker_channel_t *tracker_channel = tracker_get(id);
  tracker_channel_pub_data_t *pub_data = &tracker_channel->pub_data;

  chMtxLock(&tracker_channel->mutex_pub);
  if (NULL != info) {
    *info = pub_data->gen_info;
  }
  if (NULL != time_info) {
    *time_info = pub_data->time_info;
  }
  if (NULL != freq_info) {
    *freq_info = pub_data->freq_info;
  }
  if (NULL != ctrl_params) {
    *ctrl_params = pub_data->ctrl_info;
  }
  if (NULL != misc_params) {
    *misc_params = pub_data->misc_info;
  }
  chMtxUnlock(&tracker_channel->mutex_pub);
}

/**
 * Atomically updates carrier phase offset.
 *
 * The method locates tracking channel object, locks it, and updates the
 * carrier phase offset only if the channel is still active, belongs to the
 * same signal and has the same lock counter.
 *
 * \param[in] info                 Generic tracking channel information block
 *                                 used for locating destination channel and
 *                                 checking integrity.
 * \param[in] carrier_phase_offset Carrier phase offset to set.
 *
 * \return None
 */
void tracking_channel_set_carrier_phase_offset(
    const tracking_channel_info_t *info, double carrier_phase_offset) {
  bool adjusted = false;
  tracker_channel_t *tracker_channel = tracker_get(info->id);
  tracker_channel_pub_data_t *pub_data = &tracker_channel->pub_data;

  chMtxLock(&tracker_channel->mutex_pub);
  if (0 != (pub_data->gen_info.flags & TRACKER_FLAG_ACTIVE) &&
      mesid_is_equal(info->mesid, pub_data->gen_info.mesid) &&
      info->lock_counter == pub_data->gen_info.lock_counter) {
    pub_data->misc_info.carrier_phase_offset.value = carrier_phase_offset;
    pub_data->misc_info.carrier_phase_offset.timestamp_ms = timing_getms();
    adjusted = true;
  }
  chMtxUnlock(&tracker_channel->mutex_pub);

  if (adjusted) {
    log_debug_mesid(info->mesid,
                    "Adjusting carrier phase offset to %lf",
                    carrier_phase_offset);
  }
}
/**
 * Computes the lock time from tracking channel time info.
 *
 * \param[in]  time_info Time information block.
 * \param[in]  misc_info Miscellaneous information block.
 *
 * \return Lock time [s]
 */
double tracking_channel_get_lock_time(
    const tracking_channel_time_info_t *time_info,
    const tracking_channel_misc_info_t *misc_info) {
  u64 cpo_age_ms = 0;
  if (0 != misc_info->carrier_phase_offset.value) {
    u64 now_ms = timing_getms();
    assert(now_ms >= misc_info->carrier_phase_offset.timestamp_ms);
    cpo_age_ms = now_ms - misc_info->carrier_phase_offset.timestamp_ms;
  }

  u64 lock_time_ms = UINT64_MAX;

  lock_time_ms = MIN(lock_time_ms, time_info->ld_pess_locked_ms);
  lock_time_ms = MIN(lock_time_ms, cpo_age_ms);

  return (double)lock_time_ms / SECS_MS;
}

/**
 * Loads data relevant to cross-correlation processing
 *
 * The method loads information from all trackers for cross-correlation
 * algorithm.
 *
 * \param[out] cc_data Destination container
 *
 * \return Number of entries loaded
 *
 * \sa tracking_channel_cc_data_t
 */
u16 tracking_channel_load_cc_data(tracking_channel_cc_data_t *cc_data) {
  u16 cnt = 0;

  for (tracker_channel_id_t id = 0; id < NUM_TRACKER_CHANNELS; ++id) {
    tracker_channel_t *tracker_channel = tracker_get(id);
    tracking_channel_cc_entry_t entry;

    entry.id = id;
    entry.mesid = tracker_channel->mesid;
    entry.flags = tracker_channel->flags;
    entry.freq = tracker_channel->xcorr_freq;
    entry.cn0 = tracker_channel->cn0;

    if (0 != (entry.flags & TRACKER_FLAG_ACTIVE) &&
        0 != (entry.flags & TRACKER_FLAG_CONFIRMED) &&
        0 != (entry.flags & TRACKER_FLAG_XCORR_FILTER_ACTIVE)) {
      cc_data->entries[cnt++] = entry;
    }
  }

  return cnt;
}

/**
 * Converts tracking channel data blocks into channel measurement structure.
 *
 * The method populates measurement fields according to provided values.
 *
 * \param[in]  ref_tc    Reference timing count.
 * \param[in]  info      Generic tracking channel information block.
 * \param[in]  freq_info Frequency and phase information block.
 * \param[in]  time_info Time information block.
 * \param[in]  misc_info Miscellaneous information block.
 * \param[out] meas      Pointer to output channel_measurement_t.
 *
 * \return None
 */
void tracking_channel_measurement_get(
    u64 ref_tc,
    const tracking_channel_info_t *info,
    const tracking_channel_freq_info_t *freq_info,
    const tracking_channel_time_info_t *time_info,
    const tracking_channel_misc_info_t *misc_info,
    channel_measurement_t *meas) {
  /* Update our channel measurement. */
  memset(meas, 0, sizeof(*meas));

  meas->sid = mesid2sid(info->mesid, info->glo_orbit_slot);
  meas->code_phase_chips = freq_info->code_phase_chips;
  meas->code_phase_rate = freq_info->code_phase_rate;
  meas->carrier_phase = freq_info->carrier_phase;
  meas->carrier_freq = freq_info->carrier_freq;
  meas->time_of_week_ms = info->tow_ms;
  meas->tow_residual_ns = info->tow_residual_ns;

  meas->rec_time_delta = (double)((s32)(info->sample_count - (u32)ref_tc)) /
                         NAP_FRONTEND_SAMPLE_RATE_Hz;

  meas->cn0 = info->cn0;
  meas->lock_time = tracking_channel_get_lock_time(time_info, misc_info);
  meas->time_in_track = time_info->cn0_usable_ms / 1000.0;
  meas->elevation = TRACKING_ELEVATION_UNKNOWN;
  meas->flags = 0;
}

/**
 * Computes raw pseudorange in [m]
 *
 * \param[in]  ref_tc Reference time
 * \param[in]  meas   Pre-populated channel measurement
 * \param[out] raw_pseudorange Computed pseudorange [m]
 *
 * \retval true Pseudorange is valid
 * \retval false Error in computation.
 */
bool tracking_channel_calc_pseudorange(u64 ref_tc,
                                       const channel_measurement_t *meas,
                                       double *raw_pseudorange) {
  navigation_measurement_t nav_meas, *p_nav_meas = &nav_meas;
  gps_time_t rec_time = napcount2gpstime(ref_tc);
  s8 nm_ret = calc_navigation_measurement(1, &meas, &p_nav_meas, &rec_time);
  if (nm_ret != 0) {
    log_warn_sid(meas->sid,
                 "calc_navigation_measurement() returned an error: %" PRId8,
                 nm_ret);
    return false;
  }
  *raw_pseudorange = nav_meas.raw_pseudorange;
  return true;
}

/** Adjust all carrier phase offsets with a receiver clock correction.
 * Note that as this change to carrier is equal to the change caused to
 * pseudoranges by the clock correction, the code-carrier difference does
 * not change and thus we do not reset the lock counter.
 *
 * \param dt      Receiver clock change (s)
 */
void tracking_channel_carrier_phase_offsets_adjust(double dt) {
  /* Carrier phase offsets are adjusted for all signals matching SPP criteria */
  for (u8 i = 0; i < nap_track_n_channels; i++) {
    me_gnss_signal_t mesid;
    double carrier_phase_offset = 0.0;
    bool adjusted = false;

    tracker_channel_t *tracker_channel = tracker_get(i);
    tracker_channel_pub_data_t *pub_data = &tracker_channel->pub_data;
    volatile tracking_channel_misc_info_t *misc_info = &pub_data->misc_info;

    chMtxLock(&tracker_channel->mutex_pub);
    if (0 != (pub_data->gen_info.flags & TRACKER_FLAG_ACTIVE)) {
      carrier_phase_offset = misc_info->carrier_phase_offset.value;

      /* touch only channels that have the initial offset set */
      if (carrier_phase_offset != 0.0) {
        mesid = pub_data->gen_info.mesid;
        carrier_phase_offset -= mesid_to_carr_freq(mesid) * dt;
        misc_info->carrier_phase_offset.value = carrier_phase_offset;
        /* Note that because code-carrier difference does not change here,
         * we do not reset the lock time carrier_phase_offset.timestamp_ms */
        adjusted = true;
      }
    }
    chMtxUnlock(&tracker_channel->mutex_pub);

    if (adjusted) {
      log_debug_mesid(
          mesid, "Adjusting carrier phase offset to %f", carrier_phase_offset);
    }
  }
}

/** Utility function to find tracking channel allocated to the given mesid.
 *
 * \param[in] mesid ME signal identifier.
 *
 * \return tracker channel container for the requested mesid.
 */
tracker_channel_t *tracker_channel_get_by_mesid(const me_gnss_signal_t mesid) {
  for (u8 i = 0; i < nap_track_n_channels; i++) {
    tracker_channel_t *tracker_channel = tracker_get(i);
    if (mesid_is_equal(tracker_channel->mesid, mesid)) {
      return tracker_channel;
    }
  }
  return NULL;
}

/** Drop unhealthy GLO signal.
 *
 *  Both L1CA and L2CA decode the health information independently.
 *  In case one channel does not contain valid data,
 *  it cannot detect unhealthy status.
 *
 *  If one channel is marked unhealthy,
 *  then also drop the other channel.
 *
 *  This function is called from both GLO L1 and L2 trackers.
 *
 * \param[in] mesid ME signal to be dropped.
 *
 * \return None
 */
void tracking_channel_drop_unhealthy_glo(const me_gnss_signal_t mesid) {
  assert(IS_GLO(mesid));
  tracker_channel_t *tracker_channel = tracker_channel_get_by_mesid(mesid);
  if (tracker_channel == NULL) {
    return;
  }
  /* Double-check that channel is in enabled state.
   * Similar check exists in manage_track() in manage.c
   */
  if (STATE_ENABLED != tracker_state_get(tracker_channel)) {
    return;
  }
  tracker_channel->flags |= TRACKER_FLAG_GLO_HEALTH_DECODED;
  tracker_channel->health = GLO_SV_UNHEALTHY;
}

/**
 * Check validity of handover code phase.
 *
 * The code phase is expected to be near zero at the moment of handover.
 * If the code has rolled over just recently, then it is [0, TOLERANCE]
 * If the code hasn't rolled over yet, then it is [MAX_CHIPS - TOL, MAX_CHIPS)
 * Code phase should never be negative.
 *
 * \param[in] code_phase_chips code phase [chips].
 * \param[in] max_chips        code length of the signal
 *                             from where handover is done [chips].
 *
 * \return true if the code phase is valid, false otherwise.
 */
bool handover_valid(double code_phase_chips, double max_chips) {
  if ((code_phase_chips < 0) ||
      ((code_phase_chips > HANDOVER_CODE_PHASE_THRESHOLD) &&
       (code_phase_chips < (max_chips - HANDOVER_CODE_PHASE_THRESHOLD)))) {
    return false;
  }
  return true;
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
bool tracking_channel_nav_bit_get(tracker_channel_id_t id,
                                  nav_bit_fifo_element_t *nav_bit) {
  tracker_channel_t *tracker_channel = tracker_get(id);

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
static void data_sync(tracker_channel_id_t id, nav_data_sync_t *from_decoder) {
  assert(from_decoder);

  tracker_channel_t *tracker_channel = tracker_get(id);
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
void tracking_channel_data_sync(tracker_channel_id_t id,
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
void tracking_channel_glo_data_sync(tracker_channel_id_t id,
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
update_count_t update_count_diff(const tracker_channel_t *tracker_channel,
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

/** Lock a tracker channel for exclusive access.
 *
 * \param tracker_channel   Tracker channel to use.
 */
void tracker_lock(tracker_channel_t *tracker_channel) {
  chMtxLock(&tracker_channel->mutex);
}

/** Unlock a locked tracker channel.
 *
 * \param tracker_channel   Tracker channel to use.
 */
void tracker_unlock(tracker_channel_t *tracker_channel) {
  chMtxUnlock(&tracker_channel->mutex);
}

/** \} */
