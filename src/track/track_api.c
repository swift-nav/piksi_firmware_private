/*
 * Copyright (C) 2016 - 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "track_api.h"

#include <assert.h>
#include <ch.h>
#include <inttypes.h>
#include <swiftnav/constants.h>
#include <swiftnav/logging.h>

#include "board/nap/track_channel.h"
#include "decode/decode.h"
#include "sbp/sbp.h"
#include "sbp/sbp_utils.h"
#include "signal_db/signal_db.h"
#include "track_flags.h"

#define GPS_WEEK_LENGTH_ms (1000 * WEEK_SECS)

/* signal lock counter
 * A map of signal to an initially random number that increments each time that
 * signal begins being tracked.
 */
static u16 tracking_lock_counters[PLATFORM_ACQ_TRACK_COUNT];

static s32 normalize_tow(s32 tow) {
  assert(tow >= 0);
  return tow % GPS_WEEK_LENGTH_ms;
}

/** Read correlations from the NAP for a tracker channel.
 *
 * \param nap_channel     NAP tracking channel.
 * \param cs              Output array of correlations.
 * \param sample_count    Output sample count.
 * \param code_phase      Output code phase (chips).
 * \param carrier_phase   Output carrier phase (cycles).
 */
void tracker_correlations_read(u8 nap_channel,
                               corr_t *cs,
                               u32 *sample_count,
                               double *code_phase,
                               double *carrier_phase) {
  /* Read NAP CORR register */
  nap_track_read_results(
      nap_channel, sample_count, cs, code_phase, carrier_phase);
}

bool nap_sc_wipeoff(const tracker_t *tracker) {
  const code_t code = tracker->mesid.code;

  return (is_gal(code) || is_bds2(code)) && tracker_has_bit_sync(tracker);
}

/** Write the NAP update register for a tracker channel.
 *
 * \param[in]     tracker Tracker channel data
 * \param chips_to_correlate  Number of code chips to integrate over.
 */
void tracker_retune(tracker_t *tracker, u32 chips_to_correlate) {
  double doppler_hz = tracker->doppler_hz;
  double code_phase_rate = tracker->code_phase_rate;
  bool nap_strip_sc = nap_sc_wipeoff(tracker);
  bool narrow_correlators = tracker_has_bit_sync(tracker);
  /* Write NAP UPDATE register. */
  nap_track_update(tracker->nap_channel,
                   doppler_hz,
                   code_phase_rate,
                   chips_to_correlate,
                   nap_strip_sc,
                   narrow_correlators);
}

/** Adjust TOW for FIFO delay.
 *
 * \param[in] tracker Tracker channel data
 * \param     to_tracker nav_data_sync_t struct to use
 *
 * \return Updated TOW (ms).
 */
static s32 adjust_tow_by_bit_fifo_delay(tracker_t *tracker,
                                        const nav_data_sync_t *to_tracker) {
  s32 TOW_ms = TOW_INVALID;
  /* Compute time since the pending data was read from the FIFO */
  u8 fifo_length = nav_bit_fifo_length_for_rd_index(&tracker->nav_bit_fifo,
                                                    to_tracker->read_index);
  u32 fifo_time_diff_ms = fifo_length * tracker->bit_sync.bit_length;

  /* Add full bit times + fractional bit time to the specified TOW */
  TOW_ms = to_tracker->TOW_ms + fifo_time_diff_ms + tracker->nav_bit_offset_ms;

  TOW_ms = normalize_tow(TOW_ms);

  return TOW_ms;
}

static void update_polarity(tracker_t *tracker, s8 polarity) {
  me_gnss_signal_t mesid = tracker->mesid;
  s8 prev_polarity = tracker->bit_polarity;
  if (prev_polarity != polarity) {
    /* Print warning if there was an unexpected polarity change */
    if (BIT_POLARITY_UNKNOWN != tracker->bit_polarity) {
      log_warn_mesid(mesid, "Unexpected bit polarity change");
    }
    tracker->bit_polarity = polarity;
  }
}

static void update_tow(tracker_t *tracker,
                       const nav_data_sync_t *data_sync,
                       s32 *current_TOW_ms,
                       s32 *TOW_residual_ns,
                       bool *decoded_tow) {
  me_gnss_signal_t mesid = tracker->mesid;

  s32 TOW_ms = adjust_tow_by_bit_fifo_delay(tracker, data_sync);

  /* Warn if updated TOW does not match the current value */
  if ((*current_TOW_ms != TOW_INVALID) && (*current_TOW_ms != TOW_ms)) {
    log_error_mesid(
        mesid, "TOW mismatch: %" PRId32 ", %" PRId32, *current_TOW_ms, TOW_ms);
    /* This is rude, but safe. Do not expect it to happen normally. */
    tracker_flag_drop(tracker, CH_DROP_REASON_OUTLIER);
  }
  *current_TOW_ms = TOW_ms;
  *decoded_tow = (TOW_ms >= 0);
  *TOW_residual_ns = data_sync->TOW_residual_ns;
}

static void update_glo_string_sync(tracker_t *tracker,
                                   const nav_data_sync_t *sync) {
  u8 len = nav_bit_fifo_length_for_rd_index(&tracker->nav_bit_fifo,
                                            sync->read_index);
  u16 ms = (u16)len * tracker->bit_sync.bit_length;
  ms += tracker->nav_bit_offset_ms;
  ms %= GLO_STRING_LENGTH_MS;

  bool string_sync = (0 != (tracker->flags & TRACKER_FLAG_GLO_STRING_SYNC));
  if (string_sync && (tracker->glo_into_string_ms != ms)) {
    log_error_mesid(tracker->mesid,
                    "String sync failure: new %d, old %d",
                    ms,
                    tracker->glo_into_string_ms);
  }
  tracker->glo_into_string_ms = ms;
  tracker->flags |= TRACKER_FLAG_GLO_STRING_SYNC;
}

static void update_eph(tracker_t *tracker, const nav_data_sync_t *data_sync) {
  me_gnss_signal_t mesid = tracker->mesid;

  if ((GLO_ORBIT_SLOT_UNKNOWN != tracker->glo_orbit_slot) &&
      (tracker->glo_orbit_slot != data_sync->glo_orbit_slot)) {
    log_warn_mesid(mesid, "Unexpected GLO orbit slot change");
  }
  tracker->glo_orbit_slot = data_sync->glo_orbit_slot;
  if (!IS_GPS(mesid) && SV_UNHEALTHY == data_sync->health) {
    tracker->flags |= TRACKER_FLAG_UNHEALTHY;
    tracker_flag_drop(tracker, CH_DROP_REASON_SV_UNHEALTHY);
  }
}

/** Update the TOW for a tracker channel.
 *
 * \param tracker      Tracker channel.
 * \param current_TOW_ms       Current TOW (ms).
 * \param int_ms               Integration period (ms).
 * \param[out] TOW_residual_ns TOW residual [ns]
 * \param[out] decoded_tow     Decoded TOW indicator
 *
 * \return Updated TOW (ms).
 */
s32 tracker_tow_update(tracker_t *tracker,
                       s32 current_TOW_ms,
                       u32 int_ms,
                       s32 *TOW_residual_ns,
                       bool *decoded_tow) {
  assert(tracker);
  assert(TOW_residual_ns);
  assert(decoded_tow);

  /* Latch TOW from nav message if pending */

  nav_data_sync_t to_tracker;
  *decoded_tow = false;
  if (nav_data_sync_get(&to_tracker, &tracker->nav_data_sync)) {
    decode_sync_flags_t flags = to_tracker.sync_flags;

    if (0 != (flags & SYNC_POL)) {
      update_polarity(tracker, to_tracker.bit_polarity);
      tracker_update_bit_polarity_flags(tracker);
    }

    if (0 != (flags & SYNC_TOW)) {
      update_tow(
          tracker, &to_tracker, &current_TOW_ms, TOW_residual_ns, decoded_tow);
    }

    if (0 != (flags & SYNC_EPH)) {
      update_eph(tracker, &to_tracker);
    }

    if (0 != (flags & SYNC_GLO_STRING)) {
      update_glo_string_sync(tracker, &to_tracker);
    }
  }

  tracker->nav_bit_offset_ms += int_ms;
  tracker->glo_into_string_ms += int_ms;
  tracker->glo_into_string_ms %= GLO_STRING_LENGTH_MS;

  if (current_TOW_ms != TOW_INVALID) {
    /* Have a valid time of week - increment it. */
    current_TOW_ms += int_ms;
    current_TOW_ms = normalize_tow(current_TOW_ms);
    /* TODO: maybe keep track of week number in channel state, or
       derive it from system time */

    /* Get GLO string sync from existing TOW */
    bool string_sync = (0 != (tracker->flags & TRACKER_FLAG_GLO_STRING_SYNC));
    if (!string_sync) {
      u16 ms = current_TOW_ms % GLO_STRING_LENGTH_MS;
      tracker->glo_into_string_ms = ms;
      tracker->flags |= TRACKER_FLAG_GLO_STRING_SYNC;
    }
  }

  return current_TOW_ms;
}

/** Set bit sync phase reference
 *
 * \param tracker   Tracker channel data.
 * \param bit_phase_ref     Bit phase reference.
 */
void tracker_bit_sync_set(tracker_t *tracker, s8 bit_phase_ref) {
  bit_sync_t *bit_sync = &tracker->bit_sync;
  bit_sync_set(bit_sync, bit_phase_ref);
}

/** Compress a 32 bit integration value down to 8 bits.
 *
 * \param bit_integrate   Signed bit integration value.
 */
static s8 nav_bit_quantize(s32 bit_integrate) {
  /* compress s32 into a balanced s8, 0 reserved for sensitivity mode
   * We accumulate s16 I&Q correlators on s32 for at best 20 ms,
   * so the bits we can expect to exercise on `bit_integrate` are [22-16]
   * hence the reduced scaling which otherwise should be >> 25 */

  return (s8)(((bit_integrate >> 17) << 1) + 1);
}

/** Update bit sync and output navigation message bits for a tracker channel.
 *
 * \param[in] tracker Tracker channel data
 * \param int_ms            Integration period (ms).
 * \param sensitivity_mode  Flag indicating tracking channel sensitivity mode.
 */
void tracker_bit_sync_update(tracker_t *tracker,
                             u32 int_ms,
                             bool sensitivity_mode) {
  /* Update bit sync */
  s32 bit_integrate;
  bool integrated = bit_sync_update(&tracker->bit_sync,
                                    tracker->corrs.corr_bit.I,
                                    tracker->corrs.corr_bit.Q,
                                    int_ms,
                                    &bit_integrate);

  /* port sync information to flags */
  if (BITSYNC_UNSYNCED == tracker->bit_sync.bit_phase_ref) {
    tracker->flags &= ~TRACKER_FLAG_BIT_SYNC;
  } else {
    tracker->flags |= TRACKER_FLAG_BIT_SYNC;
  }

  me_gnss_signal_t mesid = tracker->mesid;
  if (!integrated || !code_requires_decoder(mesid.code)) {
    return;
  }

  if (CODE_GAL_E7I == mesid.code) {
    log_debug("E%02" PRIu8 " energy %+4" PRIi32 " %+4" PRIi32,
              mesid.sat,
              tracker->corrs.corr_cn0.I,
              tracker->corrs.corr_cn0.Q);
  }

  s8 soft_bit = nav_bit_quantize(bit_integrate);

  /* write to FIFO */
  nav_bit_t element = {
      .data = sensitivity_mode ? 0 : soft_bit,
      .cnt = tracker->bit_cnt,
  };
  tracker->bit_cnt++;
  if (nav_bit_fifo_write(&tracker->nav_bit_fifo, &element)) {
    /* warn if the FIFO has become full */
    if (nav_bit_fifo_full(&tracker->nav_bit_fifo)) {
      log_error_mesid(mesid, "nav bit FIFO full");
    }
  }

  /* clear nav bit offset */
  tracker->nav_bit_offset_ms = 0;
}

/** Get the bit length for a tracker channel.
 *
 * \param tracker  Tracker channel data.
 *
 * \return Bit length
 */
u8 tracker_bit_length_get(tracker_t *tracker) {
  return tracker->bit_sync.bit_length;
}

/** Get the bit alignment state for a tracker channel.
 *
 * \param[in] tracker Tracker channel data
 *
 * \return true if bit sync has been established and the most recent
 *         integration is bit aligned, false otherwise.
 */
bool tracker_bit_aligned(tracker_t *tracker) {
  return (tracker->bit_sync.bit_phase == tracker->bit_sync.bit_phase_ref);
}

/**
 * Tests if the bit sync is established in a tracker channel.
 *
 * \param[in] tracker Tracker channel data
 * \retval true  Bit sync has been established
 * \retval false Bit sync is not established.
 */
bool tracker_has_bit_sync(const tracker_t *tracker) {
  return (BITSYNC_UNSYNCED != tracker->bit_sync.bit_phase_ref);
}

/**
 * Checks if the tracker channel has all available locks.
 * Phase lock detector is not used in sensitivity profiles.
 * Frequency lock detector is not used in base station profiles.
 *
 * \param[in] tracker Tracker channel data
 * \retval true  Channel has all available locks
 * \retval false Otherwise
 */
bool tracker_has_all_locks(const tracker_t *tracker) {
  /* Mark pll_lock true if PLL is not in use, or we have pll lock. */
  bool pll_lock = ((0 == (tracker->flags & TRACKER_FLAG_PLL_USE)) ||
                   (0 != (tracker->flags & TRACKER_FLAG_HAS_PLOCK)));
  /* Mark fll_lock true if FLL is not in use, or we have fll lock. */
  bool fll_lock = ((0 == (tracker->flags & TRACKER_FLAG_FLL_USE)) ||
                   (0 != (tracker->flags & TRACKER_FLAG_HAS_FLOCK)));
  /* Check both locks. */
  return (pll_lock && fll_lock);
}

/**
 * Tests if the bit sync is established in a tracker channel.
 *
 * \param[in] tracker Tracker channel data
 * \param int_ms      Next integration period in milliseconds.
 *
 * \retval true  Bit sync has been established and the next integration is bit
 *               aligned.
 * \retval false bit sync is not established or the next integration is not bit
 *               aligned.
 */
bool tracker_next_bit_aligned(tracker_t *tracker, u32 int_ms) {
  s32 next_bit_phase = tracker->bit_sync.bit_phase + int_ms;
  next_bit_phase %= tracker->bit_sync.bit_length;

  return (next_bit_phase == tracker->bit_sync.bit_phase_ref);
}

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
static u16 tracking_lock_counter_increment(const me_gnss_signal_t mesid) {
  return ++tracking_lock_counters[mesid_to_global_index(mesid)];
}

/** Sets a channel's carrier phase ambiguity to unknown.
 * Changes the lock counter to indicate to the consumer of the tracking channel
 * observations that the carrier phase ambiguity may have changed. Also
 * invalidates the half cycle ambiguity, which must be resolved again by the
 * navigation message processing. To be called if a cycle slip is suspected.
 *
 * \param[in] tracker Tracker data
 */
void tracker_ambiguity_unknown(tracker_t *tracker) {
  tracker->bit_polarity = BIT_POLARITY_UNKNOWN;
  tracker->lock_counter = tracking_lock_counter_increment(tracker->mesid);
  tracker->reset_cpo = true;
  tracker_update_bit_polarity_flags(tracker);
}

/** Checks channel's carrier phase ambiguity status.
 *
 * \param[in] tracker Tracker channel data
 *
 * \return false if ambiguity unknown, true if it is known.
 */
bool tracker_ambiguity_resolved(tracker_t *tracker) {
  return tracker->bit_polarity != BIT_POLARITY_UNKNOWN;
}

/** Set channel's carrier phase ambiguity status.
 *
 * \param[in] tracker Tracker data
 * \param polarity Polarity of the half-cycle ambiguity
 *
 * \return None
 */
void tracker_ambiguity_set(tracker_t *tracker, s8 polarity) {
  if (BIT_POLARITY_UNKNOWN == polarity) {
    return;
  }
  tracker->bit_polarity = polarity;
  tracker_update_bit_polarity_flags(tracker);
}

/** Get the channel's GLO orbital slot information.
 *
 * \param[in] tracker Tracker channel data
 *
 * \return GLO orbital slot
 */
u16 tracker_glo_orbit_slot_get(tracker_t *tracker) {
  return tracker->glo_orbit_slot;
}

/** Output a correlation data message for a tracker channel.
 *
 * \param[in] tracker Tracker channel data
 * \param cs          Array of correlations to send.
 */
void tracker_correlations_send(tracker_t *tracker, const corr_t *cs) {
  /* Output I/Q correlations using SBP if enabled for this channel */
  if (tracker->output_iq) {
    msg_tracking_iq_t msg = {
        .channel = tracker->nap_channel,
    };
    /* TODO GLO: Handle GLO orbit slot properly. */
    if (IS_GLO(tracker->mesid)) {
      return;
    }
    gnss_signal_t sid = mesid2sid(tracker->mesid, tracker->glo_orbit_slot);
    msg.sid = sid_to_sbp(sid);
    for (u32 i = 0; i < 3; i++) {
      msg.corrs[i].I = cs[i].I;
      msg.corrs[i].Q = cs[i].Q;
    }
    sbp_send_msg(SBP_MSG_TRACKING_IQ, sizeof(msg), (u8 *)&msg);
  }
}

/** Lock a tracker channel for exclusive access.
 *
 * \param tracker   Tracker channel to use.
 */
void tracker_lock(tracker_t *tracker) { chMtxLock(&tracker->mutex); }

/** Unlock a locked tracker channel.
 *
 * \param tracker   Tracker channel to use.
 */
void tracker_unlock(tracker_t *tracker) { chMtxUnlock(&tracker->mutex); }
