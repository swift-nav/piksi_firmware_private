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

#include "track.h"

#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>

#include <assert.h>
#include <ch.h>

#include "decode.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "signal.h"

#define GPS_WEEK_LENGTH_ms (1000 * WEEK_SECS)

static s32 normalize_tow(s32 tow) {
  assert(tow >= 0);
  return tow % GPS_WEEK_LENGTH_ms;
}

/** Register a tracker interface to enable tracking for a code type.
 *
 * \note element and all subordinate data must be statically allocated!
 *
 * \param element   Struct describing the interface to register.
 */
void tracker_interface_register(tracker_interface_list_element_t *element) {
  /* p_next = address of next pointer which must be updated */
  tracker_interface_list_element_t **p_next = tracker_interface_list_ptr_get();

  while (*p_next != 0) p_next = &(*p_next)->next;

  element->next = 0;
  *p_next = element;
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

/** Write the NAP update register for a tracker channel.
 *
 * \param[in]     tracker_channel Tracker channel data
 * \param chips_to_correlate  Number of code chips to integrate over.
 */
void tracker_retune(tracker_channel_t *tracker_channel,
                    u32 chips_to_correlate) {
  double doppler_freq_hz = tracker_channel->carrier_freq;
  double code_phase_rate = tracker_channel->code_phase_rate;

  /* Write NAP UPDATE register. */
  nap_track_update(tracker_channel->nap_channel,
                   doppler_freq_hz,
                   code_phase_rate,
                   chips_to_correlate,
                   0);
}

/** Adjust TOW for FIFO delay.
 *
 * \param[in] tracker_channel Tracker channel data
 * \param     to_tracker nav_data_sync_t struct to use
 *
 * \return Updated TOW (ms).
 */
static s32 adjust_tow_by_bit_fifo_delay(tracker_channel_t *tracker_channel,
                                        const nav_data_sync_t *to_tracker) {
  s32 TOW_ms = TOW_INVALID;
  /* Compute time since the pending data was read from the FIFO */
  nav_bit_fifo_index_t fifo_length = NAV_BIT_FIFO_INDEX_DIFF(
      tracker_channel->nav_bit_fifo.write_index, to_tracker->read_index);
  u32 fifo_time_diff_ms = fifo_length * tracker_channel->bit_sync.bit_length;

  /* Add full bit times + fractional bit time to the specified TOW */
  TOW_ms = to_tracker->TOW_ms + fifo_time_diff_ms +
           tracker_channel->nav_bit_TOW_offset_ms;

  TOW_ms = normalize_tow(TOW_ms);

  return TOW_ms;
}

static void update_polarity(tracker_channel_t *tracker_channel, s8 polarity) {
  me_gnss_signal_t mesid = tracker_channel->mesid;
  s8 prev_polarity = tracker_channel->bit_polarity;
  if (prev_polarity != polarity) {
    /* Print warning if there was an unexpected polarity change */
    if (BIT_POLARITY_UNKNOWN != tracker_channel->bit_polarity) {
      log_warn_mesid(mesid, "Unexpected bit polarity change");
    }
    /* Reset carrier phase offset on bit polarity change */
    tracker_channel->reset_cpo = true;
    tracker_channel->bit_polarity = polarity;
  }
}

static void update_tow(tracker_channel_t *tracker_channel,
                       const nav_data_sync_t *data_sync,
                       s32 *current_TOW_ms,
                       s32 *TOW_residual_ns,
                       bool *decoded_tow) {
  me_gnss_signal_t mesid = tracker_channel->mesid;

  s32 TOW_ms = adjust_tow_by_bit_fifo_delay(tracker_channel, data_sync);

  /* Warn if updated TOW does not match the current value */
  if ((*current_TOW_ms != TOW_INVALID) && (*current_TOW_ms != TOW_ms)) {
    log_error_mesid(
        mesid, "TOW mismatch: %" PRId32 ", %" PRId32, *current_TOW_ms, TOW_ms);
    /* This is rude, but safe. Do not expect it to happen normally. */
    tracker_channel->flags |= TRACKER_FLAG_OUTLIER;
  }
  *current_TOW_ms = TOW_ms;
  *decoded_tow = (TOW_ms >= 0);
  *TOW_residual_ns = data_sync->TOW_residual_ns;
}

static void update_eph(tracker_channel_t *tracker_channel,
                       const nav_data_sync_t *data_sync) {
  me_gnss_signal_t mesid = tracker_channel->mesid;

  if ((GLO_ORBIT_SLOT_UNKNOWN != tracker_channel->glo_orbit_slot) &&
      (tracker_channel->glo_orbit_slot != data_sync->glo_orbit_slot)) {
    log_warn_mesid(mesid, "Unexpected GLO orbit slot change");
  }
  tracker_channel->glo_orbit_slot = data_sync->glo_orbit_slot;
  tracker_channel->health = data_sync->glo_health;
}

/** Update the TOW for a tracker channel.
 *
 * \param tracker_channel      Tracker channel.
 * \param current_TOW_ms       Current TOW (ms).
 * \param int_ms               Integration period (ms).
 * \param[out] TOW_residual_ns TOW residual [ns]
 * \param[out] decoded_tow     Decoded TOW indicator
 * \param[out] decoded_health  Decoded health indicator
 *
 * \return Updated TOW (ms).
 */
s32 tracker_tow_update(tracker_channel_t *tracker_channel,
                       s32 current_TOW_ms,
                       u32 int_ms,
                       s32 *TOW_residual_ns,
                       bool *decoded_tow,
                       bool *decoded_health) {
  assert(tracker_channel);
  assert(TOW_residual_ns);
  assert(decoded_tow);

  /* Latch TOW from nav message if pending */

  nav_data_sync_t to_tracker;
  *decoded_tow = false;
  if (nav_data_sync_get(&to_tracker, &tracker_channel->nav_data_sync)) {
    decode_sync_flags_t flags = to_tracker.sync_flags;

    if (0 != (flags & SYNC_POL)) {
      update_polarity(tracker_channel, to_tracker.bit_polarity);
      update_bit_polarity_flags(tracker_channel);
    }

    if (0 != (flags & SYNC_TOW)) {
      update_tow(tracker_channel,
                 &to_tracker,
                 &current_TOW_ms,
                 TOW_residual_ns,
                 decoded_tow);
    }

    if (0 != (flags & SYNC_EPH)) {
      update_eph(tracker_channel, &to_tracker);
      *decoded_health = true;
    }
  }

  tracker_channel->nav_bit_TOW_offset_ms += int_ms;

  if (current_TOW_ms != TOW_INVALID) {
    /* Have a valid time of week - increment it. */
    current_TOW_ms += int_ms;
    current_TOW_ms = normalize_tow(current_TOW_ms);
    /* TODO: maybe keep track of week number in channel state, or
       derive it from system time */
  }

  return current_TOW_ms;
}

/** Set bit sync phase reference
 *
 * \param tracker_channel   Tracker channel data.
 * \param bit_phase_ref     Bit phase reference.
 */
void tracker_bit_sync_set(tracker_channel_t *tracker_channel,
                          s8 bit_phase_ref) {
  bit_sync_t *bit_sync = &tracker_channel->bit_sync;
  bit_sync_set(bit_sync, bit_phase_ref);
}

/** Update bit sync and output navigation message bits for a tracker channel.
 *
 * \param[in] tracker_channel Tracker channel data
 * \param int_ms            Integration period (ms).
 * \param corr_prompt_real  Real part of the prompt correlation.
 * \param sensitivity_mode  Flag indicating tracking channel sensitivity mode.
 */
void tracker_bit_sync_update(tracker_channel_t *tracker_channel,
                             u32 int_ms,
                             s32 corr_prompt_real,
                             s32 corr_prompt_imag,
                             bool sensitivity_mode) {
  /* Update bit sync */
  s32 bit_integrate;
  bool integrated = bit_sync_update(&tracker_channel->bit_sync,
                                    corr_prompt_real,
                                    corr_prompt_imag,
                                    int_ms,
                                    &bit_integrate);

  if (BITSYNC_UNSYNCED == tracker_channel->bit_sync.bit_phase_ref) {
    tracker_channel->flags &= ~TRACKER_FLAG_BIT_SYNC;
  } else {
    tracker_channel->flags |= TRACKER_FLAG_BIT_SYNC;
  }

  me_gnss_signal_t mesid = tracker_channel->mesid;
  if (!integrated || !code_requires_decoder(mesid.code)) {
    return;
  }

  s8 soft_bit = nav_bit_quantize(bit_integrate);

  /* write to FIFO */
  nav_bit_fifo_element_t element = {.soft_bit = soft_bit,
                                    .sensitivity_mode = sensitivity_mode};
  if (nav_bit_fifo_write(&tracker_channel->nav_bit_fifo, &element)) {
    /* warn if the FIFO has become full */
    if (nav_bit_fifo_full(&tracker_channel->nav_bit_fifo)) {
      log_warn_mesid(mesid, "nav bit FIFO full");
    }
  }

  /* clear nav bit TOW offset */
  tracker_channel->nav_bit_TOW_offset_ms = 0;
}

/** Get the bit length for a tracker channel.
 *
 * \param tracker_channel  Tracker channel data.
 *
 * \return Bit length
 */
u8 tracker_bit_length_get(tracker_channel_t *tracker_channel) {
  return tracker_channel->bit_sync.bit_length;
}

/** Get the bit alignment state for a tracker channel.
 *
 * \param[in] tracker_channel Tracker channel data
 *
 * \return true if bit sync has been established and the most recent
 *         integration is bit aligned, false otherwise.
 */
bool tracker_bit_aligned(tracker_channel_t *tracker_channel) {
  return (tracker_channel->bit_sync.bit_phase ==
          tracker_channel->bit_sync.bit_phase_ref);
}

/**
 * Tests if the bit sync is established in a tracker channel.
 *
 * \param[in] tracker_channel Tracker channel data
 * \retval true  Bit sync has been established
 * \retval false Bit sync is not established.
 */
bool tracker_has_bit_sync(tracker_channel_t *tracker_channel) {
  return (BITSYNC_UNSYNCED != tracker_channel->bit_sync.bit_phase_ref);
}

/**
 * Tests if the bit sync is established in a tracker channel.
 *
 * \param[in] tracker_channel Tracker channel data
 * \param int_ms      Next integration period in milliseconds.
 *
 * \retval true  Bit sync has been established and the next integration is bit
 *               aligned.
 * \retval false bit sync is not established or the next integration is not bit
 *               aligned.
 */
bool tracker_next_bit_aligned(tracker_channel_t *tracker_channel, u32 int_ms) {
  s32 next_bit_phase = tracker_channel->bit_sync.bit_phase + int_ms;
  next_bit_phase %= tracker_channel->bit_sync.bit_length;

  return (next_bit_phase == tracker_channel->bit_sync.bit_phase_ref);
}

/** Sets a channel's carrier phase ambiguity to unknown.
 * Changes the lock counter to indicate to the consumer of the tracking channel
 * observations that the carrier phase ambiguity may have changed. Also
 * invalidates the half cycle ambiguity, which must be resolved again by the
 * navigation
 *  message processing. Should be called if a cycle slip is suspected.
 *
 * \param[in] tracker_channel Tracker channel data
 */
void tracker_ambiguity_unknown(tracker_channel_t *tracker_channel) {
  tracker_channel->bit_polarity = BIT_POLARITY_UNKNOWN;
  tracker_channel->lock_counter =
      tracking_lock_counter_increment(tracker_channel->mesid);
  tracker_channel->reset_cpo = true;
  update_bit_polarity_flags(tracker_channel);
}

/** Checks channel's carrier phase ambiguity status.
 *
 * \param[in] tracker_channel Tracker channel data
 *
 * \return false if ambiguity unknown, true if it is known.
 */
bool tracker_ambiguity_resolved(tracker_channel_t *tracker_channel) {
  return tracker_channel->bit_polarity != BIT_POLARITY_UNKNOWN;
}

/** Set channel's carrier phase ambiguity status.
 *
 * \param[in] tracker_channel Tracker channel data
 * \param polarity Polarity of the half-cycle ambiguity
 *
 * \return None
 */
void tracker_ambiguity_set(tracker_channel_t *tracker_channel, s8 polarity) {
  if (BIT_POLARITY_UNKNOWN == polarity) {
    return;
  }
  tracker_channel->bit_polarity = polarity;
  update_bit_polarity_flags(tracker_channel);
}

/** Get the channel's GLO orbital slot information.
 *
 * \param[in] tracker_channel Tracker channel data
 *
 * \return GLO orbital slot
 */
u16 tracker_glo_orbit_slot_get(tracker_channel_t *tracker_channel) {
  return tracker_channel->glo_orbit_slot;
}

/** Get the channel's GLO health information.
 *
 * \param tracker_channel  Tracker channel.
 *
 * \return GLO health information
 */
glo_health_t tracker_glo_sv_health_get(tracker_channel_t *tracker_channel) {
  assert(IS_GLO(tracker_channel->mesid));
  return tracker_channel->health;
}

/** Output a correlation data message for a tracker channel.
 *
 * \param[in] tracker_channel Tracker channel data
 * \param cs          Array of correlations to send.
 */
void tracker_correlations_send(tracker_channel_t *tracker_channel,
                               const corr_t *cs) {
  /* Output I/Q correlations using SBP if enabled for this channel */
  if (tracker_channel->output_iq) {
    msg_tracking_iq_t msg = {
        .channel = tracker_channel->nap_channel,
    };
    /* TODO GLO: Handle GLO orbit slot properly. */
    if (IS_GLO(tracker_channel->mesid)) {
      return;
    }
    gnss_signal_t sid =
        mesid2sid(tracker_channel->mesid, tracker_channel->glo_orbit_slot);
    msg.sid = sid_to_sbp(sid);
    for (u32 i = 0; i < 3; i++) {
      msg.corrs[i].I = cs[i].I;
      msg.corrs[i].Q = cs[i].Q;
    }
    sbp_send_msg(SBP_MSG_TRACKING_IQ, sizeof(msg), (u8 *)&msg);
  }
}

/**
 * The function checks if PRN fail (decoded prn from L2C data stream
 * is not correspond to SVID) flag set or not.
 * Called from Tracking task.
 * \param[in] tracker_channel Tracker channel data
 * \return    TRUE if PRN fail flag is set, otherwise FAIL
 */
bool tracker_check_prn_fail_flag(tracker_channel_t *tracker_channel) {
  return tracker_channel->prn_check_fail;
}

/**
 * Checks if the tracker has cross-correlation flag set.
 *
 * Tracker can use this method to check if a cross-correlation flag is set by
 * external thread.
 *
 * \param[in] tracker_channel Tracker channel data
 *
 * \return Cross-correlation flag value-
 */
bool tracker_check_xcorr_flag(tracker_channel_t *tracker_channel) {
  return tracker_channel->xcorr_flag;
}
